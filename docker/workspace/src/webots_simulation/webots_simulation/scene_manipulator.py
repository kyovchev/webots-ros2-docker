"""webots_ros2 plugin: REST API for moving objects in a Webots world.

The Webots Supervisor API may only be used from the thread that steps the
simulation. The HTTP server therefore runs in a background thread and hands
every request to step(), which executes it on the simulation thread.

Endpoints (JSON in / JSON out):
    GET  /objects                  top-level nodes that have a DEF name
    GET  /objects/<DEF>/pose       current position / orientation
    PUT  /objects/<DEF>/pose       set position and/or orientation (POST also works)

PUT body:
    {
      "position": [x, y, z],                 # optional, metres
      "rpy": [roll, pitch, yaw]              # optional orientation, radians
        or "quaternion": [x, y, z, w]
        or "axis_angle": [x, y, z, angle],   # Webots' native format
      "reset_physics": true                  # optional, default true
    }

The same operations are available over ROS 2:
    /scene/object_pose_cmd   geometry_msgs/PoseStamped (header.frame_id = DEF name)
    /scene/set_object_pose   webots_simulation_interfaces/srv/SetObjectPose
    /scene/get_object_pose   webots_simulation_interfaces/srv/GetObjectPose
An all-zero orientation quaternion (the message default) leaves the orientation unchanged.
"""

import json
import math
import queue
import threading
from concurrent.futures import Future, TimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from webots_simulation_interfaces.srv import GetObjectPose, SetObjectPose


# --------------------------------------------------------------------------
# Pose maths: Webots stores rotations as axis-angle, ROS uses quaternions
# --------------------------------------------------------------------------

def _vec(value, n, name):
    ok = isinstance(value, (list, tuple)) and len(value) == n
    if ok:
        for v in value:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                ok = False
    if not ok:
        raise ValueError(f"'{name}' must be a list of {n} finite numbers")
    return [float(v) for v in value]


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy]   # w


def _quat_to_axis_angle(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('quaternion must be non-zero')
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0:                                  # take the shortest rotation
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:                               # no rotation
        return [0.0, 0.0, 1.0, 0.0]
    return [x / s, y / s, z / s, 2.0 * math.acos(min(1.0, w))]


def _axis_angle_to_quat(a):
    x, y, z, angle = a
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    s = math.sin(angle / 2) / n
    return [x * s, y * s, z * s, math.cos(angle / 2)]


def _parse_pose_request(body):
    """Validate a PUT body (on the HTTP thread) and normalise it."""
    if not isinstance(body, dict):
        raise ValueError('request body must be a JSON object')

    cmd = {'reset_physics': bool(body.get('reset_physics', True))}
    if 'position' in body:
        cmd['position'] = _vec(body['position'], 3, 'position')

    given = [k for k in ('quaternion', 'rpy', 'axis_angle') if k in body]
    if len(given) > 1:
        raise ValueError("use only one of 'quaternion', 'rpy', 'axis_angle'")
    if given:
        key = given[0]
        if key == 'rpy':
            quat = _rpy_to_quat(*_vec(body[key], 3, key))
            cmd['axis_angle'] = _quat_to_axis_angle(quat)
        elif key == 'quaternion':
            cmd['axis_angle'] = _quat_to_axis_angle(_vec(body[key], 4, key))
        else:
            x, y, z, angle = _vec(body[key], 4, key)
            n = math.sqrt(x * x + y * y + z * z)
            if n < 1e-12:
                raise ValueError("'axis_angle' needs a non-zero axis")
            cmd['axis_angle'] = [x / n, y / n, z / n, angle]

    if 'position' not in cmd and 'axis_angle' not in cmd:
        raise ValueError(
            "give 'position' and/or one of 'quaternion', 'rpy', 'axis_angle'")
    return cmd


def _cmd_from_pose(pose, reset_physics=True):
    """Turn a geometry_msgs/Pose into the same command dict the REST layer builds."""
    p, q = pose.position, pose.orientation
    cmd = {'position': _vec([p.x, p.y, p.z], 3, 'position'),
           'reset_physics': reset_physics}
    quat = _vec([q.x, q.y, q.z, q.w], 4, 'orientation')
    if any(quat):  # all zeros = message default = keep the current orientation
        cmd['axis_angle'] = _quat_to_axis_angle(quat)
    return cmd


# --------------------------------------------------------------------------
# HTTP layer (runs in server threads, never touches the Supervisor directly)
# --------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):
    sim = None  # the SceneManipulator, injected when the server is created

    def log_message(self, fmt, *args):  # keep the launch log quiet
        pass

    def do_GET(self):
        self._handle('GET')

    def do_PUT(self):
        self._handle('PUT')

    do_POST = do_PUT

    def _handle(self, method):
        try:
            status, payload = self._route(method)
        except LookupError as exc:
            status, payload = 404, {'error': str(exc)}
        except ValueError as exc:  # also covers JSONDecodeError
            status, payload = 400, {'error': str(exc)}
        except TimeoutError:
            status, payload = 503, {
                'error': 'simulation did not respond (paused?)'}
        except Exception as exc:
            status, payload = 500, {'error': f'{type(exc).__name__}: {exc}'}

        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _route(self, method):
        parts = [unquote(p) for p in urlsplit(self.path).path.split('/') if p]

        if method == 'GET' and parts == ['objects']:
            return 200, self.sim.call(self.sim.list_objects)

        if len(parts) == 3 and parts[0] == 'objects' and parts[2] == 'pose':
            name = parts[1]
            if method == 'GET':
                return 200, self.sim.call(lambda: self.sim.get_pose(name))
            length = int(self.headers.get('Content-Length') or 0)
            cmd = _parse_pose_request(json.loads(
                self.rfile.read(length) or b'{}'))
            return 200, self.sim.call(lambda: self.sim.set_pose(name, cmd))

        raise LookupError(f'unknown route: {method} {self.path}')


# --------------------------------------------------------------------------
# The webots_ros2 plugin
# --------------------------------------------------------------------------

class SceneManipulator:
    REQUEST_TIMEOUT = 2.0            # seconds to wait for the simulation to run a request
    MAX_ROS_CALLBACKS_PER_STEP = 8   # topic/service callbacks handled per timestep

    def init(self, webots_node, properties):
        # Create everything step() touches first, so a failed init can never
        # turn into an AttributeError on every timestep.
        self._commands = queue.Queue()
        self._server = None
        self._node = None
        self._executor = None
        self._robot = webots_node.robot

        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node('scene_manipulator')
        log = node.get_logger()

        if not hasattr(self._robot, 'getFromDef'):
            log.error(
                "Robot is not a Supervisor: set 'supervisor TRUE' on its node in the .wbt")
            return

        # ROS interface. Callbacks are spun from step(), so they run on the
        # simulation thread and may call the Supervisor API directly.
        node.create_subscription(
            PoseStamped, '/scene/object_pose_cmd', self._on_pose_topic, 10)
        node.create_service(
            SetObjectPose, '/scene/set_object_pose', self._on_set_pose)
        node.create_service(
            GetObjectPose, '/scene/get_object_pose', self._on_get_pose)
        self._node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)

        host = properties.get('host', '0.0.0.0')
        port = int(properties.get('port', 8080))
        handler = type('SceneHandler', (_Handler,), {'sim': self})
        try:
            self._server = ThreadingHTTPServer((host, port), handler)
        except OSError as exc:
            log.error(f'Cannot start REST API on {host}:{port}: {exc}')
            return

        threading.Thread(target=self._server.serve_forever,
                         daemon=True).start()
        log.info(f'Scene REST API listening on http://{host}:{port}')

    # -- called from the HTTP threads ----------------------------------------

    def call(self, fn):
        """Run fn() on the simulation thread and return its result."""
        future = Future()
        self._commands.put((fn, future))
        try:
            return future.result(timeout=self.REQUEST_TIMEOUT)
        except TimeoutError:
            future.cancel()  # don't apply a stale request after a pause
            raise

    # -- called by webots_ros2_driver every timestep (simulation thread) -----

    def step(self):
        if self._executor is not None:  # ROS topic / service callbacks
            for _ in range(self.MAX_ROS_CALLBACKS_PER_STEP):
                self._executor.spin_once(timeout_sec=0)

        while True:
            try:
                fn, future = self._commands.get_nowait()
            except queue.Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(fn())
            except Exception as exc:
                future.set_exception(exc)

    # -- Supervisor operations (only ever run from step()) -------------------

    def _find(self, name):
        node = self._robot.getFromDef(name)
        if node is None:
            raise LookupError(f"no node with DEF name '{name}'")
        return node

    def list_objects(self):
        children = self._robot.getRoot().getField('children')
        objects = []
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node.getDef():
                objects.append(
                    {'name': node.getDef(), 'type': node.getTypeName()})
        return {'objects': objects}

    def get_pose(self, name):
        node = self._find(name)
        pose = {'name': name, 'type': node.getTypeName()}
        translation = node.getField('translation')
        rotation = node.getField('rotation')
        if translation is not None:
            pose['position'] = list(translation.getSFVec3f())
        if rotation is not None:
            axis_angle = list(rotation.getSFRotation())
            pose['axis_angle'] = axis_angle
            pose['quaternion'] = _axis_angle_to_quat(axis_angle)
        return pose

    def set_pose(self, name, cmd):
        node = self._find(name)
        translation = node.getField(
            'translation') if 'position' in cmd else None
        rotation = node.getField('rotation') if 'axis_angle' in cmd else None
        if ('position' in cmd and translation is None) or \
           ('axis_angle' in cmd and rotation is None):
            raise ValueError(
                f"'{name}' has no translation/rotation field to set")

        if translation is not None:
            translation.setSFVec3f(cmd['position'])
        if rotation is not None:
            rotation.setSFRotation(cmd['axis_angle'])
        if cmd['reset_physics']:
            node.resetPhysics()  # zero velocities so the object doesn't keep moving

        return {'name': name,
                'applied': {k: cmd[k] for k in ('position', 'axis_angle') if k in cmd}}

    # -- ROS callbacks (run from step(), so Supervisor calls are safe) -------

    def _on_pose_topic(self, msg):
        name = msg.header.frame_id
        try:
            self.set_pose(name, _cmd_from_pose(msg.pose))
        except Exception as exc:
            self._node.get_logger().warning(
                f"object_pose_cmd for '{name}' failed: {exc}")

    def _on_set_pose(self, request, response):
        try:
            cmd = _cmd_from_pose(
                request.pose, reset_physics=not request.keep_velocity)
            self.set_pose(request.name, cmd)
            response.success, response.message = True, 'ok'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_get_pose(self, request, response):
        try:
            pose = self.get_pose(request.name)
            if 'position' not in pose or 'quaternion' not in pose:
                raise ValueError(
                    f"'{request.name}' has no translation/rotation field")
            p, q = pose['position'], pose['quaternion']
            response.pose.position.x, response.pose.position.y, response.pose.position.z = p
            (response.pose.orientation.x, response.pose.orientation.y,
             response.pose.orientation.z, response.pose.orientation.w) = q
            response.success, response.message = True, 'ok'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response
