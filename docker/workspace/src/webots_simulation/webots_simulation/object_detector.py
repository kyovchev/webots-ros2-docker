import rclpy
from geometry_msgs.msg import PoseStamped


class ObjectDetector:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot  # needs supervisor TRUE in the .wbt

        # Pre-declare everything so step() is safe even if init fails partway
        self.__node = None
        self.publisher = None
        self.target = None
        self.translation_field = None

        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('object_detector')

        self.publisher = self.__node.create_publisher(
            PoseStamped, '/detected_object', 1)

        timestep = int(self.__robot.getBasicTimeStep())
        self.camera = self.__robot.getDevice('camera')
        self.camera.enable(timestep)
        self.range_finder = self.__robot.getDevice('range-finder')
        self.range_finder.enable(timestep)

        self.target = self.__robot.getFromDef('TARGET_BOX')
        if self.target is None:
            self.__node.get_logger().error('TARGET_BOX not found')
            return
        self.translation_field = self.target.getField('translation')

    def step(self):
        if self.__node is None or self.translation_field is None:
            return

        rclpy.spin_once(self.__node, timeout_sec=0)

        pos = self.translation_field.getSFVec3f()

        msg = PoseStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
