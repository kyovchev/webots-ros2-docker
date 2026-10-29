#!/usr/bin/env python3
"""Demo client for the Webots scene REST API (scene_manipulator plugin).

    python3 scene_api_demo.py --url http://localhost:8080 --object TARGET_BOX

Lists the objects, reads the pose of one, teleports it, makes it orbit its
start position while spinning, then puts it back where it was.
Uses only the standard library.
"""

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import quote


class SceneApi:
    def __init__(self, base_url):
        self.base = base_url.rstrip('/')

    def _request(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(
            self.base + path, data=data, method=method,
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            try:
                detail = json.load(exc).get('error', exc.reason)
            except ValueError:
                detail = exc.reason
            raise RuntimeError(f'{method} {path} -> HTTP {exc.code}: {detail}') from None

    def list_objects(self):
        return self._request('GET', '/objects')['objects']

    def get_pose(self, name):
        return self._request('GET', f'/objects/{quote(name)}/pose')

    def set_pose(self, name, **pose):
        """pose: position=[x,y,z] and/or one of rpy / quaternion / axis_angle."""
        return self._request('PUT', f'/objects/{quote(name)}/pose', pose)


def main():
    parser = argparse.ArgumentParser(description='Demo for the Webots scene REST API')
    parser.add_argument('--url', default='http://localhost:8002')
    parser.add_argument('--object', default='TARGET_BOX', help='DEF name of the node to move')
    parser.add_argument('--radius', type=float, default=0.2, help='orbit radius in metres')
    parser.add_argument('--seconds', type=float, default=6.0, help='orbit duration')
    args = parser.parse_args()
    api = SceneApi(args.url)

    print('Objects in the world:')
    for obj in api.list_objects():
        print(f"  {obj['name']} ({obj['type']})")

    start = api.get_pose(args.object)
    print(f'Start pose of {args.object}: {start}')
    cx, cy, cz = start['position']

    try:
        # 1. Jump 0.3 m along +x and turn 90 degrees about z (orientation as roll/pitch/yaw).
        api.set_pose(args.object, position=[cx + 0.3, cy, cz], rpy=[0, 0, math.pi / 2])
        print('Moved +0.3 m in x, yaw 90 deg')
        time.sleep(1.0)

        # 2. Orbit the start position while spinning. Every request is applied on the
        #    next simulation step, so ~20 updates per second is plenty.
        print(f'Orbiting for {args.seconds:.0f} s ...')
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < args.seconds:
            angle = 2 * math.pi * t / args.seconds
            api.set_pose(args.object,
                         position=[cx + args.radius * math.cos(angle),
                                   cy + args.radius * math.sin(angle), cz],
                         rpy=[0, 0, angle])
            time.sleep(0.05)
    finally:
        # 3. Put it back using the values we read at the start (position + axis_angle).
        restore = {k: start[k] for k in ('position', 'axis_angle') if k in start}
        api.set_pose(args.object, **restore)
        print('Restored the original pose.')


if __name__ == '__main__':
    try:
        main()
    except urllib.error.URLError as exc:
        sys.exit(f'Cannot reach the API ({exc.reason}). Is the port published and the sim running?')
    except RuntimeError as exc:
        sys.exit(str(exc))
