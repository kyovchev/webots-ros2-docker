from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'webots_simulation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages',
                   ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                   ['launch/simulation.launch.py']))
data_files.append(('share/' + package_name + '/worlds',
                   ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource',
                   ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/resource',
                   ['resource/object_detector.urdf']))
for root, dirs, files in os.walk('protos'):
    install_dir = os.path.join(
        'share',
        package_name,
        root
    )
    data_files.append((
        install_dir,
        [os.path.join(root, f) for f in files]
    ))
data_files.append(('share/' + package_name,
                   ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = webots_simulation.my_robot_driver:main',
            'object_detector = webots_simulation.object_detector:main',
            'scene_manipulator = webots_simulation.scene_manipulator:main',
            'ros_web_service = webots_simulation.ros_web_service:main'
        ],
    },
)
