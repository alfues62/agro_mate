from setuptools import setup
import os
from glob import glob

package_name = 'agro_mate_nav2_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name), glob('test/*.py')) 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tallalsudo',
    maintainer_email='okabi.rintaro.2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = agro_mate_nav2_system.initial_pose_pub:main',
            'waypoint_follower = agro_mate_nav2_system.waypoint_follower:main',
            'to_pose = agro_mate_nav2_system.to_pose:main',
            'my_waypoints_follower = agro_mate_nav2_system.my_waypoints_follower:main',
            'service_waypoint_caller = agro_mate_nav2_system.service_waypoint_caller:main'
        ],
    },
)
