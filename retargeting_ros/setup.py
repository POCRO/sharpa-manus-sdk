from setuptools import setup
import os
from glob import glob

package_name = 'retargeting_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mock_keypoints_publisher = retargeting_ros.mock_keypoints_publisher:main',
            'hand_action_bridge = retargeting_ros.hand_action_bridge:main',
        ],
    },
)
