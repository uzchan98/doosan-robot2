
# Author: Chemin Ahn (chemx3937@gmail.com)
  
# Copyright (c) 2024 Doosan Robotics
# Use of this source code is governed by the BSD, see LICENSE

import glob
import os
from setuptools import find_packages, setup

package_name = 'dsr_visualservoing_ex'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*launch.py'))),
        (share_dir + '/config', glob.glob(os.path.join('config', '*yaml'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chemin Ahn',
    maintainer_email='chemx3937@gmail.com',
    description='Visual Servoing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                
                # Camera_publisher
                'camera_publisher = dsr_visualservoing_ex.camera_publisher:main',

                #Cobot pos: [0, 0, 90, 0, 90, 0]
                'joint90 = dsr_visualservoing_ex.joint90:main',

                # Visual Servoing @ Gazebo
                # 마커 좌표 인식 (Gazebo 용도)
                'detect_marker_gz = dsr_visualservoing_ex.detect_marker_gz:main',
                # 인식한 마커를 기반으로 Cobot 제어 (Gazebo 용도))
                'send_pose_servol_gz = dsr_visualservoing_ex.send_pose_servol_gz:main',


        ],
    },
)