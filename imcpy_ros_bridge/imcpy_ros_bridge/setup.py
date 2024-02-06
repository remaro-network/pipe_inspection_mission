import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imcpy_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olaya',
    maintainer_email='olaya_93@hotmail.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_single_reference_server = imcpy_ros_bridge.follow_single_reference_server:main',
            'follow_single_reference_client = imcpy_ros_bridge.follow_single_reference_client:main',
            'imc2ros = imcpy_ros_bridge.imc2ros:main',
            'ros2imc = imcpy_ros_bridge.ros2imc:main',
            'follow_reference_example = imcpy_ros_bridge.follow_reference_example:main'
        ],
    },
)

