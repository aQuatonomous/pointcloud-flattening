import os
from glob import glob
from setuptools import setup

package_name = 'pointcloud_flattening'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sabrina Button',
    maintainer_email='sabrina.button@queensu.ca',
    description='A ROS package to flatten pointclouds.',
    license='MIT',
    tests_require=['pytest'],
)
