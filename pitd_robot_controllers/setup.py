from setuptools import setup
import os
from glob import glob

package_name = 'pitd_robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', "*launch.[pxy][yma]*")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philip',
    maintainer_email='philip@randomsmiths.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller=pitd_robot_controllers.controller:main",
            "position=pitd_robot_controllers.position_handler:main"
        ],
    },
)
