from setuptools import setup

package_name = 'pitd_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "world=pitd_controllers.world_controller:main",
            "defender=pitd_controllers.defender_controller:main",
            "attacker=pitd_controllers.attacker_controller:main",
            "pathing=pitd_controllers.pathing_controller:main"
        ],
    },
)
