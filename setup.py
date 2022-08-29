from setuptools import setup

package_name = 'moveit2_dualshock_controller'

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
    maintainer='Mykyta Ielanskyi',
    maintainer_email='mykyta.ielanskyi@grandgarage.eu',
    description='PS4 Dualshock controller compatibility for moveit2 driven bots.',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'moveit2_dualshock_controller = moveit2_dualshock_controller.main:main',
            'dualshock_controller_node = moveit2_dualshock_controller.remote_controller:main',
            'gripper_control = moveit2_dualshock_controller.gripper_control_node:main'
        ],
    },
)
