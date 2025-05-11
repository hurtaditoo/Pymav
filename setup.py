from setuptools import setup

package_name = 'pymav'

setup(
    name=package_name,
    version='0.0.1',
    packages=['pymav'],
    py_modules=['pymav.mission_control_node', 'pymav.battery_gps_node'],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'std_msgs'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'battery_gps_node = pymav.battery_gps_node:main',
            'mission_control_node = pymav.mission_control_node:main',
        ],
    },
)
