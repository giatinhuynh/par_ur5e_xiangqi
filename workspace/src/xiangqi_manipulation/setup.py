from setuptools import setup

package_name = 'xiangqi_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'manipulation_node = xiangqi_manipulation.manipulation_node:main',
            'gripper_controller_node = xiangqi_manipulation.gripper_controller_node:main',
            'safety_monitor_node = xiangqi_manipulation.safety_monitor_node:main',
        ],
    },
)
