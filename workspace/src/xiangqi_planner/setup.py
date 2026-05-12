from setuptools import setup, find_packages

package_name = 'xiangqi_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'task_planner_node = xiangqi_planner.task_planner_node:main',
        ],
    },
)
