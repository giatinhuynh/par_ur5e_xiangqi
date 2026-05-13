from setuptools import setup
import os
from glob import glob

package_name = 'xiangqi_ai'

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
    maintainer='Xiangqi Team',
    maintainer_email='student@rmit.edu.au',
    description='AI engines and game manager for Xiangqi robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_manager_node = xiangqi_ai.game_manager_node:main',
            'ai_engine_node = xiangqi_ai.ai_engine_node:main',
            'simple_ai_cycle_test = xiangqi_ai.simple_ai_cycle_test:main',
        ],
    },
)
