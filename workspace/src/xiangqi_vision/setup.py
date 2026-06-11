from setuptools import setup
import os
from glob import glob

package_name = 'xiangqi_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xiangqi Team',
    maintainer_email='student@rmit.edu.au',
    description='Vision pipeline for Xiangqi robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = xiangqi_vision.vision_node:main',
            'calibration_tool = xiangqi_vision.calibration_tool:main',
            'vision_preprocess_experiment = xiangqi_vision.vision_preprocess_experiment:main',
        ],
    },
)
