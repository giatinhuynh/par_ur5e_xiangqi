from setuptools import setup
import os
from glob import glob

package_name = 'xiangqi_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('xiangqi_dashboard/templates/*.html')),
        (os.path.join('share', package_name, 'static/css'), glob('xiangqi_dashboard/static/css/*.css')),
        (os.path.join('share', package_name, 'static/js'), glob('xiangqi_dashboard/static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'dashboard_node = xiangqi_dashboard.dashboard_node:main',
        ],
    },
)
