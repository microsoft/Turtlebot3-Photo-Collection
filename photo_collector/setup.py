import os
from glob import glob
from setuptools import setup

package_name = 'photo_collector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Abby Harrison',
    maintainer_email='abby.harrison@microsoft.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main' + ' = ' + package_name + '.' + 'main' + ':main',
        ],
    },
)
