# Need to do some trickery to get this thing to work with WSL
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'multirobot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seka',
    maintainer_email='aljkyt@utu.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_controller_new = multirobot_control.tello_controller_new:main',
            'tello_calculator = multirobot_control.tello_calculator:main',
            'tello_detection = multirobot_control.tello_detection:main',
            'create3_controller = multirobot_control.create3_controller:main',

        ],
    },
)
