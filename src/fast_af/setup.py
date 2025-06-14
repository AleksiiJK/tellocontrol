from setuptools import setup
import os
from glob import glob

package_name = 'fast_af'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eeavir',
    maintainer_email='eeavir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centroid_pid_sim = fast_af.centroid_pid_sim:main',
            'centroid_pid = fast_af.centroid_pid:main',
            'coordinate_controller = fast_af.coordinate_controller:main',
            'edge_detection_sim = fast_af.edge_detection_sim:main',
            'edge_detection = fast_af.edge_detection:main',
            'hsv_filter_node = fast_af.hsv_filter_node:main',
            'masked_area_calculator = fast_af.masked_area_calculator:main',       
        ],
    },
)
