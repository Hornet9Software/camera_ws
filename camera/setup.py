import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. (includes both python and xml launch files)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shengbin',
    maintainer_email='shengbin.chan@gmail.com',
    description='Camera Package to perform Stereo Vision and Object Detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'debug_detect = camera.debug_detect:main',
            'enhance = camera.enhance:main',
           
            'lines = camera.lines:main',
            'stereo_vision = camera.stereo_vision:main',
            'export = camera.export:main',

            'qualification_gate = camera.qualification_gate:main',
            'gate_detecto = camera.gate_detecto:main',
            'gate_yolo = camera.gate_yolo:main',

            'flare = camera.flare:main',
        ],
    },
)
