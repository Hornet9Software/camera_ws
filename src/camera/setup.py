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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shengbin',
    maintainer_email='shengbin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = camera.cam_pub:main',
            'cam_sub = camera.cam_sub:main',
            'cam_oflow = camera.cam_oflow:main'
        ],
    },
)
