from setuptools import find_packages
from setuptools import setup

setup(
    name='theora_image_transport',
    version='2.5.0',
    packages=find_packages(
        include=('theora_image_transport', 'theora_image_transport.*')),
)
