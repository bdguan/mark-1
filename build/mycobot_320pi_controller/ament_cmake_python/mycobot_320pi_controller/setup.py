from setuptools import find_packages
from setuptools import setup

setup(
    name='mycobot_320pi_controller',
    version='0.0.0',
    packages=find_packages(
        include=('mycobot_320pi_controller', 'mycobot_320pi_controller.*')),
)
