from setuptools import find_packages
from setuptools import setup

setup(
    name='catalyst_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('catalyst_interfaces', 'catalyst_interfaces.*')),
)
