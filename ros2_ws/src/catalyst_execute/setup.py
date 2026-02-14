from setuptools import find_packages, setup

package_name = 'catalyst_execute'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aman',
    maintainer_email='aman@todo.todo',
    description='Execution scripts for the Catalyst Manipulator',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_execute = catalyst_execute.test_execute:main',
        ],
    },
)
