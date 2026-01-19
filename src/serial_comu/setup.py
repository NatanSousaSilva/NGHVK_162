from setuptools import find_packages, setup

package_name = 'serial_comu'

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
    maintainer='natan',
    maintainer_email='natanssousassiva@gmail.com',
    description='Serial communication node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'serial_sender = serial_comu.serial_node:main'
        ],
    },
)
