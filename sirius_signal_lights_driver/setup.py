from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'sirius_signal_lights_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='klaudia',
    maintainer_email='klaudianiedzi@gmail.com',
    description='Sent can_msgs about lamps',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sent_msgs = sirius_signal_lights_driver.sent_msgs:main'
        ],
    },
    
)
