import os
from glob import glob
from setuptools import setup

package_name = 'locomotion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hari Vikinesh',
    maintainer_email='harivikinesh@gmail.com',
    description='Navigation Controller for DDR and HDR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'keyboard_control_ddr = locomotion_control.keyboard_control_ddr:main',
        'keyboard_control_hdr = locomotion_control.keyboard_control_hdr:main',
        ],
    },
)
