from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daigo424',
    maintainer_email='ishii.daigo@gmail.com',
    description='Server-side arm actions for diegobot: picking objects found by the camera and placing them.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator = amr_manipulation.manipulator:main',
        ],
    },
)
