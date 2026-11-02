from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_behaviors'

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
    description='Server-side behaviors for diegobot that combine driving, the arm and the camera: '
                'searching for and approaching objects.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'behavior_server = amr_behaviors.behavior_server:main',
        ],
    },
)
