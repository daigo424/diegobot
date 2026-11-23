from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_teleop'

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
    description='Teleoperation of the diegobot arm with an SO-101 leader arm, and recording of the episodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_teleop = amr_teleop.leader_teleop:main',
            'record_episodes = amr_teleop.record_episodes:main',
            'check_arm = amr_teleop.check_arm:main',
            'head_slider = amr_teleop.head_slider:main',
            'policy_runner = amr_teleop.policy_runner:main',
        ],
    },
)
