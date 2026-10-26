from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daigo424',
    maintainer_email='ishii.daigo@gmail.com',
    description='Server-side perception for diegobot: processes sensor data streamed from the robot.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_detector = amr_perception.object_detector:main',
            'export_engine = amr_perception.export_engine:main',
        ],
    },
)
