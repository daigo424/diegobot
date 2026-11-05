from setuptools import find_packages, setup

package_name = 'amr_robot_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daigo424',
    maintainer_email='ishii.daigo@gmail.com',
    description="Server-side HTTP API for diegobot that lets an LLM agent (Dify) call the robot's search, approach, "
                'pick and place actions.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_tools = amr_robot_tools.robot_tools:main',
        ],
    },
)
