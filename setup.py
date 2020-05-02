from setuptools import setup
from os.path import join
from glob import glob

package_name = 'luos_bike_alarm_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages', ['resource/bike.rviz']),
        ('share/ament_index/resource_index/packages', ['resource/bike.stl']),
        (join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yoan Mollard',
    maintainer_email='yoan@aubrune.eu',
    description='Example of ROS package using Luos modules',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bike_alarm = luos_bike_alarm_example.bike_alarm:main'
        ],
    },
)
