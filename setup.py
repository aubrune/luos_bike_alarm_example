from setuptools import setup

package_name = 'luos_bike_alarm_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
