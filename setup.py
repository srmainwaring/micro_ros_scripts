from setuptools import find_packages, setup

package_name = 'micro_ros_scripts'

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
    maintainer='Rhys Mainwaring',
    maintainer_email='rhys.mainwaring@me.com',
    description='Python scripts for the micro ROS examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'int32_publisher = micro_ros_scripts.int32_publisher:main',
        ],
    },
)
