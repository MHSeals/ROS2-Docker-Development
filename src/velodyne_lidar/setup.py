from setuptools import setup

package_name = 'velodyne_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fusha153',
    maintainer_email='pipandw@gmail.com',
    description='ROS2 package that subscribes to the Velodynes PointCloud topic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = velodyne_lidar.subscriber:main'
        ],
    },
)
