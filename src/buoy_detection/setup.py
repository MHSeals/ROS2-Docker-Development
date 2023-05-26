from setuptools import setup

package_name = 'buoy_detection'

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
    maintainer='Zutki',
    maintainer_email='45642687+Zutki@users.noreply.github.com',
    description='This returns the results from the MHSeals YOLO buoy detection model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai = buoy_detection.ai:main'
        ],
    },
)
