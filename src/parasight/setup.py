from setuptools import find_packages, setup

package_name = 'parasight'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WarrG3X',
    maintainer_email='warrier.abhishek@gmail.com',
    description='Parasight ROS2 Package that integrates registration and tracking for assisted knee surgery',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = parasight.hello_world:main'
        ],
    },
)
