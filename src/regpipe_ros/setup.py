from setuptools import find_packages, setup

package_name = 'regpipe_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/source', ['source/femur.ply']),
        ('share/' + package_name + '/source', ['source/femur_shell.ply']),
        # ('share/' + package_name + '/source', ['source/femur_shell2.ply']),
        ('share/' + package_name + '/source', ['source/plan_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warra',
    maintainer_email='warrier.abhishek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = regpipe_ros.hello_world:main',
            'pcd_publisher = regpipe_ros.pcd_publisher:main',
            'pcd_subscriber = regpipe_ros.pcd_subscriber:main',
            'pcd_regpipe = regpipe_ros.pcd_regpipe:main',
            'pcd_regpipe_single = regpipe_ros.pcd_regpipe_single:main',
            'pcd_sampipe = regpipe_ros.pcd_sampipe:main',
            'rgb_tracker = regpipe_ros.rgb_tracker:main',
        ],
    },
)
