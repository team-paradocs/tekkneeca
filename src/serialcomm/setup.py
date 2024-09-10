from setuptools import find_packages, setup

package_name = 'serialcomm'

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
    maintainer='shivangi',
    maintainer_email='shivangi@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testsub_exec=serialcomm.testsub:main',
            'serialwriter_exec=serialcomm.write_to_serial:main'
        ],
    },
)
