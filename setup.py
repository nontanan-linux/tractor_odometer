from setuptools import find_packages, setup

package_name = 'tractor_odometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tractor_odometer.launch.py', 'launch/tractor_odometer.launch.xml', 'launch/tractor_position.launch.py']),
        ('share/' + package_name + '/config', ['config/tractor_config.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/tractor.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nontanan',
    maintainer_email='nontanan@gensurv.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometer_node = tractor_odometer.odometer_node:main',
            'nmb_odometer_node = tractor_odometer.nmb_odometer_node:main',
        ],
    },
)
