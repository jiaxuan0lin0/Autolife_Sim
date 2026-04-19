from setuptools import find_packages, setup

package_name = 'autolife_control'

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
    maintainer='JiaxuanLin',
    maintainer_email='jiaxuan0lin0@gmail.com',
    description='ROS2 Controller for Autolife in Isaac Sim',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_controller = autolife_control.base_contorller:main',
            'torso_controller = autolife.control.torso_controller:main', 
            'head_controller = autolife.control.head_controller:main',
            'gripper_controller = autolife.control.gripper_controller',
        ],
    },
)
