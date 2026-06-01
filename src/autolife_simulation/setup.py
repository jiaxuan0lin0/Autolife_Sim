from setuptools import find_packages, setup

package_name = 'autolife_simulation'

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
    description='Isaac Sim and OmniGibson integration for loading BEHAVIOR-1K scenes with the Autolife robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_base_drive_response = autolife_simulation.base_drive_response_test:main',
            'test_joint_drive_response = autolife_simulation.joint_drive_response_test:main',
        ],
    },
)
