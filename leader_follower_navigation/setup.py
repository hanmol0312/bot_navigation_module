
from setuptools import setup
from glob import glob
import os

package_name = 'leader_follower_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anmol',
    maintainer_email='ec21b1086@iiitdm.ac.in',
    description='A package for autonomous robot operations',
    license='Apache License 2.0',  # Update with your actual license
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'follower_node = leader_follower_navigation.follower:main',
        ],
    },
)

