from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='habibarezq',
    maintainer_email='habibarezq30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_node=mypkg.my_node:main",
            "oop_node=mypkg.oop_node:main",
            "publisher_node=mypkg.publisher_node:main",
            "subscriber_node=mypkg.subscriber_node:main",
            "server_node=mypkg.server_node:main",
            "client_node=mypkg.client_node:main",
            "test_node=mypkg.test_node:main",
            "param_node=mypkg.param_node:main"
        ],
    },
)
