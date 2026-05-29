from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含所有 .msg 文件
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        # 包含所有 .srv 文件
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaikai',
    maintainer_email='kaikai@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_keyboard = motor_control.motion_keyboard:main',
            'angle_control = motor_control.angle_control:main',
        ],
    },
)
