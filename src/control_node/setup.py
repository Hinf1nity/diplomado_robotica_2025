from setuptools import find_packages, setup

package_name = 'control_node'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_base = control_node.move_base:main',
            'odom_callback = control_node.odom_callback:main',
            'perception = control_node.perception:main',
            'aruco_pose = control_node.aruco_pose:main',
            'sub_tf = control_node.sub_tf:main',
            'teleop_robot = control_node.teleop_robot:main',
        ],
    },
)
