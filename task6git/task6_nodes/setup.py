from setuptools import find_packages, setup

package_name = 'task6_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy','std_msgs','task6_interface'],
    zip_safe=True,
    maintainer='muhir',
    maintainer_email='muhir.mitmpl2024@mit.manipal.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'node_1=task6_nodes.node_1:main',
        'node_2=task6_nodes.node_2:main',
        'node_3=task6_nodes.node_3:main',
        ],
    },
)
