from setuptools import setup

package_name = 'tidybot_network_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Qiu',
    maintainer_email='aqiu34@stanford.edu',
    description='Network bridge utilities for TidyBot2 remote control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_compression_node = tidybot_network_bridge.image_compression_node:main',
        ],
    },
)
