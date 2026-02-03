from setuptools import setup

package_name = 'tidybot_ik'

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
    maintainer='Alex Qiu, Matt Strong and Joonwon Kang',
    maintainer_email='aqiu34@stanford.edu, mastro1@stanford.edu, jwkang@stanford.edu',
    description='Motion planning and IK solver for TidyBot2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motion_planner_node = tidybot_ik.motion_planner_node:main',
            'motion_planner_real_node = tidybot_ik.motion_planner_real_node:main',
        ],
    },
)
