from setuptools import setup

setup(
    name='team_controller', version='0.2.0',
    packages=['team_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/team_controller']),
        ('share/team_controller', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Competition team', maintainer_email='team@example.com',
    description='Reactive LiDAR controller for F1TENTH V1.1', license='MIT',
    entry_points={'console_scripts': [
        'controller = team_controller.controller_node:main',
    ]},
)
