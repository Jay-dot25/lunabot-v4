from setuptools import setup

package_name = 'lunabot_tools'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LunaBot maintainers',
    maintainer_email='maintainers@example.invalid',
    description='LunaBot Phase 1 command and sensor utilities',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'command_gate = lunabot_tools.command_gate:main',
        'obstacle_monitor = lunabot_tools.obstacle_monitor:main',
        'keyboard_teleop = lunabot_tools.keyboard_teleop:main',
        'runtime_probe = lunabot_tools.runtime_probe:main',
    ]},
)
