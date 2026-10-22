from setuptools import setup

package_name = 'lunabot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LunaBot Team',
    maintainer_email='ops@lunabot.local',
    description='Manual keyboard teleoperation for LunaBot development.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wasd_teleop = lunabot_teleop.wasd_teleop:main',
        ],
    },
)
