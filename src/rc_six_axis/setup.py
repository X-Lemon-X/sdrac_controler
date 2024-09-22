from setuptools import find_packages, setup

package_name = 'rc_six_axis'
# submodules = 'rc_six_axis/Pilot_6_axis/ros'
# submodules = ['lemonx_receive', 'receiver_ros']

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
    maintainer='lemonx',
    maintainer_email='dudzinski.patryk.02@gmail.com',
    description='Remote control node for six axis rc from https://github.com/X-Lemon-X/Pilot_6_axis',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'rc_node = rc_six_axis.rc_six_axis:main',
        ],
    },
)
