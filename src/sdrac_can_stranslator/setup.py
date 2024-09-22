from setuptools import find_packages, setup

package_name = 'sdrac_can_stranslator'

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
    description='Can node for read write to can for sdrac',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'can_code = sdrac_can_stranslator.can_node:main',
        ],
    },
)
