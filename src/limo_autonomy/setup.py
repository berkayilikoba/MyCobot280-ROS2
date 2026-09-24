from glob import glob

from setuptools import setup

package_name = 'limo_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunes',
    description='LIMO otonom gorev katmani',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'patrol = limo_autonomy.patrol_node:main',
    ]},
)
