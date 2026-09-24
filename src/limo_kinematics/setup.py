from setuptools import setup

package_name = 'limo_kinematics'

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
    maintainer='gunes',
    description='LIMO kinematik hesaplari (URDF -> Nav2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'limo_kinematics = limo_kinematics.cli:main',
        'ackermann_cmd_adapter = limo_kinematics.ackermann_cmd_adapter:main',
    ]},
)
