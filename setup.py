from setuptools import setup

package_name = 'turn_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'turn_robot.script',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ItoMasaki',
    maintainer_email='is0449sh@ed.ritsumei.ac.jp',
    description='TODO: Package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turn_robot = turn_robot.script:main',
        ],
    },
)
