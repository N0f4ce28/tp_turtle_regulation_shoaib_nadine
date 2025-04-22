from setuptools import setup

package_name = 'turtle_regulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadine',
    maintainer_email='nadinevilmont3004@gmail.com',
    description='Waypoint regulation for TurtleSim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_way_point = turtle_regulation.set_way_point:main',
        ],
    },
)