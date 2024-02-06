from setuptools import setup

package_name = 'unavsim_ros_pkgs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={package_name: 'scripts'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Olaya Alvarez Tunon',
    maintainer_email='olaya@ece.au.dk',
    description='Description of the unavsim_ros_pkgs package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mimir_recorder = unavsim_ros_pkgs.mimir_recorder:main'
        ],
    },
)
