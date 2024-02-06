import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imcpy_trees'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'launch*']),
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name: ['tree_actions/*','behaviours/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olaya',
    maintainer_email='olaya_93@hotmail.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'followsingleref_server= imcpy_trees.tree_actions.follow_single_reference:main',
            'followSingleRef= imcpy_trees.follow_single_reference_tree:main',
            'square= imcpy_trees.square_tree:main'
        ],
    },
)
