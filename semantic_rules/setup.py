from setuptools import find_packages, setup
import os, glob

package_name = 'semantic_rules'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mihkel Marten Rüütli',
    maintainer_email='lightprohvet@gmail.com',
    description='Provides an interface for converting segmentation results to costmaps',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'detection_costmap_rule = semantic_rules.detection_costmap_rule:main',
                'detection_converter_node = detection_converter.detection_converter_node:main',
                'rule_assigner_node = detection_converter.rule_assigner_node:main',
        ],
    },
)
