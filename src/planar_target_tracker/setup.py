from setuptools import find_packages, setup
import glob
import os
package_name = 'planar_target_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Delgado Ramos (A01645788)',
    maintainer_email='A01645788@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_segmenter_node=planar_target_tracker.depth_segmenter_node:main',
            'pose_estimator_node=planar_target_tracker.pose_estimator_node:main'
        ],
    },
)
