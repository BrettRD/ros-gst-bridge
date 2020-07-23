from setuptools import setup
from glob import glob

package_name = 'gst_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.config.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brettrd',
    maintainer_email='brettrd@brettrd.com',
    description='Run GStreamer pipelines in ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_node = gst_pipeline.pipeline_node:main'
        ],
    },
)
