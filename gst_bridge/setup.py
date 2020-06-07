from setuptools import setup

package_name = 'gst_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brettrd',
    maintainer_email='brettrd@brettrd.com',
    description='Bridge GStreamer to ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_node = gst_bridge.pipeline_node:main'
        ],
    },
)
