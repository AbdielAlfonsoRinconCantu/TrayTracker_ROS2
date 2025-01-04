from setuptools import setup

package_name = 'video_stream_ros2'

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
    maintainer='relampago',
    maintainer_email='relampago@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_tray_image_web = video_stream_ros2.app_2:main',
            'count_penny_image_web = video_stream_ros2.app_3:main',
            'image_web = video_stream_ros2.app:main',
        ],
    },
)
