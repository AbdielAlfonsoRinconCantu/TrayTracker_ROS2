from setuptools import find_packages, setup

package_name = 'ros2_image'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='auto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'track_tray_image_show = ros2_opencv.subscriberImage:main',
        'count_penny_image_show = ros2_opencv.subscriberImage_2:main',
        'image_show = ros2_opencv.subscriberImage_3:main',
        'image_node = ros2_opencv.webcam:main',
        ],
    },
)
