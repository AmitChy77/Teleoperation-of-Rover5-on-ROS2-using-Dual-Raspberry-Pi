from setuptools import find_packages, setup

package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch', [
            'launch/robot_with_camera.launch.py'
            ]),
        ('share/' + package_name +'/config', [
            'config/camera_view.rviz'
            ]),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amit',
    maintainer_email='xavierian.amit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['livestream = project.livestream:main',
        'motor_control = project.motor_control:main',
        'servo_control= project.servo_control:main',                
        ],
    },
)
