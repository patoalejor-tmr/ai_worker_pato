from setuptools import find_packages, setup
import glob
package_name = 'ffw_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),  # glob 처리
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),        # config도 있으면 추가
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.sdf')),          # worlds도 있으면 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ywh@robotis.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_controller_left = ffw_bringup.hand_controller_left:main',
            'hand_controller_right = ffw_bringup.hand_controller_right:main',
            'hand_calibratior_right = ffw_bringup.hand_calibrator_right:main',
            'hand_calibratior_left = ffw_bringup.hand_calibrator_left:main',
            'init_position_for_follower_teleop = ffw_bringup.init_position_for_follower_teleop:main',
        ],
    },
)
