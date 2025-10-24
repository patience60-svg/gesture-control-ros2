from setuptools import find_packages, setup

package_name = 'finger_follower'

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
    maintainer='teli',
    maintainer_email='patiencegondwe60@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "cam = finger_follower.cam:main",
            "finger_detector = finger_follower.finger_detector:main",
            "hand_gesture_controller = finger_follower.hand_gesture_controller:main"
        ],
    },
)
