from setuptools import find_packages, setup

package_name = 'lmpvc_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/test_objects.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parikkao',
    maintainer_email='ossi.parikka@tuni.fi',
    description='Object detectiona accessible with a service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = lmpvc_detector.detector_srv:main',
            'pose_recorder = lmpvc_detector.pose_recorder:main'
        ],
    },
)
