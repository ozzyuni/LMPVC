from setuptools import find_packages, setup

package_name = 'lmpvc_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/core_config.json']),
        ('share/' + package_name, ['config/preamble.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parikkao',
    maintainer_email='ossi.parikka@tuni.fi',
    description='LMPVC core voice control interface node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'codegen_test = lmpvc_core.codegen_cli:main',
            'listener_test = lmpvc_core.listener_cli:main',
            'controller_test = lmpvc_core.controller_cli:main',
            'detector_test = lmpvc_core.detector_cli:main',
            'talker_test = lmpvc_core.talker_cli:main',
            'voice_control = lmpvc_core.main:main',
        ],
    },
)
