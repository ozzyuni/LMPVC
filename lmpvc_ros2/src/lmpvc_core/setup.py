import json
from setuptools import find_packages, setup

package_name = 'lmpvc_core'

def policy_files():
    """Loads the index file and corresponding sources from disk"""
    data_files = []

    # Read index
    dir = package_name + '/policies'
    index_path = dir + '/index.json'
    index = {}
    with open(index_path, 'r') as index_file:
        index = json.load(index_file)

    # Install index file
    data_files.append(('share/' + package_name + '/policies', [index_path]))

    src_paths = []

    for policy, src_path in index.items():
        src_paths.append(dir + '/' + src_path)

    data_files.append(('share/' + package_name + '/policies/src', src_paths))

    return data_files
            

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
    ] + policy_files(),
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
