from setuptools import find_packages, setup

package_name = 'lmpvc_listener'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/listener_config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parikkao',
    maintainer_email='ossi.parikka@tuni.fi',
    description='Records and transcribes audio input',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = lmpvc_listener.listener_srv:main',
            'test = lmpvc_listener.listener:main',
            'transcriber = lmpvc_listener.transcriber_srv:main',
            'transcriber_tcp_srv = lmpvc_listener.transcriber_tcp_srv:main',
            'transcriber_tcp_cli = lmpvc_listener.transcriber_tcp_cli:main'
        ],
    },
)
