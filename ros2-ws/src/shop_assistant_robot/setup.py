from setuptools import setup
import os
from glob import glob

package_name = 'shop_assistant_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py') if os.path.exists('launch') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucia',
    maintainer_email='lucia31257@gmail.com',
    description='Shop Assistant Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_text_server = shop_assistant_robot.speech_to_text_server:main',
        ],
    },
)