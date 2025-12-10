from setuptools import setup

package_name = 'empty_slot_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 明确声明 Python package 目录
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lwl',
    maintainer_email='lwl@example.com',
    description='Empty slot detector for shelf monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = empty_slot_detector.detector_node:main',
        ],
    },
)

