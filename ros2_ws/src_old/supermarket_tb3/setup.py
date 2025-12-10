from setuptools import setup

package_name = 'supermarket_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/tb3_supermarket.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lwl',
    maintainer_email='lwl@example.com',
    description='Launch TB3 in supermarket world',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)

