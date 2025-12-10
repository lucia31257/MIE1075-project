from setuptools import setup

package_name = 'simple_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lwl',
    maintainer_email='',
    description='Simple XY navigation with cmd_vel',
    entry_points={
        'console_scripts': [
            'goto_xy = simple_nav.goto_xy:main',
        ],
    },
)

