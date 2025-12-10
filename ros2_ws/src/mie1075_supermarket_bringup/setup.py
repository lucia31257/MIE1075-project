from setuptools import setup
from glob import glob
import os

package_name = 'mie1075_supermarket_bringup'

def get_all_files(directory):
    """Return only files (not directories) recursively."""
    files = []
    for root, dirs, filenames in os.walk(directory):
        for f in filenames:
            files.append(os.path.join(root, f))
    return files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # ⬇ Install models (recursively, only files)
        ('share/' + package_name + '/models', get_all_files('models')),

        # ⬇ Install world + maps (recursively, only files)
        ('share/' + package_name + '/MIE1075-project', get_all_files('MIE1075-project')),

        # ⬇ Install nav2 config
        ('share/' + package_name + '/nav2_config', get_all_files('nav2_config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaoxiangyin',
    maintainer_email='none@example.com',
    description='Supermarket bringup package',
    license='Apache License 2.0',
    entry_points={'console_scripts': []},
)
