from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pc_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'configs'), glob(os.path.join('configs', '*'))),
        (os.path.join('share', package_name, 'model_state_dicts'), glob(os.path.join('model_state_dicts', '*'))),
        (os.path.join('share', package_name, 'radar_cfgs'), glob(os.path.join('radar_cfgs', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='david.hunt@duke.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc_combiner= pc_processing.pc_combiner:main',
            'vel_filtering= pc_processing.vel_filtering:main',
            'pc_integrator= pc_processing.pc_integrator:main',
            'radsar_processor= pc_processing.radsar_processor:main'
        ],
    },
)
