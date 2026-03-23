import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ares_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the YOLO model file so the script can find it when built
        (os.path.join('share', package_name), glob('*.pt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shivansh',
    maintainer_email='shivansh@todo.todo',
    description='YOLO AI Vision processing for ARES-X',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This links the terminal command to your python script
            'visual_cortex = ares_brain.visual_cortex:main'
        ],
    },
)