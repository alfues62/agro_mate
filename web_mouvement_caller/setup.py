from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'web_mouvement_caller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')) #incluir
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tallalsudo',
    maintainer_email='okabi.rintaro.2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_movement_server = web_mouvement_caller.web_movement_server:main' #incluir
        ],
    },
)
