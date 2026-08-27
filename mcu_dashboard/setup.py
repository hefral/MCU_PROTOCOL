from glob import glob
from setuptools import find_packages, setup


package_name = 'mcu_dashboard'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyQt5>=5.15', 'pyqtgraph>=0.13'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='hang',
    maintainer_email='hang@todo.invalid',
    description='MCU telemetry dashboard and eight-channel motor command test panel.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mcu_dashboard = mcu_dashboard.main:main',
        ],
    },
)
