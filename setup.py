import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'bno055'

setup(
    name=package_name,
    version='0.4.1',
    # find sub-packages automatically in order to allow sub-modules, etc. to be imported:
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    py_modules=[
        package_name + '.bno055',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('bno055/params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='flynneva',
    author_email='evanflynn.msu@gmail.com',
    maintainer='flynneva',
    maintainer_email='evanflynn.msu@gmail.com',
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Bosch BNO055 IMU driver for ROS2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055 = bno055.bno055:main',
        ],
    },
)
