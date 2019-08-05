from setuptools import setup
 
package_name = 'bno055'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        package_name,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='flynneva',
    author_email="evanflynn.msu@gmail.com",
    maintainer='flynneva',
    maintainer_email="evanflynn.msu@gmail.com",
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
            'bno055 = bno055:main',
        ],
    },
)
