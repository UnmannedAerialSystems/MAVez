from setuptools import setup, find_packages

setup(
    name='MAVez',
    version='1.0',
    packages=find_packages(),
    description='A Python library for controlling MAVLink drones',
    author='Ted Tasman',
    license='MIT',
    install_requires=[
        'pymavlink',
        'pyserial'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6'
)