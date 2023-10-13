from setuptools import find_packages, setup

package_name = 'team19_chase_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joseph Sommer',
    maintainer_email='jsommer33@gatech.edu',
    description='Detects and chases an object',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'detect_object = team19_chase_object.detect_object:main',
		'get_object_range = team19_chase_object.get_object_range:main',
		'chase_object = team19_chase_object.chase_object:main',
        ],
    },
)
