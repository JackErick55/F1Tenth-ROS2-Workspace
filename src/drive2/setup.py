from setuptools import find_packages, setup

package_name = 'drive2'

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
    maintainer='darc-f1-01',
    maintainer_email='darc-f1-01@todo.todo',
    description='drive',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'drive2 = drive2.drive2:main',
        ],
    },
)
