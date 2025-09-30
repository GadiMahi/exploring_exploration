from setuptools import find_packages, setup

package_name = 'gps_frontier_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_frontier_explorer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahi',
    maintainer_email='mahigadi27@gmail.com',
    description='GPS-guided frontier exploration package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_frontier_node = gps_frontier_explorer.gps_frontier_node:main',
        ],
    },
)
