from setuptools import find_packages, setup

package_name = 'priority_action_system'

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
    maintainer='rohit',
    maintainer_email='rd8614196@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'high_priority_server = priority_action_system.high_priority_server:main',
            'low_priority_server = priority_action_system.low_priority_server:main',
            'priority_client = priority_action_system.priority_client:main',
        ],
    },
)
