from setuptools import find_packages, setup

package_name = 'smart_diffbot_clients'

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
    maintainer='Kees van Teeffelen',
    maintainer_email='k.j.vanteeffelen@saxion.nl',
    description='Clients package of smart_diffbot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_client = smart_diffbot_clients.docking_client:main',
            'line_follow_client = smart_diffbot_clients.line_follow_client:main'
        ],
    },
)
