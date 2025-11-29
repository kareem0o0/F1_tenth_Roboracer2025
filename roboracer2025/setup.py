from setuptools import find_packages, setup

package_name = 'roboracer2025'

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
    maintainer='root',
    maintainer_email='kareem.salahazazy10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = roboracer2025.wall_follower:main',
            'pps_cdc_QRound = roboracer2025.pps_cdc_QRound:main',
        ],
    },
)
