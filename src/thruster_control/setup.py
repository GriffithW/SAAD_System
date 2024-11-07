from setuptools import find_packages, setup

package_name = 'thruster_control'

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
    maintainer='griffithw',
    maintainer_email='griffithwiele@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "thruster_control_keyboard = thruster_control.thruster_control_keyboard:main"
            #"thruster_control_left = thruster_control.thruster_control_left:main"
            #"thruster_control_right = thruster_control.thruster_control_right:main"
        ],
    },
)
