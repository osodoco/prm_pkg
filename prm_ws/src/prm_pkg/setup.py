from setuptools import setup
import os
from glob import glob

package_name = 'prm_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # This line is crucial for ROS 2 to find your package's resources.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        # Install launch files (like your main_navigation.launch.py)
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
        # Install CoppeliaSim scene files
        ('share/' + package_name + '/coppelia_scenes', glob('coppelia_scenes/*.ttt')),
        
        # Install your package's XML definition
        ('share/' + package_name, ['package.xml']),
        
        # Install map files (.yaml and .pgm)
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        
        # Install RViz configuration files (if you create a 'rviz' folder with a .rviz file)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
        # Install other configuration files (e.g., AMCL parameters in a 'config' folder)
        # Uncomment the line below if you plan to add a 'config' directory with .yaml files
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osdoco',
    maintainer_email='osdoco@alumno.upv.es',
    description='PRM Navigation integrated with CoppeliaSim and RViz2',
    license='MIT',
    entry_points={
        'console_scripts': [
            # These lines define your executable nodes, allowing you to run them with 'ros2 run'
            'generate_prm = prm_pkg.generate_prm:main',
            'amcl_odom = prm_pkg.amcl_odom:main',
            'prm_navigation = prm_pkg.prm_navigation:main',
        ],
    },
)
