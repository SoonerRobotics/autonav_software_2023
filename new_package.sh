read -p "Package Name: " package_name
read -p "Author: " author
read -p "Email: " email

# Create the package directory
mkdir autonav_ws/src/autonav_$package_name

# Create the package src directory
mkdir autonav_ws/src/autonav_$package_name/src

# Create the package launch directory
mkdir autonav_ws/src/autonav_$package_name/launch

# Create the package resource directory
mkdir autonav_ws/src/autonav_$package_name/resource

# Create the package.xml file
printf "<?xml version=\"1.0\"?>
<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>
<package format=\"3\">
    <name>autonav_$package_name</name>
    <version>1.0.0</version>
    <description>An empty AutoNav ROS package</description>
    <maintainer email=\"$email\">$author</maintainer>
    <license>MIT License</license>

    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>

    <export>
    <build_type>ament_python</build_type>
    </export>
</package>" >> autonav_ws/src/autonav_$package_name/package.xml

# Create the setup.cfg
printf "[develop]
script_dir=\$base/lib/autonav_$package_name
[install]
install_scripts=\$base/lib/autonav_$package_name" >> autonav_ws/src/autonav_$package_name/setup.cfg

printf "from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonav_$package_name'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.xml')))
    ],
    install_requires=['setuptools'],
    maintainer='$name',
    maintainer_email='$email',
    description='An empty AutoNav ROS package',
    license='MIT License',
    entry_points={}
)
" >> autonav_ws/src/autonav_$package_name/setup.py

touch autonav_ws/src/autonav_$package_name/resource/autonav_$package_name
touch autonav_ws/src/autonav_$package_name/src/__init__.py
touch autonav_ws/src/autonav_$package_name/src/main.py