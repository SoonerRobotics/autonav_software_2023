#!/usr/bin/env python3

import os


# path to this script's directory where format files are located
path_to_script = os.path.dirname(os.path.abspath(__file__))

# read format files for text data
with open(path_to_script + "/CMakeListsFormat.txt", 'r') as cmake_format:
    cmake_text = cmake_format.read()

with open(path_to_script +"/packageFormat.txt", "r") as package_format:
    package_text = package_format.read()

# ensure that duplicate files are not created
try:
    os.remove("CMakeLists.txt")
except Exception as e:
    print("CMakelists.txt didn't exist yet")

try:
    os.remove("package.xml")
except Exception as e:
    print("packge.xml didn't exist yet")

# create CmakeLists.txt and package.xml
print("Creating CMakeLists.txt for a python and cpp package")
with open("CMakeLists.txt", "w") as cmake:
    cmake.write(cmake_text)

print("Creating package.xml for a python and cpp package")
with open("package.xml", "w") as package:
    package.write(package_text)