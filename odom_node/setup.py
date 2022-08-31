from setuptools import setup
import os
from glob import glob

package_name = "odom_node"
submoudles = "bno055_node/submodules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submoudles],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="cbsim@zigbang.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["odom_node = odom_node.odom_node:main"],
        "console_scripts": ["odom_node = odom_node.ls7366r:main"],
    },
)
