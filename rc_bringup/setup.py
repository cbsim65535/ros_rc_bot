from setuptools import setup
import os
from glob import glob

package_name = "rc_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
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
        "console_scripts": ["pca9685_node = pca9685_node.pca9685_node:main"],
    },
)
