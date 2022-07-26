from setuptools import setup

package_name = "bno055_node"
submoudles = "bno055_node/submodules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submoudles],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="cbsim@zigbang.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["bno055_node = bno055_node.bno055_node:main"],
    },
)
