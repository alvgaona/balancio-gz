from glob import glob

from setuptools import setup

package_name = "balancio_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alvaro J. Gaona",
    maintainer_email="alvgaona@gmail.com",
    description="Launch files, Gazebo world and configs that bring up the Balancio simulation",
    license="MIT",
)
