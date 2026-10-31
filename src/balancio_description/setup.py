from glob import glob

from setuptools import setup

package_name = "balancio_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        (
            "share/" + package_name + "/meshes",
            glob("meshes/*.obj") + glob("meshes/*.mtl"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": ["motor_model = balancio_description.motor_model:main"],
    },
    maintainer="Alvaro J. Gaona",
    maintainer_email="alvgaona@gmail.com",
    description="Balancio robot description: URDF, meshes, parameters and motor model",
    license="MIT",
)
