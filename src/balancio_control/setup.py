from glob import glob

from setuptools import setup

package_name = "balancio_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alvaro J. Gaona",
    maintainer_email="alvgaona@gmail.com",
    description="Nonlinear MPC for the Balancio two-wheeled self-balancing robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_node = balancio_control.mpc_node:main",
            "path_follower = balancio_control.path_follower:main",
        ],
    },
)
