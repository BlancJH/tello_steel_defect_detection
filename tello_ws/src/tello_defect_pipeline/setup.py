from glob import glob
from setuptools import find_packages, setup

package_name = "tello_defect_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tello Defect Pipeline Maintainer",
    maintainer_email="user@example.com",
    description="ROS 2 Jazzy pipeline for Tello EDU video streaming and steel defect detection.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
