from glob import glob
from setuptools import setup

package_name = "harvest_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harvesting Team",
    maintainer_email="maintainer@example.com",
    description="Launch composition for the robotic harvesting system.",
    license="Apache-2.0",
)
