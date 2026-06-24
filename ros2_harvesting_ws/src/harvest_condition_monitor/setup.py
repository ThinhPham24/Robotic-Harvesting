from setuptools import find_packages, setup

package_name = "harvest_condition_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/condition_monitor.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harvesting Team",
    maintainer_email="maintainer@example.com",
    description="Condition monitoring for harvesting assets.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["condition_monitor = harvest_condition_monitor.condition_monitor:main"]},
)
