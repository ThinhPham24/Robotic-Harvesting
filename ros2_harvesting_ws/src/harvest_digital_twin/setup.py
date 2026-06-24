from setuptools import find_packages, setup

package_name = "harvest_digital_twin"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/twin_monitor.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Harvesting Team",
    maintainer_email="maintainer@example.com",
    description="Operational digital twin synchronization and residual monitoring.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "twin_monitor = harvest_digital_twin.twin_monitor:main",
        ],
    },
)
