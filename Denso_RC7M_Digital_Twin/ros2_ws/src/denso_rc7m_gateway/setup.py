from glob import glob
from setuptools import find_packages, setup

package_name = "denso_rc7m_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Team",
    maintainer_email="maintainer@example.com",
    description="Serial and telemetry gateways for RC7M.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "telemetry_bridge = denso_rc7m_gateway.telemetry_bridge:main",
            "serial_gateway = denso_rc7m_gateway.serial_gateway:main",
            "serial_read_test = denso_rc7m_gateway.serial_read_test:main",
        ]
    },
)
