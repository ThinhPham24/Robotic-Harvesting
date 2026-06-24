from setuptools import find_packages, setup

package_name = "harvest_mpc_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mpc.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Harvesting Team",
    maintainer_email="maintainer@example.com",
    description="Shadow-mode constrained MPC reference generator.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["mpc_reference = harvest_mpc_controller.mpc_reference:main"]},
)
