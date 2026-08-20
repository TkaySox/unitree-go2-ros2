from setuptools import setup

package_name = "quad_servo_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/README.md"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="quad",
    maintainer_email="todo@email.com",
    description="CHAMP to Feetech servo driver with calibration offsets",
    license="BSD",
    entry_points={
        "console_scripts": [
            "champ_servo_driver = quad_servo_driver.champ_servo_driver:main",
        ],
    },
)
