from setuptools import find_packages, setup

package_name = "quad_servo_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"])
    + ["scservo_sdk"],
    package_dir={
        "scservo_sdk": "third_party/scservo_sdk",
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/README.md"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="quad",
    maintainer_email="todo@email.com",
    description="CHAMP to Feetech servo driver (scservo_sdk) + read-only diag",
    license="BSD",
    entry_points={
        "console_scripts": [
            "champ_servo_driver = quad_servo_driver.champ_servo_driver:main",
            "servo_diag = quad_servo_driver.servo_diag:main",
            "hw_joint_test = quad_servo_driver.hw_joint_test:main",
            "servo_calibrate = quad_servo_driver.servo_calibrate:main",
            "servo_home = quad_servo_driver.servo_home:main",
            "servo_torque = quad_servo_driver.servo_torque:main",
        ],
    },
)
