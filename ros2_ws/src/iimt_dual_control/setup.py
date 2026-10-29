from glob import glob

from setuptools import setup

setup(
    name="iimt_dual_control",
    version="0.2.0",
    packages=["iimt_dual_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/iimt_dual_control"]),
        ("share/iimt_dual_control", ["package.xml", "README.md"]),
        ("share/iimt_dual_control/launch", glob("launch/*.py")),
        ("share/iimt_dual_control/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": ["dual_coordinator = iimt_dual_control.coordinator:main"]
    },
    maintainer="IIMT workspace maintainers",
    maintainer_email="maintainer@example.com",
    description="Coordinated twelve-axis control for two IIMT CI-05 arms",
    license="Apache-2.0",
)
