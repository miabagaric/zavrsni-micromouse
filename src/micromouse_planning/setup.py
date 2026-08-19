from setuptools import find_packages, setup

package_name = "micromouse_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mia",
    maintainer_email="mb55566@fer.hr",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "planner_node = micromouse_planning.planner_node:main",
            "test_astar_truth = micromouse_planning.test.test_astar_truth:main",
        ],
    },
)
