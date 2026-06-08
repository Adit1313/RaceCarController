from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["rockpara_telemetry"],
    scripts=["scripts/rockpara_telemetry_node.py"],
    package_dir={"": "src"},
)

setup(**d)
