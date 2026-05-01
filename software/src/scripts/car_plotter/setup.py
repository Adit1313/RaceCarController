from setuptools import find_packages, setup

setup(
    name="car_plotter",
    version="1.0",
    description="Plotting utils for crs",
    author="Joshua Näf",
    author_email="naefjo@ethz.ch",
    packages=find_packages(),
    entry_points={
        "console_scripts": [
            "plot_car=scripts.plot_car:main",
        ],
    },
)
