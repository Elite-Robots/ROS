from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages



d = generate_distutils_setup(
    name="elite_driver",
    version="1.0.0",
    author="elite",
    author_email="elite@elibot.cn",
    description="ROS Driver For Elite!",
    packages=['elite_driver'],
    package_dir={'': 'src'}
)

setup(**d)
