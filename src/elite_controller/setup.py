from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    name="elite_controller",
    version="1.0.0",
    # author="elite",
    # author_email="elite@elite.com",
    # description="ROS Controller For Elite!",
    packages=['elite_controller'],
    package_dir={'': 'src'}
)

setup(**d)
