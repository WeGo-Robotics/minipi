# web_gui_pkg/setup.py

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['web_gui_pkg'],
    package_dir={'': 'src'}
)

setup(**d)