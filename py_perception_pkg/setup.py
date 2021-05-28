from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

p = generate_distutils_setup(
    packages=['py_perception_pkg'],
    package_dir={'': 'src'}
)

setup(**p)
