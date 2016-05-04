from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['periodic_poisson_processes', 'region_observation'],
    # scripts=['scripts/trajectory_region_knowledge.py'],
    package_dir={'': 'src'}
)

setup(**setup_args)
