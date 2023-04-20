# Install the supervisor library

from setuptools import setup, find_packages

setup(
    name='supervisor',
    version='0.1.0',
    packages=find_packages(where="src"),
    package_dir={'': 'src'},
    install_requires=[],
    author="ASU ROAR",
    author_email='asuroar.eg@gmail.com',
    description="Supervisor library",
    license='GNU General Public License v3.0',
    url='https://github.com/asu-roar/roar'
)