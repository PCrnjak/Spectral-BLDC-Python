from setuptools import setup, find_packages
import codecs
import os

here = os.path.abspath(os.path.dirname(__file__))

with codecs.open(os.path.join(here, "README.md"), encoding="utf-8") as fh:
    long_description = "\n" + fh.read()

VERSION = '1.17'
DESCRIPTION = 'Python module for Spectral BLDC motor controllers'

# Setting up
setup(
    name="Spectral_BLDC",
    version=VERSION,
    author="Source robotics (Petar Crnjak)",
    author_email="<info@source-robotics.com>",
    license = "MIT",
    project_urls = {
        "Documentation": "https://source-robotics.github.io/PAROL-docs/",
        "Source": "https://source-robotics.github.io/PAROL-docs/",
    },
    description=DESCRIPTION,
    long_description_content_type="text/markdown",
    long_description=long_description,
    packages=find_packages(),
    install_requires=['python-can','pyserial'],
    keywords=['python', 'BLDC', 'CANBUS', 'Robot', 'Source robotics', 'robotics'],
    classifiers=[
        "Development Status :: 1 - Planning",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: Unix",
        "Operating System :: Microsoft :: Windows",
    ]
)