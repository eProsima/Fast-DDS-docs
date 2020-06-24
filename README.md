# eProsima Fast DDS Documentation

[![Releases](https://img.shields.io/github/release/eProsima/Fast-DDS.svg)](https://github.com/eProsima/Fast-DDS/releases)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Linux Build Status](http://jenkins.eprosima.com:8080/job/nightly_fastdds-docs_master/badge/icon?subject=CI%20testing%20)](http://jenkins.eprosima.com:8080/job/nightly_fastdds-docs_master/)

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

*eprosima Fast DDS* (formerly Fast RTPS) is a C++ implementation of the DDS (Data Distribution Service) standard of the OMG (Object Management Group). eProsima Fast DDS implements the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP,
as defined and maintained by the Object Management Group (OMG) consortium. RTPS is also the wire interoperability protocol defined for the Data Distribution
Service (DDS) standard. *eProsima Fast DDS* exposes an API to access the RTPS protocol directly, giving the user full access to the protocol internals.

Some of the main features of this library are:

* Configurable best-effort and reliable publish-subscribe communication policies for real-time
applications.
* Plug and play connectivity so that any new applications are automatically discovered by any other
members of the network.
* Modularity and scalability to allow continuous growth with complex and simple devices in the
network.
* Configurable network behavior and interchangeable transport layer: Choose the best protocol and
system input/output channel combination for each deployment.
* Two API Layers: a high-level Publisher-Subscriber one focused on usability (DDS) and a lower-level Writer-Reader one that provides finer access to the inner workings of the RTPS protocol.

*eProsima Fast DDS* has been adopted by multiple organizations in many sectors including these important cases:

* Robotics: ROS (Robotic Operating System) as their default middleware for ROS2.
* EU R&D: FIWARE Incubated GE.

You can find all the library's source code on our [GitHub repository](https://github.com/eProsima/Fast-DDS).

The documentation is built using [Sphinx](https://www.sphinx-doc.org), and it is hosted at [Read the Docs](https://readthedocs.org).
The online documentation generated with this project can be found in [Fast DDS documentation](https://fast-dds.docs.eprosima.com).

1. [Project structure](#project-structure)
1. [Installation Guide](#installation-guide)
1. [Getting Started](#getting-started)
1. [Generating documentation in other formats](#generating-documentation-in-other-formats)
1. [Running documentation tests](#running-documentation-tests)
1. [Simulating Read the Docs](#simulating-read-the-docs)
1. [Contributing](#contributing)

## Project structure

The project is structured as follows:

1. The root directory contains global scope files, such as this one.
1. The [docs directory](#docs-directory) contains all documentation source code.
1. Code snippets and testing code is located in the [code directory](#code-directory).

### `doc` directory

The [docs](docs) directory contains:

* [_static](docs/_static): For HTML theme related files.
* [01-figures](docs/01-figures): For all the documentation figures. SVG files are the preferred format since the XML can be modified or otherwise checked for differences.
* [02-formalia](docs/02-formalia): Must-have pages such as Introduction.
* [fastdds](docs/fastdds): Fast DDS documentation.
* [fastddsgen](docs/fastddsgen): Fast DDS-Gen documentation.
* [installation](docs/installation): Installation manual.
* [notes](docs/notes): Release notes.

All new documentation must fall into one of these directories, with the exception of those contributions which are not related to any of the given descriptions.
Keep in mind that this is an Sphinx based project, and as such, the all the documentation is written in `reStructuredText`.

All unrecognized words must be added to the [spelling_wordlist.txt](docs/spelling_wordlist.txt) dictionary in alphabetical order, with exception of the ones coming from the API reference documentation, which must be added to [docs/fastdds/api_reference/spelling_wordlist.txt](docs/fastdds/api_reference/spelling_wordlist.txt).

### `code` directory

The [code](code) directory contains all the files related to code snippets and CI testing.
Files of particular importance are:

* [CodeTester.cpp](code/CodeTester.cpp): Contains all Fast DDS pub-sub and RTPS layer snippets.
It is a buildable file, so it can be compiled and linked against the Fast DDS to verify that all samples of code are up to date.
Furthermore, it is used to create an executable that is then used to generate Ctest tests.
* [DDSCodeTester.cpp](code/DDSCodeTester.cpp): Contains Fast DDS DDS layer snippets.
* [XMLTester.xml](code/XMLTester.xml): Contains XML snippets.
* [doxygen-config.in](code/doxygen-config.in): Doxyfile to configure Doxygen for creating Fast DDS API reference.

## Installation Guide

1. In order to build and test the documentation, some dependencies must be installed beforehand:

    ```bash
    sudo apt update
    sudo apt install -y \
        git \
        gcc \
        g++ \
        cmake \
        curl \
        wget \
        libasio-dev \
        libtinyxml2-dev \
        doxygen \
        python3 \
        python3-pip \
        python3-venv \
        python3-sphinxcontrib.spelling \
        imagemagick
    ```

1. Clone the repository

    ```bash
    cd ~
    git clone https://github.com/eProsima/Fast-RTPS-docs fastdds-docs
    ```

1. Create a virtual environment and install python3 dependencies.

    ```bash
    cd ~/fastdds-docs
    python3 -m venv fastdds-docs-venv
    source fastdds-docs-venv/bin/activate
    pip3 install -r docs/requirements.txt
    cd fastdds-docs-venv/lib/<python-version>/site-packages
    curl https://patch-diff.githubusercontent.com/raw/sphinx-doc/sphinx/pull/7851.diff | git apply
    cd -
    ```

    The version of python3 used in the virtual environment can be seen by running the following command within the virtual environment:

    ```bash
    python3 -V
    ```

## Getting Started

To generate the documentation in a HTML format for a specific branch of Fast DDS run:

```bash
cd ~/fastdds-docs
source fastdds-docs-venv/bin/activate
make html
```

### Selecting Fast DDS branch

It is possible to specify the Fast DDS branch for which the documentation is generated via the environment variable `FASTDDS_BRANCH`.

```bash
cd ~/fastdds-docs
source fastdds-docs-venv/bin/activate
FASTDDS_BRANCH=<branch> make help
```

## Generating documentation in other formats

The documentation can be generated in several formats such as HTML, PDF, LaTex, etc. For a complete list of targets run:

```bash
cd ~/fastdds-docs
make help
```

Once you have selected a format, generate the documentation with:

```bash
cd ~/fastdds-docs
source fastdds-docs-venv/bin/activate
FASTDDS_BRANCH=<branch> make <output_format>
```

## Running documentation tests

DISCLAIMER: In order to run documentation tests, access to eProsima's intranet is required.

This repository provides a set of tests that verify that:

1. The RST follows the style guidelines
1. The HTML is built correctly
1. The C++ snippets compile against the library's version
1. The XML snippets define valid configurations

Run the tests by:

```bash
cd ~/fastdds-docs
source fastdds-docs-venv/bin/activate
FASTDDS_BRANCH=<branch> make test
```

## Simulating Read the Docs

Read the Docs generates the documentation using Sphinx and [conf.py](docs/conf.py).
This means that it does not execute `make` and therefore Fast DDS is not downloaded for API reference documentation generation.
[conf.py](docs/conf.py) provides some extra logic to download Fast DDS and generate the Doxygen documentation when running on a Read the Docs environment.
This is done by means of the environment variable `READTHEDOCS`.
When this variable is set to `True`, [conf.py](docs/conf.py) will clone Fast DDS in `build/code/external/eprosima/src/` (same place as CMake) and will set it to a branch applying the following criteria:

1. Try to checkout to the branch specified by `FASTDDS_BRANCH`.
1. If the variable is not set, or the branch does not exist, try to checkout to a branch with the same name as the current branch on this repository.
1. If the previous fails, fallback to `master`.

To simulating Read the Docs operation, make sure you do not have a `build` directory.

```bash
cd ~/fastdds-docs
rm -rf build
```

Then, set `READTHEDOCS`, `FASTDDS_BRANCH` and run sphinx:

```bash
READTHEDOCS=True FASTDDS_BRANCH=<branch> sphinx-build \
    -b html \
    -Dbreathe_projects.FastDDS=<abs_path_to_docs_repo>/fastdds-docs/build/code/doxygen/xml \
    -d <abs_path_to_docs_repo>/fastdds-docs/build/doctrees \
    docs <abs_path_to_docs_repo>/fastdds-docs/build/html
```

## Contributing

If you are interested in making some contributions, either in the form of an issue or a pull request, please refer to our [Contribution Guidelines](https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md).
