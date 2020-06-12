# eProsima Fast DDS Documentation

[![Releases](https://img.shields.io/github/release/eProsima/Fast-DDS.svg)](https://github.com/eProsima/Fast-DDS/releases)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Linux Build Status](http://jenkins.eprosima.com:8080/job/nightly_fastdds-docs_master/badge/icon)](http://jenkins.eprosima.com:8080/job/nightly_fastdds-docs_master/)

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

*eprosima Fast DDS* is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium. RTPS is also the wire interoperability protocol defined for the Data Distribution Service (DDS) standard, again by the OMG. eProsima Fast DDS holds the benefit of being standalone and up-to-date, as most vendor solutions either implement RTPS as a tool to implement DDS or use past versions of the specification.

For more information about the library, check out the [Fast DDS documentation](https://fast-dds.docs.eprosima.com/en/latest/).
You can find all the library's source code on our [GitHub repository](https://github.com/eProsima/Fast-DDS).

1. [Installation Guide](#installation-guide)
1. [Getting Started](#getting-started)
1. [Generating documentation in other formats](#generating-documentation-in-other-formats)
1. [Running documentation tests](#running-documentation-tests)
1. [Simulating Read the Docs](#simulating-read-the-docs)

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

1. Create a virtual environment and install python3 dependencies

    ```bash
    cd ~/fastdds-docs
    python3 -m venv fastdds-docs-venv
    source fastdds-docs-venv/bin/activate
    pip3 install -r docs/requirements.txt
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
