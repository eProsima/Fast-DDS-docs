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

1. [Installation Guide](#installation-guide)
1. [Project structure](#project-structure)
1. [Contributing](#contributing)

## Installation Guide

The following guide has been developed and tested in Ubuntu 20.04.
It covers two mechanisms for build and testing the documentation (colcon and CMake), and one that just generates the HTML, which can be used to simulate the process followed in ReadTheDocs.
Contributors are asked to not only generate the HTML, but to also build and test their changes locally following one of the two first methods.

1. [Install common dependencies](#install-common-dependencies)
1. [Python virtual environment](#python-virtual-environment)
1. [Colcon installation](#colcon-installation)
1. [CMake installation](#cmake-installation)
1. [Simulating Read the Docs](#simulating-read-the-docs)
1. [Generating documentation in other formats](#generating-documentation-in-other-formats)

### Install common dependencies

Install dependencies common to all methods:

```bash
sudo apt update
sudo apt install -y \
    git \
    curl \
    wget \
    doxygen \
    python3-doc8 \
    python3 \
    python3-pip \
    python3-venv \
    imagemagick \
    libenchant-2-2 \
    plantuml
```

### Python virtual environment

In order to build the documentation, some python3 dependencies are required.
This guide uses a python3 virtual environment to install such dependencies, thus avoiding polluting the user's installation.
Create a virtual environment and install python3 dependencies.

```bash
python3 -m venv fastdds-docs-venv
source fastdds-docs-venv/bin/activate
wget https://raw.githubusercontent.com/eProsima/Fast-DDS-docs/master/docs/requirements.txt
pip3 install -r requirements.txt
```

The version of python3 used in the virtual environment can be seen by running the following command within the virtual environment:

```bash
python3 -V
```

### Colcon installation

1. Fast DDS-docs depends on Fast DDS, so its dependencies must be installed alongside some required
building tools.

    ```bash
    sudo apt update
    sudo apt install -y \
        gcc \
        g++ \
        cmake \
        libasio-dev \
        libtinyxml2-dev \
        libssl-dev \
        python3-sphinx
    python -m pip install sphinx-toolbox
    pip install xmlschema
    ```

1. Create a colcon workspace containing Fast DDS and Fast DDS-docs:

    ```bash
    mkdir -p <path_to_ws>/fastdds-docs_ws/src
    cd <path_to_ws>/fastdds-docs_ws
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS-docs/<version>/colcon.meta
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS/<version>/fastdds.repos
    vcs import src < fastdds.repos
    cd src
    git clone https://github.com/eProsima/Fast-DDS-docs fastdds-docs
    ```

1. [OPTIONAL]: You can checkout to different Fast DDS and Fast DDS-docs branches within the appropriate repositories located in `<path_to_ws>/fastdds-docs_ws/src`

1. Build the workspace using colcon[^colcon_ignore]

    ```bash
    source <path_to_venv>/fastdds-docs-venv/bin/activate
    cd <path_to_ws>/fastdds-docs_ws
    colcon build
    ```

[^colcon_ignore]: If the virtual environment is placed within the colcon workspace, it is recommended to add an empty `COLCON_IGNORE` file in the root of the virtual environment so that colcon does not inspect it.

1. Run documentation tests:

    ```bash
    source <path_to_venv>/fastdds-docs-venv/bin/activate
    cd <path_to_ws>/fastdds-docs_ws
    colcon test --packages-select fastdds-docs --event-handlers=console_direct+
    ```


### CMake installation

Fast DDS-docs is a CMake project, and as such it can be build using CMake directly.
This guide does not cover building Fast DDS or installing its dependencies.
However, it is required to have a Fast DDS installation built with option `-DSECURITY=ON`.
It is up to the user to link to the appropriate Fast DDS installation using, for instance,
`CMAKE_PREFIX_PATH`.

```bash
source <path_to_venv>/fastdds-docs-venv/bin/activate
git clone https://github.com/eProsima/Fast-DDS-docs fastdds-docs
cd fastdds-docs
mkdir build
cd build
cmake ..  -DBUILD_DOCUMENTATION=ON -DCOMPILE_TESTS=ON
cmake --build .
```

Now, you can run documentation tests:

```bash
source <path_to_venv>/fastdds-docs-venv/bin/activate
cd <path_to_repo>/fastdds-docs/build
ctest -VV
```

### Simulating Read the Docs

ReadTheDocs generates the documentation using Sphinx and [conf.py](docs/conf.py).
This means that it does not execute any colcon or CMake commands.
Furthermore, Fast DDS is not available for API reference documentation generation.
[conf.py](docs/conf.py) provides some extra logic to download Fast DDS and generate the Doxygen documentation when running on a Read the Docs environment.
This is done by means of the environment variable `READTHEDOCS`.
When this variable is set to `True`, [conf.py](docs/conf.py) will clone Fast DDS in `build/fastdds/` and will set it to a specific branch according to the following criteria:

1. Try to checkout to the branch specified by environment variable `FASTDDS_BRANCH`.
1. If the variable is not set, or the branch does not exist, try to checkout to a branch with the same name as the current branch on this repository.
1. If the previous fails, fallback to `master`.

Also Fast DDS Python bindings is cloned and follows a similar criteria:

1. Try to checkout to the branch specified by environment variable `FASTDDS_PYTHON_BRANCH`.
1. If the variable is not set, or the branch does not exist, try to checkout to a branch with the same name as the current branch on this repository.
1. If the previous fails, fallback to `main`.

To simulate ReadTheDocs operation, make sure you do not have a `build` directory.
Then, set `READTHEDOCS`, `FASTDDS_BRANCH` and `FASTDDS_PYTHON_BRANCH` and run sphinx:

```bash
source <path_to_venv>/fastdds-docs-venv/bin/activate
cd <path_to_docs_repo>/fastdds-docs
rm -rf build
READTHEDOCS=True FASTDDS_BRANCH=<branch> FASTDDS_PYTHON_BRANCH=<branch> sphinx-build \
    -b html \
    -D breathe_projects.FastDDS=<abs_path_to_docs_repo>/fastdds-docs/build/doxygen/xml \
    -d <abs_path_to_docs_repo>/fastdds-docs/build/doctrees \
    docs <abs_path_to_docs_repo>/fastdds-docs/build/html
```

### Generating documentation in other formats

Using either CMake or colcon, the documentation is built using Sphinx's `html` builder.
However, Sphinx supports several other building formats, which are enabled through [Sphinx builders](https://www.sphinx-doc.org/en/master/usage/builders/index.html).
Once a builder is selected, the documentation can be built using the [Simulating Read the Docs](#simulating-read-the-docs) approach, specifying the appropriate builder with the `-b` CLI option

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
* [03-exports](docs/03-exports): Files for common aliases so they can be included when needed
* [fastdds](docs/fastdds): Fast DDS documentation.
* [fastddscli](docs/fastddscli): Fast DDS CLI documentation.
* [fastddsgen](docs/fastddsgen): Fast DDS-Gen documentation.
* [installation](docs/installation): Installation manual.
* [notes](docs/notes): Release notes.

All new documentation must fall into one of these directories, with the exception of those contributions which are not related to any of the given descriptions.
Keep in mind that this is an Sphinx based project, and as such, all the documentation is written in `reStructuredText`.

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

## Contributing

If you are interested in making some contributions, either in the form of an issue or a pull request, please refer to our [Contribution Guidelines](https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md).
