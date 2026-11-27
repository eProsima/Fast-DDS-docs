<a id="mac-sources"></a>

<a id="mac-os-installation-from-sources"></a>

# Mac OS installation from sources

The instructions for installing both the [Fast DDS library](#fastdds-lib-sm)
and the [Fast DDS-Gen](#fastddsgen-sm) generation tool from sources are provided in this page.
It is organized as follows:

<a id="fastdds-lib-sm"></a>

<a id="fast-dds-library-installation"></a>

## Fast DDS library installation

This section describes the instructions for installing *eProsima Fast DDS* in a Mac OS environment from
sources. The following packages will be installed:

* `foonathan_memory_vendor`, an STL compatible C++ memory allocator
  [library](https://github.com/foonathan/memory).
* `fastdds_gen`, a Java application that generates source code using the data types defined in an IDL file.
* `fastcdr`, a C++ library that serializes according to the
  [standard CDR](https://www.omg.org/cgi-bin/doc?formal/02-06-51) serialization mechanism.
* `fastdds`, the core library of *eProsima Fast DDS* library.

First of all, the [Requirements](#requirements-sm) and [Dependencies](#dependencies-sm) detailed below need to be met.
Afterwards, the user can choose whether to follow either the [colcon](#colcon-installation-mac))
or the [CMake](#cmake-installation-mac)) installation instructions.

<a id="requirements-sm"></a>

<a id="requirements"></a>

### Requirements

The installation of *eProsima Fast DDS* in a MacOS environment from sources requires the following tools to be
installed in the system:

* [Homebrew](#homebrew-sm)
* [Xcode Command Line Tools](#xcode-cli-sm)
* [CMake, g++, pip3, wget and git](#cmake-gcc-pip3-wget-git-sm)
* [Gtest](#gtest-sm) [optional]
* [XML validation tool](#pythonreq-sm) [optional]

<a id="homebrew-sm"></a>

<a id="homebrew"></a>

#### Homebrew

Homebrew is a macOS package manager, it is needed to install some of *eProsima Fast DDS*’s dependencies.
To install it open a terminal window and run the following command.

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

<a id="xcode-cli-sm"></a>

<a id="xcode-command-line-tools"></a>

#### Xcode Command Line Tools

The Xcode command line tools package is separate from Xcode and allows for command line development in mac.
The previous step should have installed Xcode CLI, to check the correct installation run the following command:

```default
gcc --version
```

<a id="cmake-gcc-pip3-wget-git-sm"></a>

<a id="cmake-g-pip3-wget-and-git"></a>

#### CMake, g++, pip3, wget and git

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Install [CMake](https://cmake.org), [pip3](https://docs.python.org/3/installing/index.html) and [wget](https://www.gnu.org/software/wget/) using the Homebrew package manager:

```bash
brew install cmake python3 wget
```

<a id="gtest-sm"></a>

<a id="gtest"></a>

#### Gtest

GTest is a unit testing library for C++.
By default, *eProsima Fast DDS* does not compile tests.
It is possible to activate them with the opportune
[CMake configuration options](https://cmake.org/cmake/help/v3.6/manual/cmake.1.html#options)
when calling [colcon](https://colcon.readthedocs.io/en/released/) or [CMake](https://cmake.org).
For more details, please refer to the [CMake options](../configuration/cmake_options.md#cmake-options) section.
Also add the [Gtest repository](https://github.com/google/googletest) into the workspace directory.

```bash
git clone --branch release-1.11.0 https://github.com/google/googletest src/googletest-distribution
```

<a id="pythonreq-sm"></a>

<a id="xml-validation-tool"></a>

#### XML validation tool

XML validation is a new command introduced to validate the XML profiles against an XSD schema through Fast DDS CLI.
That ensures the proper characterization of the entities using the xml profiles.

For more details, please refer to the [xml](../../fastddscli/cli/cli.md#cli-xml) section.

Install the [xmlschema](https://pypi.org/project/xmlschema/) dependency to be able to use this optional tool.

<a id="dependencies-sm"></a>

<a id="dependencies"></a>

### Dependencies

*eProsima Fast DDS* has the following dependencies, when installed from binaries in a Linux environment:

* [Asio and TinyXML2 libraries](#asiotinyxml2-sm)
* [OpenSSL](#openssl-sm)

#### SEE ALSO
For further information about this Fast DDS version dependencies, as well as for the corresponding versions of
other related products, please refer to the Fast DDS [Library dependencies](../../notes/versions.md#dependencies-compatibilities-library-dependencies)
section.

<a id="asiotinyxml2-sm"></a>

<a id="asio-and-tinyxml2-libraries"></a>

#### Asio and TinyXML2 libraries

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using Homebrew:

```bash
brew install asio tinyxml2
```

<a id="openssl-sm"></a>

<a id="openssl"></a>

#### OpenSSL

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install [OpenSSL](https://www.openssl.org/) using Homebrew:

```bash
brew install openssl@1.1
```

<a id="colcon-installation-mac"></a>

<a id="colcon-installation"></a>

### Colcon installation

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org) aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

1. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:
   ```bash
   pip3 install -U colcon-common-extensions vcstool
   ```
2. Create a `Fast-DDS` directory and download the repos file that will be used to install
   *eProsima Fast DDS* and its dependencies:
   ```bash
    mkdir ~/Fast-DDS
    cd ~/Fast-DDS
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastdds.repos
    mkdir src
    vcs import src < fastdds.repos
   ```
3. Build the packages:
   ```bash
   colcon build --packages-up-to fastdds
   ```

#### NOTE
The `--cmake-args` option allows to pass the CMake configuration options to the `colcon build` command.
In Mac OS the location of OpenSSL is not found automatically and therefore has to be passed explicitly:
`--cmake-args -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib`.
This is only required when building with [Security](../../fastdds/security/security.md#security).
For more information on the specific syntax, please refer to the [CMake specific arguments](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments) page of the [colcon](https://colcon.readthedocs.io/en/released/)
manual.

<a id="run-app-colcon-sm"></a>

<a id="run-an-application"></a>

#### Run an application

When running an instance of an application using *eProsima Fast DDS*, the colcon overlay built in the
dedicated `Fast-DDS` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:
  ```bash
  source ~/Fast-DDS/install/setup.bash
  ```
* Add the sourcing of the colcon overlay permanently to the `PATH`, by typing the following:
  ```bash
  touch ~/.bash_profile
  echo 'source ~/Fast-DDS/install/setup.bash' >> ~/.bash_profile
  ```

<a id="cmake-installation-mac"></a>

<a id="cmake-installation"></a>

### CMake installation

This section explains how to compile *eProsima Fast DDS* with [CMake](https://cmake.org), either [locally](#local-installation-sm) or
[globally](#global-installation-sm).

<a id="local-installation-sm"></a>

<a id="local-installation"></a>

#### Local installation

1. Create a `Fast-DDS` directory where to download and build *eProsima Fast DDS* and its dependencies:
   ```bash
   mkdir ~/Fast-DDS
   ```
2. Clone the following dependencies and compile them using [CMake](https://cmake.org).
   * [Foonathan memory](https://github.com/foonathan/memory)
     ```bash
     cd ~/Fast-DDS
     git clone https://github.com/eProsima/foonathan_memory_vendor.git
     mkdir foonathan_memory_vendor/build
     cd foonathan_memory_vendor/build
     cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DBUILD_SHARED_LIBS=ON
     sudo cmake --build . --target install
     ```
   * [Fast CDR](https://github.com/eProsima/Fast-CDR.git)
     ```bash
     cd ~/Fast-DDS
     git clone https://github.com/eProsima/Fast-CDR.git
     mkdir Fast-CDR/build
     cd Fast-CDR/build
     cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
     sudo cmake --build . --target install
     ```
3. Once all dependencies are installed, install *eProsima Fast DDS*:
   ```bash
   cd ~/Fast-DDS
   git clone https://github.com/eProsima/Fast-DDS.git
   mkdir Fast-DDS/build
   cd Fast-DDS/build
   cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DCMAKE_PREFIX_PATH=~/Fast-DDS/install
   sudo cmake --build . --target install
   ```

#### NOTE
By default, *eProsima Fast DDS* does not compile tests.
However, they can be activated by downloading and installing [Gtest](https://github.com/google/googletest).

<a id="global-installation-sm"></a>

<a id="global-installation"></a>

#### Global installation

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of `Fast-CDR` and `Fast-DDS`, and change the first in the
configuration step of `foonathan_memory_vendor` to the following:

```bash
-DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
```

<a id="run-app-cmake-sm"></a>

<a id="id2"></a>

#### Run an application

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed, which in the case of system-wide installation  is: `/usr/local/lib/` (if local
installation is used, adjust for the correct directory).
There are two possibilities:

* Prepare the environment locally by typing the command:
  ```bash
  export LD_LIBRARY_PATH=/usr/local/lib/
  ```
* Add it permanently it to the `PATH`, by typing:
  ```bash
  touch ~/.bash_profile
  echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bash_profile
  ```

<a id="fastddsgen-sm"></a>

<a id="fast-dds-gen-installation"></a>

## Fast DDS-Gen installation

This section provides the instructions for installing *Fast DDS-Gen* in a Mac OS environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
Please refer to [Introduction](../../fastddsgen/introduction/introduction.md#fastddsgen-intro) for more information, and to
[eProsima products compatibility](../../notes/versions.md#dependencies-compatibilities-product-compatibility) for the compatibility matrix against Fast DDS versions.

<a id="id3"></a>

### Requirements

*Fast DDS-Gen* is built using Gradle.
Gradle is an open-source build automation tool which requires a Java version to be executed (see
[Gradle-Java compatibility matrix](https://docs.gradle.org/current/userguide/compatibility.html)).

#### IMPORTANT
Even though earlier versions of Gradle support Java 8, *Fast DDS-Gen* stopped supporting Java versions previous to
Java 11 since release v2.4.0.

#### IMPORTANT
*Fast DDS-Gen* introduced support for Gradle 7 in release v2.2.0.
Gradle 8 is not yet supported.

#### SEE ALSO
For further information about Fast DDS-Gen product related versions, please refer to the
[Library dependencies](../../notes/versions.md#dependencies-compatibilities-library-dependencies) section.

<a id="java-sm"></a>

<a id="java-jdk"></a>

#### Java JDK

The JDK is a development environment for building applications and components using the Java language.
Download and install it following the steps given in the
[Oracle website](https://www.oracle.com/java/technologies/javase-downloads.html).

#### NOTE
*Fast DDS-Gen* supports Java versions from 11 to 19.

<a id="compiling-fast-dds-gen"></a>

### Compiling Fast DDS-Gen

In order to compile *Fast DDS-Gen*, an executable script is included in the repository which will download Gradle
temporarily for the compilation step.
Please, follow the steps below to build *Fast DDS-Gen*:

#### NOTE
If Fast DDS has already been installed following [Colcon installation](#colcon-installation-mac), skip cloning *Fast DDS-Gen*’s
repository, as it can already be found under the `src` directory within the colcon workspace.

```bash
mkdir -p ~/Fast-DDS/src
cd ~/Fast-DDS/src
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen
cd fastddsgen
./gradlew assemble
```

#### NOTE
In case that a supported Gradle version is already installed in the system, *Fast DDS-Gen* can also be built running
directly:

```bash
gradle assemble
```

<a id="contents"></a>

#### Contents

The `Fast-DDS-Gen` folder contains the following packages:

* `share/fastddsgen`, where the generated Java application is.
* `scripts`, containing some user friendly scripts.

  #### NOTE
  To make these scripts accessible from any shell session and directory, add the `scripts` folder path to the
  `PATH` environment variable using the method described above.

<!-- External links -->
