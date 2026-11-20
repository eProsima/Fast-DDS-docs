<a id="linux-sources"></a>

<a id="linux-installation-from-sources"></a>

# Linux installation from sources

The instructions for installing the [Fast DDS library](#fastdds-lib-sl),
the [Fast DDS Python bindings](#fastdds-python-sl)
and the [Fast DDS-Gen](#fastddsgen-sl) generation tool from sources are provided in this page.
It is organized as follows:

<a id="fastdds-lib-sl"></a>

<a id="fast-dds-library-installation"></a>

## Fast DDS library installation

This section describes the instructions for installing *eProsima Fast DDS* in a Linux environment from sources.
The following packages will be installed:

* `foonathan_memory_vendor`, an STL compatible C++ memory allocator
  [library](https://github.com/foonathan/memory).
* `fastdds_gen`, a Java application that generates source code using the data types defined in an IDL file.
* `fastcdr`, a C++ library that serializes according to the
  [standard CDR](https://www.omg.org/cgi-bin/doc?formal/02-06-51) serialization mechanism.
* `fastdds`, the core library of *eProsima Fast DDS* library.

First of all, the [Requirements](#requirements-sl) and [Dependencies](#dependencies-sl) detailed below need to be met.
Afterwards, the user can choose whether to follow either the [colcon](#colcon-installation-linux)
or the [CMake](#cmake-installation-linux) installation instructions.

<a id="requirements-sl"></a>

<a id="requirements"></a>

### Requirements

The installation of *eProsima Fast DDS* in a Linux environment from sources requires the following tools to be
installed in the system:

* [CMake, g++, pip3, wget and git](#cmake-gcc-pip3-wget-git-sl)

<a id="cmake-gcc-pip3-wget-git-sl"></a>

<a id="cmake-g-pip3-wget-and-git"></a>

#### CMake, g++, pip3, wget and git

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Install [CMake](https://cmake.org), [g++](https://gcc.gnu.org/), [pip3](https://docs.python.org/3/installing/index.html), [wget](https://www.gnu.org/software/wget/) and [git](https://git-scm.com/) using the package manager of the appropriate
Linux distribution.
For example, on Ubuntu use the command:

```bash
sudo apt install cmake g++ python3-pip wget git
```

<a id="dependencies-sl"></a>

<a id="dependencies"></a>

### Dependencies

*eProsima Fast DDS* has the following dependencies, when installed from sources in a Linux environment:

* [Asio and TinyXML2 libraries](#asiotinyxml2-sl)
* [OpenSSL](#openssl-sl)
* [Libp11 and SoftHSM libraries](#libp11-sl)
* [Gtest](#gtest-sl) [optional]
* [XML validation tool](#pythonreq-sl) [optional]

#### SEE ALSO
For further information about this Fast DDS version dependencies, as well as for the corresponding versions of
other related products, please refer to the Fast DDS [Library dependencies](../../notes/versions.md#dependencies-compatibilities-library-dependencies)
section.

<a id="asiotinyxml2-sl"></a>

<a id="asio-and-tinyxml2-libraries"></a>

#### Asio and TinyXML2 libraries

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

```bash
sudo apt install libasio-dev libtinyxml2-dev
```

<a id="openssl-sl"></a>

<a id="openssl"></a>

#### OpenSSL

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install [OpenSSL](https://www.openssl.org/) using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

```bash
sudo apt install libssl-dev
```

<a id="libp11-sl"></a>

<a id="libp11-and-softhsm-libraries"></a>

#### Libp11 and SoftHSM libraries

Libp11 provides PKCS#11 support for OpenSSL.
This is an optional dependency, that is needed only when *eprosima Fast DDS* is used with security and PKCS#11 URIs.

Install [libp11](https://github.com/OpenSC/libp11/) using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

```bash
sudo apt install libp11-dev
```

SoftHSM is a software implementation of an HSM (Hardware Security Module).
If *eProsima Fast DDS* tests are activated and *libp11* is installed
on the system, SoftHSM is additionally required to run tests of PKCS#11 features.

Install [SoftHSM](https://www.opendnssec.org/softhsm/) using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

```bash
sudo apt install softhsm2
```

Note that the *softhsm2* package creates a new group called *softhsm*.
In order to grant access to the HSM module a user must belong to this group.

```bash
sudo usermod -a -G softhsm $USER
```

#### WARNING
If you add your current user to a new group, you must log out and log back in for the change to take effect.

OpenSSL access HSM and other hardware devices through its engine functionality.
In order to set up a new engine the OpenSSL configuration files (usually /etc/ssl/openssl.cnf)
must be updated specifying the [libp11](https://github.com/OpenSC/libp11/) and hardware module (here [SoftHSM](https://www.opendnssec.org/softhsm/)) dynamic libraries
location.

This configuration step can be avoided using [p11kit](https://github.com/p11-glue/p11-kit) which allows OpenSSL to find PKCS#11
devices on runtime without static configuration.
This kit is often available through the Linux distribution package manager.
On Ubuntu, for example:

```bash
sudo apt install libengine-pkcs11-openssl
```

Once installed, to check [p11kit](https://github.com/p11-glue/p11-kit) is able to find the [SoftHSM](https://www.opendnssec.org/softhsm/) module use:

```bash
p11-kit list-modules
```

In order to check if OpenSSL is able to access PKCS#11 engine use:

```bash
openssl engine pkcs11 -t
```

<a id="gtest-sl"></a>

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

<a id="pythonreq-sl"></a>

<a id="xml-validation-tool"></a>

#### XML validation tool

XML validation is a new command introduced to validate the XML profiles against an XSD schema through Fast DDS CLI.
That ensures the proper characterization of the entities using the xml profiles.

For more details, please refer to the [xml](../../fastddscli/cli/cli.md#cli-xml) section.

Install the [xmlschema](https://pypi.org/project/xmlschema/) dependency to be able to use this optional tool.

<a id="colcon-installation-linux"></a>

<a id="colcon-installation"></a>

### Colcon installation

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org) aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

1. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:
   ```bash
   pip3 install -U colcon-common-extensions vcstool
   ```

   #### NOTE
   Mind that under non-root users, `pip3` may install python `colcon` and `vcs` executables in
   `$HOME/.local/bin`, for instance when running with `--user`.
   To be able to run these applications, make sure that `pip3` binary installation directory is in your
   `$PATH` (`$HOME/.local/bin` is normally introduced while login on an interactive non-root shell).
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
Being based on [CMake](https://cmake.org), it is possible to pass CMake configuration options to the `colcon build` command.
For more information on the specific syntax, please refer to the
[CMake specific arguments](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments)
page of the [colcon](https://colcon.readthedocs.io/en/released/) manual.

<a id="run-app-colcon-sl"></a>

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
  echo 'source ~/Fast-DDS/install/setup.bash' >> ~/.bashrc
  ```

<a id="cmake-installation-linux"></a>

<a id="cmake-installation"></a>

### CMake installation

This section explains how to compile *eProsima Fast DDS* with [CMake](https://cmake.org), either [locally](#local-installation-sl) or
[globally](#global-installation-sl).

<a id="local-installation-sl"></a>

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
     cmake --build . --target install
     ```
   * [Fast CDR](https://github.com/eProsima/Fast-CDR.git)
     ```bash
     cd ~/Fast-DDS
     git clone https://github.com/eProsima/Fast-CDR.git
     mkdir Fast-CDR/build
     cd Fast-CDR/build
     cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
     cmake --build . --target install
     ```
3. Once all dependencies are installed, install *eProsima Fast DDS*:
   ```bash
   cd ~/Fast-DDS
   git clone https://github.com/eProsima/Fast-DDS.git
   mkdir Fast-DDS/build
   cd Fast-DDS/build
   cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
   cmake --build . --target install
   ```

#### NOTE
By default, *eProsima Fast DDS* does not compile tests.
However, they can be activated by downloading and installing [Gtest](https://github.com/google/googletest).

<a id="global-installation-sl"></a>

<a id="global-installation"></a>

#### Global installation

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of `Fast-CDR` and `Fast-DDS`, and change the first in the
configuration step of `foonathan_memory_vendor` to the following:

```bash
-DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
```

#### NOTE
Installation on system directories may need of permissions.
Maybe permissions have to be granted through `sudo`.

```bash
sudo cmake --build . --target install
```

<a id="run-app-cmake-sl"></a>

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
  echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc
  ```

<a id="fastdds-python-sl"></a>

<a id="fast-dds-python-bindings-installation"></a>

## Fast DDS Python bindings installation

This section provides the instructions for installing *Fast DDS Python bindings* in a Linux environment from sources.
*Fast DDS Python bindings* is an extension of *Fast DDS* which provides access to the Fast DDS API through Python.
Therefore, its installation is an extension of the installation of [Fast DDS](#fastdds-lib-sl).

*Fast DDS Python bindings* source code consists on several .i files which will be processed by [SWIG](http://www.swig.org/).
Then C++ files (for connecting C++ and Python) and Python files (Python module for Fast DDS) will be generated.

First of all, the [Requirements](#requirements-python-sl) and [Dependencies](#dependencies-python-sl) detailed below need to be met.
Afterwards, the user can choose whether to follow either the [colcon](#colcon-installation-python-linux) or the
[CMake](#cmake-installation-python-linux) installation instructions.

<a id="requirements-python-sl"></a>

<a id="id3"></a>

### Requirements

The installation of *Fast DDS Python bindings* in a Linux environment from sources requires the following tools to be
installed in the system:

- [Fast DDS requirements](#requirements-sl)
- [SWIG](#swig-python-sl)
- [Header files and static library for Python](#libpython-dev-sl)

<a id="swig-python-sl"></a>

<a id="swig"></a>

#### SWIG

[SWIG](http://www.swig.org/) is a development tool that allows connecting programs written in C/C++ with a variety of
other programming languages, among them Python.
SWIG version lower than 4.2 is required to build Fast DDS Python bindings.

#### NOTE
More recent [SWIG](http://www.swig.org/) releases are not yet supported.
Please, ensure to be using SWIG 4.0.

<!-- end-windows-swig -->

[SWIG](http://www.swig.org/) can be installed directly from the package manager of the appropriate Linux distribution.
For Ubuntu, please run:

```bash
sudo apt install swig
```

#### WARNING
For Ubuntu 24.04, [SWIG](http://www.swig.org/) must be installed using the following command:

```bash
sudo apt install swig4.1
```

<a id="libpython-dev-sl"></a>

<a id="header-files-and-static-library-for-python"></a>

#### Header files and static library for Python

Python static libraries and header files are needed to compile C++ source code generated by [SWIG](http://www.swig.org/).
They can be installed directly from the package manager of the appropriate Linux distribution.
For Ubuntu, please run:

```bash
sudo apt install libpython3-dev
```

<a id="dependencies-python-sl"></a>

<a id="id4"></a>

### Dependencies

*Fast DDS Python bindings* has the following dependencies, when installed from sources in a Linux environment:

- [Fast DDS dependencies](#dependencies-sl)

<a id="colcon-installation-python-linux"></a>

<a id="id5"></a>

### Colcon installation

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org) aimed at building sets of software packages.
This section explains how to use it to compile *Fast DDS Python bindings* and its dependencies.

1. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:
   ```bash
   pip3 install -U colcon-common-extensions vcstool
   ```

   #### NOTE
   If this fails due to an Environment Error, add the `--user` flag to the `pip3` installation command.
2. Create a `Fast-DDS-python` directory and download the repos file that will be used to install
   *Fast DDS Python bindings* and its dependencies:
   ```bash
    mkdir ~/Fast-DDS-python
    cd ~/Fast-DDS-python
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS-python/master/fastdds_python.repos
    mkdir src
    vcs import src < fastdds_python.repos
   ```
3. Build the packages:
   ```bash
   colcon build --packages-up-to fastdds_python
   ```

#### NOTE
Being based on [CMake](https://cmake.org), it is possible to pass CMake configuration options to the `colcon build` command.
For more information on the specific syntax, please refer to the
[CMake specific arguments](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments)
page of the [colcon](https://colcon.readthedocs.io/en/released/) manual.

<a id="id7"></a>

#### Run an application

When running an instance of an application using *Fast DDS Python bindings*, the colcon overlay built in the
dedicated `Fast-DDS-python` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:
  ```bash
  source ~/Fast-DDS-python/install/setup.bash
  ```
* Add the sourcing of the colcon overlay permanently to the `PATH`, by typing the following:
  ```bash
  echo 'source ~/Fast-DDS-python/install/setup.bash' >> ~/.bashrc
  ```

<a id="cmake-installation-python-linux"></a>

<a id="id8"></a>

### CMake installation

This section explains how to compile *Fast DDS Python bindings* with [CMake](https://cmake.org), either
[locally](#local-installation-python-sl) or [globally](#global-installation-python-sl).

<a id="local-installation-python-sl"></a>

<a id="id9"></a>

#### Local installation

1. Create a `Fast-DDS-python` directory where to download and build *Fast DDS Python bindings* and its
   dependencies:
   ```bash
   mkdir ~/Fast-DDS-python
   ```
2. Clone the following dependencies and compile them using [CMake](https://cmake.org).
   * [Foonathan memory](https://github.com/foonathan/memory)
     ```bash
     cd ~/Fast-DDS-python
     git clone https://github.com/eProsima/foonathan_memory_vendor.git
     mkdir foonathan_memory_vendor/build
     cd foonathan_memory_vendor/build
     cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install -DBUILD_SHARED_LIBS=ON
     cmake --build . --target install
     ```
   * [Fast CDR](https://github.com/eProsima/Fast-CDR.git)
     ```bash
     cd ~/Fast-DDS-python
     git clone https://github.com/eProsima/Fast-CDR.git
     mkdir Fast-CDR/build
     cd Fast-CDR/build
     cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
     cmake --build . --target install
     ```
   * [Fast DDS](https://github.com/eProsima/Fast-DDS.git)
     ```bash
     cd ~/Fast-DDS-python
     git clone https://github.com/eProsima/Fast-DDS.git
     mkdir Fast-DDS/build
     cd Fast-DDS/build
     cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
     cmake --build . --target install
     ```
3. Once all dependencies are installed, install *Fast DDS Python bindings*:
   ```bash
   cd ~/Fast-DDS-python
   git clone https://github.com/eProsima/Fast-DDS-python.git
   mkdir -p Fast-DDS-python/fastdds_python/build
   cd Fast-DDS-python/fastdds_python/build
   cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
   cmake --build . --target install
   ```

<a id="global-installation-python-sl"></a>

<a id="id12"></a>

#### Global installation

To install *Fast DDS Python bindings* system-wide instead of locally, remove all the flags that
appear in the configuration steps of `Fast-CDR`, `Fast-DDS` and `Fast-DDS-python`, and change the
first in the configuration step of `foonathan_memory_vendor` to the following:

```bash
-DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
```

#### NOTE
Installation on system directories may need of permissions.
Maybe permissions have to be granted through `sudo`.

```bash
sudo cmake --build . --target install
```

<a id="id13"></a>

#### Run an application

When running an instance of an application using *Fast DDS Python bindings*, it must be linked with the library where
the packages have been installed, which in the case of system-wide installation is: `/usr/local/lib/` (if local
installation is used, adjust for the correct directory).
There are two possibilities:

* Prepare the environment locally by typing the command:
  ```bash
  export LD_LIBRARY_PATH=/usr/local/lib/
  ```
* Add it permanently it to the `PATH`, by typing:
  ```bash
  echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc
  ```

<a id="fastddsgen-sl"></a>

<a id="fast-dds-gen-installation"></a>

## Fast DDS-Gen installation

This section provides the instructions for installing *Fast DDS-Gen* in a Linux environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
Please refer to [Introduction](../../fastddsgen/introduction/introduction.md#fastddsgen-intro) for more information, and to
[eProsima products compatibility](../../notes/versions.md#dependencies-compatibilities-product-compatibility) for the compatibility matrix against Fast DDS versions.

<a id="id14"></a>

### Requirements

*Fast DDS-Gen* is built using Gradle.
Gradle is an open-source build automation tool which requires a Java version to be executed (see
[Gradle-Java compatibility matrix](https://docs.gradle.org/current/userguide/compatibility.html)).

#### IMPORTANT
Even though earlier versions of Gradle support Java 11, *Fast DDS-Gen* stopped supporting Java versions previous to
Java 17 since release v4.3.0.

#### IMPORTANT
*Fast DDS-Gen* introduced support for Gradle 9 in release v4.3.0.

#### SEE ALSO
For further information about Fast DDS-Gen product related versions, please refer to the
[Library dependencies](../../notes/versions.md#dependencies-compatibilities-library-dependencies) section.

<a id="java-sl"></a>

<a id="java-jdk"></a>

#### Java JDK

The JDK is a development environment for building applications and components using the Java language.
There are several versions of Java available.
For instance, to install Java 17 JDK, run the following command:

```bash
sudo apt install openjdk-17-jdk
```

#### NOTE
*Fast DDS-Gen* supports Java versions from 17 to 21.

<a id="compiling-fast-dds-gen"></a>

### Compiling Fast DDS-Gen

In order to compile *Fast DDS-Gen*, an executable script is included in the repository which will download Gradle
temporarily for the compilation step.
Please, follow the steps below to build *Fast DDS-Gen*:

#### NOTE
If Fast DDS has already been installed following [Colcon installation](#colcon-installation-linux), skip cloning *Fast DDS-Gen*’s
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
  `PATH` environment variable.

<!-- External links -->
