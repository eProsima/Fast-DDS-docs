<a id="windows-sources"></a>

<a id="windows-installation-from-sources"></a>

# Windows installation from sources

The instructions for installing both the [Fast DDS library](#fastdds-lib-sw)
and the [Fast DDS-Gen](#fastddsgen-sw) generation tool from sources are provided in this page.
It is organized as follows:

<a id="fastdds-lib-sw"></a>

<a id="fast-dds-library-installation"></a>

## Fast DDS library installation

This section provides the instructions for installing *eProsima Fast DDS* in a Windows environment from sources.
The following packages will be installed:

* `foonathan_memory_vendor`, an STL compatible C++ memory allocator
  [library](https://github.com/foonathan/memory).
* `fastdds_gen`, a Java application that generates source code using the data types defined in an IDL file.
* `fastcdr`, a C++ library that serializes according to the
  [standard CDR](https://www.omg.org/cgi-bin/doc?formal/02-06-51) serialization mechanism.
* `fastdds`, the core library of *eProsima Fast DDS* library.

First of all, the [Requirements](#requirements-sw) and [Dependencies](#dependencies-sw) detailed below need to be met.
Afterwards, the user can choose whether to follow either the [colcon](#colcon-installation-windows)
or the [CMake](#cmake-installation-windows) installation instructions.

<a id="requirements-sw"></a>

<a id="requirements"></a>

### Requirements

The installation of *eProsima Fast DDS* in a Windows environment from sources requires the following tools to be
installed in the system:

* [Visual Studio](#visual-studio-sw)
* [Chocolatey](#chocolatey-sw)
* [CMake, pip3, wget and git](#cmake-pip3-wget-git-sw)
* [Gtest](#gtest-sw) [optional]
* [XML validation tool](#pythonreq-sw) [optional]

<a id="visual-studio-sw"></a>

<a id="visual-studio"></a>

#### Visual Studio

[Visual Studio](https://visualstudio.microsoft.com/) is required to have a C++ compiler in the system.
For this purpose, make sure to check the `Desktop development with C++` option during the Visual Studio
installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to `Tools` -> `Get Tools and Features` and in the `Workloads` tab enable
`Desktop development with C++`.
Finally, click `Modify` at the bottom right.

<a id="chocolatey-sw"></a>

<a id="chocolatey"></a>

#### Chocolatey

Chocolatey is a Windows package manager.
It is needed to install some of *eProsima Fast DDS*’s dependencies.
Download and install it directly from the [website](https://chocolatey.org/).

<a id="cmake-pip3-wget-git-sw"></a>

<a id="cmake-pip3-wget-and-git"></a>

#### CMake, pip3, wget and git

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Download and install [CMake](https://cmake.org), [pip3](https://docs.python.org/3/installing/index.html), [wget](https://www.gnu.org/software/wget/) and [git](https://git-scm.com/) by following the instructions detailed in the respective
websites.
Once installed, add the path to the executables to the `PATH` from the
*Edit the system environment variables* control panel.

<a id="gtest-sw"></a>

<a id="gtest"></a>

#### Gtest

GTest is a unit testing library for C++.
By default, *eProsima Fast DDS* does not compile tests.
It is possible to activate them with the opportune
[CMake configuration options](https://cmake.org/cmake/help/v3.6/manual/cmake.1.html#options)
when calling [colcon](https://colcon.readthedocs.io/en/released/) or [CMake](https://cmake.org).
For more details, please refer to the [CMake options](../configuration/cmake_options.md#cmake-options) section.
Also add the [Gtest repository](https://github.com/google/googletest) into the workspace directory.

```winbatch
git clone --branch release-1.11.0 https://github.com/google/googletest src/googletest-distribution
```

and add next argument to the colcon call

```winbatch
colcon build --cmake-args -Dgtest_force_shared_crt=ON
```

<a id="pythonreq-sw"></a>

<a id="xml-validation-tool"></a>

#### XML validation tool

XML validation is a new command introduced to validate the XML profiles against an XSD schema through Fast DDS CLI.
That ensures the proper characterization of the entities using the xml profiles.

For more details, please refer to the [xml](../../fastddscli/cli/cli.md#cli-xml) section.

Install the [xmlschema](https://pypi.org/project/xmlschema/) dependency to be able to use this optional tool.

<a id="dependencies-sw"></a>

<a id="dependencies"></a>

### Dependencies

*eProsima Fast RTPS* has the following dependencies, when installed from sources in a Windows environment:

* [Asio and TinyXML2 libraries](#asiotinyxml2-sw)
* [OpenSSL](#openssl-sw)

#### SEE ALSO
For further information about this Fast DDS version dependencies, as well as for the corresponding versions of
other related products, please refer to the Fast DDS [Library dependencies](../../notes/versions.md#dependencies-compatibilities-library-dependencies)
section.

<a id="asiotinyxml2-sw"></a>

<a id="asio-and-tinyxml2-libraries"></a>

#### Asio and TinyXML2 libraries

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
They can be downloaded directly from the links below:

* [Asio](https://www.nuget.org/api/v2/package/asio-cppp-latest/1.24.0)
* [TinyXML2](https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg)

After downloading these packages, open an administrative shell with *PowerShell* and execute the following command:

```winbatch
choco install -y -s <PATH_TO_DOWNLOADS> asio-cppp-latest tinyxml2
```

where `<PATH_TO_DOWNLOADS>` is the folder into which the packages have been downloaded.

Also include the path to the `asio` and `tinyxml2` libraries in the `PATH` environment variable.

```powershell
Set-Item -Force -Path "env:PATH" -Value "C:\ProgramData\chocolatey\lib\asio-cppp-latest\build\native;C:\ProgramData\chocolatey\lib\tinyxml2;C:\ProgramData\chocolatey\lib\tinyxml2\lib;$env:PATH"
```

<a id="openssl-sw"></a>

<a id="openssl"></a>

#### OpenSSL

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install it by running the following command inside an administrative shell with *PowerShell*:

```winbatch
choco install -y openssl
```

<a id="libp11-sw"></a>

<a id="libp11-and-softhsm-libraries"></a>

#### Libp11 and SoftHSM libraries

Libp11 provides PKCS#11 support for OpenSSL.
This is an optional dependency, that is needed only when *eprosima Fast DDS* is used with security and PKCS#11 URIs.

Download the latest [libp11](https://github.com/OpenSC/libp11/) version for Windows from this
[repository](https://github.com/OpenSC/libp11)
and follow the [installation instructions](https://github.com/OpenSC/libp11/blob/master/INSTALL.md)

SoftHSM is a software implementation of an HSM (Hardware Security Module).
If *eProsima Fast DDS* tests are activated and *libp11* is installed
on the system, SoftHSM is additionally required to run tests of PKCS#11 features.

Download the [SoftHSM](https://www.opendnssec.org/softhsm/) for Windows installer from this
[repository](https://github.com/disig/SoftHSM2-for-Windows).
Execute the installer and follow the installation instructions.

OpenSSL access HSM and other hardware devices through its engine functionality.
In order to set up a new engine the OpenSSL configuration files must be updated specifying the
[libp11](https://github.com/OpenSC/libp11/) and hardware module (here [SoftHSM](https://www.opendnssec.org/softhsm/)) dynamic libraries location.

OpenSSL on Windows references its default configuration file through the OPENSSL_CONF
environment variable.
By default OpenSSL installs two identical default configuration files:

* C:\\Program Files\\OpenSSL-Win64\\bin\\cnf\\openssl.cnf mimics the Linux distributions one.
* C:\\Program Files\\OpenSSL-Win64\\bin\\openssl.cfg kept for backward compatibility.

Neither of them are loaded by default.
In order to direct OpenSSL to load one of them or any other we must set the variable:

```console
cmd> set OPENSSL_CONF=C:\Program Files\OpenSSL-Win64\bin\cnf\openssl.cnf
powershell> $Env:OPENSSL_CONF="C:\Program Files\OpenSSL-Win64\bin\cnf\openssl.cnf"
```

Once we have hinted OpenSSL the configuration file to use we must modify it to set up the
new PKCS#11 engine following the
[OpenSSL guidelines](https://www.openssl.org/docs/man1.1.1/man5/config.html#Engine-Configuration-Module)
replacing the binaries path with the proper ones.
For example, before any section in the configuration file we introduce:

```cfg
openssl_conf = openssl_init
```

at the end of the file we include the engine devoted sections.
Note to use POSIX path separator instead of the windows one.

```cfg
[openssl_init]
    engines = engine_section

[engine_section]
    pkcs11 = pkcs11_section

    [pkcs11_section]
    engine_id = pkcs11
    dynamic_path = C:/Program Files/libp11/src/pkcs11.dll
    MODULE_PATH = C:/Program Files (x86)/SoftHSM2/lib/softhsm2-x64.dll
    init = 0
```

A proper set up can be verified using OpenSSL command line tool:

```console
openssl engine pkcs11 -t
```

<a id="colcon-installation-windows"></a>

<a id="colcon-installation"></a>

### Colcon installation

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org) aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

#### IMPORTANT
Run colcon within a Visual Studio prompt.
To do so, launch a *Developer Command Prompt* from the search engine.

1. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:
   ```winbatch
   pip3 install -U colcon-common-extensions vcstool
   ```

   and add the path to the `vcs` executable to the `PATH` from the
   *Edit the system environment variables* control panel.

   #### NOTE
   If this fails due to an Environment Error, add the `--user` flag to the `pip3` installation command.
2. Create a `Fast-DDS` directory and download the repos file that will be used to install
   *eProsima Fast DDS* and its dependencies:
   ```winbatch
    mkdir ~\Fast-DDS
    cd ~\Fast-DDS
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastdds.repos -output fastdds.repos
    mkdir src
    vcs import src --input fastdds.repos
   ```

   Finally, use [colcon](https://colcon.readthedocs.io/en/released/) to compile all software:
   ```winbatch
   colcon build --packages-up-to fastdds
   ```

#### NOTE
Being based on [CMake](https://cmake.org), it is possible to pass the CMake configuration options to the `colcon build` command.
For more information on the specific syntax, please refer to the
[CMake specific arguments](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments)
page of the [colcon](https://colcon.readthedocs.io/en/released/) manual.

<a id="run-app-colcon-sw"></a>

<a id="run-an-application"></a>

#### Run an application

When running an instance of an application using *eProsima Fast DDS*, the colcon overlay built in the
dedicated `Fast-DDS` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:
  ```winbatch
  setup.bat
  ```
* Add the sourcing of the colcon overlay permanently, by opening the
  *Edit the system environment variables* control panel, and adding `~/Fast-DDS/install/setup.bat`
  to the `PATH`.

<a id="cmake-installation-windows"></a>

<a id="cmake-installation"></a>

### CMake installation

This section explains how to compile *eProsima Fast DDS* with [CMake](https://cmake.org), either [locally](#local-installation-sw) or
[globally](#global-installation-sw).

<a id="local-installation-sw"></a>

<a id="local-installation"></a>

#### Local installation

1. Open a command prompt, and create a `Fast-DDS` directory where to download and build *eProsima Fast DDS* and
   its dependencies:
   ```winbatch
   mkdir %USERPROFILE%\Fast-DDS
   ```
2. Clone the following dependencies and compile them using [CMake](https://cmake.org).
   * Fast DDS depends on [Foonathan memory](https://github.com/foonathan/memory).
     To ease the dependency management, *eProsima* provides a vendor package
     [Foonathan memory vendor](https://github.com/eProsima/foonathan_memory_vendor), which downloads and builds a
     specific revision of *Foonathan memory* if the library is not found in the system.
     ```winbatch
     cd %USERPROFILE%\Fast-DDS
     git clone https://github.com/eProsima/foonathan_memory_vendor.git
     cd foonathan_memory_vendor
     mkdir build && cd build
     cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
     cmake --build . --target install
     ```
   * [Fast CDR](https://github.com/eProsima/Fast-CDR.git)
     ```winbatch
     cd %USERPROFILE%\Fast-DDS
     git clone https://github.com/eProsima/Fast-CDR.git
     cd Fast-CDR
     mkdir build && cd build
     cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
     cmake --build . --target install
     ```
3. Once all dependencies are installed, install *eProsima Fast DDS*:
   ```winbatch
   cd %USERPROFILE%\Fast-DDS
   git clone https://github.com/eProsima/Fast-DDS.git
   cd Fast-DDS
   mkdir build && cd build
   cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
   cmake --build . --target install
   ```

<a id="global-installation-sw"></a>

<a id="global-installation"></a>

#### Global installation

To install *eProsima Fast DDS* system-wide instead of locally, remove the `CMAKE_INSTALL_PREFIX` flags that
appear in the configuration steps of `Fast-CDR` and `Fast-DDS`.

#### NOTE
By default, *eProsima Fast DDS* does not compile tests.
However, they can be activated by downloading and installing [Gtest](https://github.com/google/googletest).

<a id="run-app-cmake-sw"></a>

<a id="id3"></a>

#### Run an application

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed.
This can be done by opening the *Edit system environment variables* control panel and adding to the `PATH` the
*Fast DDS* and *Fast CDR* installation directories:

* *Fast DDS*: C:\\Program Files\\fastdds
* *Fast CDR*: C:\\Program Files\\fastcdr

<a id="fastdds-python-sw"></a>

<a id="fast-dds-python-bindings-installation"></a>

## Fast DDS Python bindings installation

This section provides the instructions for installing *Fast DDS Python bindings* in a Windows environment from sources.
*Fast DDS Python bindings* is an extension of *Fast DDS* which provides access to the Fast DDS API through Python.
Therefore, its installation is an extension of the installation of [Fast DDS](#fastdds-lib-sw).

*Fast DDS Python bindings* source code consists on several .i files which will be processed by [SWIG](#swig).
Then C++ files (for connecting C++ and Python) and Python files (Python module for Fast DDS) will be generated.

First of all, the [Requirements](#requirements-python-sw) and [Dependencies](#dependencies-python-sw) detailed below need to be met.
Afterwards, the user can choose whether to follow either the [colcon](#colcon-installation-python-windows) or the
[CMake](#cmake-installation-python-windows) installation instructions.

<a id="requirements-python-sw"></a>

<a id="id4"></a>

### Requirements

The installation of *Fast DDS Python bindings* in a Windows environment from sources requires the following tools to be
installed in the system:

- [Fast DDS requirements](#requirements-sw)
- [SWIG](#swig-python-sw)

<a id="swig-python-sw"></a>

<a id="swig"></a>

#### SWIG

[SWIG](#swig) is a development tool that allows connecting programs written in C/C++ with a variety of
other programming languages, among them Python.
SWIG version lower than 4.2 is required to build Fast DDS Python bindings.

#### NOTE
More recent [SWIG](#swig) releases are not yet supported.
Please, ensure to be using SWIG 4.0.

<a id="dependencies-python-sw"></a>

<a id="id5"></a>

### Dependencies

*Fast DDS Python bindings* has the following dependencies, when installed from sources in a Windows environment:

- [Fast DDS dependencies](#dependencies-sw)

<a id="colcon-installation-python-windows"></a>

<a id="id6"></a>

### Colcon installation

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org) aimed at building sets of software packages.
This section explains how to use it to compile *Fast DDS Python bindings* and its dependencies.

#### IMPORTANT
Run colcon within a Visual Studio prompt.
To do so, launch a *Developer Command Prompt* from the search engine.

1. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:
   ```winbatch
   pip3 install -U colcon-common-extensions vcstool
   ```

   and add the path to the `vcs` executable to the `PATH` from the
   *Edit the system environment variables* control panel.

   #### NOTE
   If this fails due to an Environment Error, add the `--user` flag to the `pip3` installation command.
2. Create a `Fast-DDS-python` directory and download the repos file that will be used to install
   *Fast DDS Python bindings* and its dependencies:
   ```winbatch
    mkdir ~\Fast-DDS-python
    cd ~\Fast-DDS-python
    wget https://raw.githubusercontent.com/eProsima/Fast-DDS-python/master/fastdds_python.repos
    mkdir src
    vcs import src --input fastdds_python.repos
   ```
3. Build the packages:
   ```winbatch
   colcon build --packages-up-to fastdds_python
   ```

#### NOTE
Being based on [CMake](https://cmake.org), it is possible to pass CMake configuration options to the `colcon build` command.
For more information on the specific syntax, please refer to the
[CMake specific arguments](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments)
page of the [colcon](https://colcon.readthedocs.io/en/released/) manual.

<a id="id8"></a>

#### Run an application

When running an instance of an application using *Fast DDS Python bindings*, the colcon overlay built in the
dedicated `Fast-DDS-python` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:
  ```winbatch
  setup.bat
  ```
* Add the sourcing of the colcon overlay permanently, by opening the
  *Edit the system environment variables* control panel, and adding `~/Fast-DDS/install/setup.bat`
  to the `PATH`.

<a id="cmake-installation-python-windows"></a>

<a id="id9"></a>

### CMake installation

This section explains how to compile *Fast DDS Python bindings* with [CMake](https://cmake.org), either
[locally](#local-installation-python-sw) or [globally](#global-installation-python-sw).

<a id="local-installation-python-sw"></a>

<a id="id10"></a>

#### Local installation

1. Open a command prompt, and create a `Fast-DDS-python` directory where to download and build
   *Fast DDS Python bindings* and its dependencies:
   ```winbatch
   mkdir %USERPROFILE%\Fast-DDS-python
   ```
2. Clone the following dependencies and compile them using [CMake](https://cmake.org).
   * Fast DDS depends on [Foonathan memory](https://github.com/foonathan/memory).
     To ease the dependency management, *eProsima* provides a vendor package
     [Foonathan memory vendor](https://github.com/eProsima/foonathan_memory_vendor), which downloads and builds a
     specific revision of *Foonathan memory* if the library is not found in the system.
     ```winbatch
     cd %USERPROFILE%\Fast-DDS-python
     git clone https://github.com/eProsima/foonathan_memory_vendor.git
     cd foonathan_memory_vendor
     mkdir build && cd build
     cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
     cmake --build . --target install
     ```
   * [Fast CDR](https://github.com/eProsima/Fast-CDR.git)
     ```winbatch
     cd %USERPROFILE%\Fast-DDS-python
     git clone https://github.com/eProsima/Fast-CDR.git
     cd Fast-CDR
     mkdir build && cd build
     cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
     cmake --build . --target install
     ```
   * [Fast DDS](https://github.com/eProsima/Fast-DDS.git)
     ```winbatch
     cd %USERPROFILE%\Fast-DDS-python
     git clone https://github.com/eProsima/Fast-DDS.git
     cd Fast-DDS
     mkdir build && cd build
     cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
     cmake --build . --target install
     ```
3. Once all dependencies are installed, install *Fast DDS Python bindings*:
   ```winbatch
   cd ~/Fast-DDS-python
   git clone https://github.com/eProsima/Fast-DDS-python.git
   cd Fast-DDS-python
   mkdir build && cd build
   cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
   cmake --build . --target install
   ```

<a id="global-installation-python-sw"></a>

<a id="id14"></a>

#### Global installation

To install *Fast DDS Python bindings* system-wide instead of locally, remove all the flags that
appear in the configuration steps of `Fast-CDR`, `Fast-DDS` and `Fast-DDS-python`, and change the
first in the configuration step of `foonathan_memory_vendor` to the following:

```winbatch
-DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
```

#### NOTE
Installation on system directories may need of permissions.
Maybe permissions have to be granted through `sudo`.

```winbatch
sudo cmake --build . --target install
```

<a id="id15"></a>

#### Run an application

When running an instance of an application using *Fast DDS Python bindings*, it must be linked with the library where
the packages have been installed.
This can be done by opening the *Edit system environment variables* control panel and adding to the `PATH` the
*Fast DDS python*, *Fast CDR* and *Fast DDS* installation directories:

* *Fast DDS python*: C:\\Program Files\\fastdds_python
* *Fast DDS*: C:\\Program Files\\fastdds
* *Fast CDR*: C:\\Program Files\\fastcdr

<a id="fastddsgen-sw"></a>

<a id="fast-dds-gen-installation"></a>

## Fast DDS-Gen installation

This section outlines the instructions for installing *Fast DDS-Gen* in a Windows environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
Please refer to [Introduction](../../fastddsgen/introduction/introduction.md#fastddsgen-intro) for more information, and to
[eProsima products compatibility](../../notes/versions.md#dependencies-compatibilities-product-compatibility) for the compatibility matrix against Fast DDS versions.

<a id="id16"></a>

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

<a id="java-sb"></a>

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
If Fast DDS has already been installed following [Colcon installation](#colcon-installation-windows), skip cloning *Fast DDS-Gen*’s
repository, as it can already be found under the `src` directory within the colcon workspace.

```winbatch
mkdir -p ~/Fast-DDS/src
cd ~/Fast-DDS/src
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen
cd fastddsgen
gradlew.bat assemble
```

#### NOTE
In case that a supported Gradle version is already installed in the system, *Fast DDS-Gen* can also be built running
directly:

```winbatch
gradle assemble
```

<a id="contents"></a>

#### Contents

The `Fast-DDS-Gen` folder contains the following packages:

* `share/fastddsgen`, where the generated Java application is.
* `scripts`, containing some user friendly scripts.

  #### NOTE
  To make these scripts accessible from any directory, add the `scripts` folder path to the
  `PATH` environment variable.

<!-- External links -->
