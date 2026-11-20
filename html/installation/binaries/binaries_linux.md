<a id="linux-binaries"></a>

<a id="linux-installation-from-binaries"></a>

# Linux installation from binaries

The instructions for installing *eProsima Fast DDS* in a Linux environment from
binaries are provided in this page.

<a id="install-bl"></a>

<a id="install"></a>

## Install

The latest release of *eProsima Fast DDS* for Linux is available at the eProsima website
[Downloads tab](https://eprosima.com/index.php/downloads-all).
Once downloaded, install the package using apt.

```bash
sudo apt update && sudo apt install ./fastdds-3.6.2_amd64.deb
```

<a id="contents-bl"></a>

<a id="contents"></a>

### Contents

By default, the installation will download all the available packages, namely:

* `foonathan_memory_vendor`, an STL compatible C++ memory allocator
  [library](https://github.com/foonathan/memory).
* `fastcdr`, a C++ library that serializes according to the
  [standard CDR](https://www.omg.org/cgi-bin/doc?formal/02-06-51) serialization mechanism.
* `fastdds`, the core library of *eProsima Fast DDS* library.
* `fastddsgen`, a Java application that generates source code using the data types defined in an IDL file.

  #### NOTE
  To run this application, a Java Runtime Environment (JRE) must be installed in the system.
  The recommended version is 17 or higher.
  ```bash
  sudo apt install openjdk-17-jre
  ```

#### SEE ALSO
For further information about Fast DDS dependencies, as well as for the corresponding versions of other related
products, please refer to the Fast DDS [Dependencies and compatibilities](../../notes/versions.md#dependencies-compatibilities) section.

<a id="linking-bl"></a>

<a id="including-fast-dds-in-a-cmake-project"></a>

### Including Fast-DDS in a CMake project

The installer deploys *CMake config* files that simplify to incorporate **Fast-DDS** to any CMake project via
the *find_package* CMake API.

Shared and static libraries are provided by the installer. The user can select which one will be used in the CMake
project using next mechanisms.

1. Through CMake package components when calling find_package().
   ```cmake
   find_package(fastdds shared) # Load shared library target
   find_package(fastdds static) # Load static library target
   ```
2. Through the custom CMake variable fastdds_SHARED_LIBS.
   ```bash
   cmake -Dfastdds_SHARED_LIBS=ON .. # Load shared library target
   cmake -Dfastdds_SHARED_LIBS=OFF .. # Load static library target
   ```
3. Through the built-in CMake variable BUILD_SHARED_LIBS.
   ```bash
   cmake -DBUILD_SHARED_LIBS=ON .. # Load shared library target
   cmake -DBUILD_SHARED_LIBS=OFF .. # Load static library target
   ```
4. In case no previous mechanism is used, CMake will try to load static library target.
   If it fails then CMake will try to load shared library target.

For example in order to build the examples dynamically linked to **Fast-DDS** do:

```bash
cmake -Bbuildexample -DBUILD_SHARED_LIBS=ON .
cmake --build buildexample --target install
```

<a id="cli-bl"></a>

<a id="fast-dds-cli-optional"></a>

## Fast DDS CLI (optional)

The [Fast DDS CLI](../../fastddscli/cli/cli.md#ffastddscli-cli) (Command Line Interface) is a tool that provides a set commands and
sub-commands to perform, Fast DDS related, maintenance and configuration tasks.
As an optional tool, its dependencies are not installed by default, but they can be installed by running the
following command:

```bash
pip install psutil xmlschema
```

Python3 is required to run the CLI tool, and the [xmlschema](https://pypi.org/project/xmlschema/) dependency is
needed to use the [XML validation command](../../fastddscli/cli/cli.md#cli-xml).
