<a id="windows-binaries"></a>

<a id="windows-installation-from-binaries"></a>

# Windows installation from binaries

The instructions for installing *eProsima Fast DDS* in a Windows environment from
binaries are provided in this page.
It is organized as follows:

First of all, the [Requirements](#requirements-bw) detailed below need to be met.

<a id="requirements-bw"></a>

<a id="requirements"></a>

## Requirements

The installation of *eProsima Fast DDS* in a Windows environment from binaries requires the following tools to be
installed in the system:

* [Visual Studio](#visual-studio-bw)

<a id="visual-studio-bw"></a>

<a id="visual-studio"></a>

### Visual Studio

[Visual Studio](https://visualstudio.microsoft.com/) is required to
have a C++ compiler in the system. For this purpose, make sure to check the
`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to `Tools` -> `Get Tools and Features` and in the `Workloads` tab enable
`Desktop development with C++`. Finally, click `Modify` at the bottom right.

<a id="install-bw"></a>

<a id="install"></a>

## Install

The latest release of *eProsima Fast DDS* for Windows is available at the company website
[downloads page](https://eprosima.com/index.php/downloads-all).
Once downloaded, execute the installer and follow the instructions, choosing the preferred Visual Studio
version and architecture when prompted.

#### NOTE
By default, *eProsima Fast DDS* does not compile tests. To activate them, please refer to the
[Windows installation from sources](../sources/sources_windows.md#windows-sources) page.

<a id="contents-bw"></a>

<a id="contents"></a>

### Contents

By default, the installation will download all the available packages, namely:

* `foonathan_memory_vendor`, an STL compatible C++ memory allocator
  [library](https://github.com/foonathan/memory).
* `fastcdr`, a C++ library that serializes according to the
  [standard CDR](https://www.omg.org/cgi-bin/doc?formal/02-06-51) serialization mechanism.
* `fastdds`, the core library of *eProsima Fast DDS* library.
* `fastddsgen`, a Java application that generates source code using the data types defined in an IDL file.

#### SEE ALSO
For further information about Fast DDS dependencies, as well as for the corresponding versions of other related
products, please refer to the Fast DDS [Dependencies and compatibilities](../../notes/versions.md#dependencies-compatibilities) section.

<a id="env-vars-bw"></a>

<a id="environment-variables"></a>

### Environment variables

*eProsima Fast DDS* requires the following environment variable setup in order to function properly:

* `FASTDDSHOME`: Root folder where *eProsima Fast DDS* is installed.
* Additions to the `PATH`: The location of *eProsima Fast DDS* scripts and libraries should be
  appended to the `PATH`.

These variables are set automatically by checking the corresponding box during the installation process.

<a id="linking-bw"></a>

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
   ```winbatch
   cmake -Dfastdds_SHARED_LIBS=ON .. # Load shared library target
   cmake -Dfastdds_SHARED_LIBS=OFF .. # Load static library target
   ```
3. Through the built-in CMake variable BUILD_SHARED_LIBS.
   ```winbatch
   cmake -DBUILD_SHARED_LIBS=ON .. # Load shared library target
   cmake -DBUILD_SHARED_LIBS=OFF .. # Load static library target
   ```
4. In case no previous mechanism is used, CMake will try to load static library target.
   If it fails then CMake will try to load shared library target.

For example in order to build the examples dynamically linked to **Fast-DDS** do:

```winbatch
cmake -Bbuildexample -DBUILD_SHARED_LIBS=ON .
cmake --build buildexample --target install
```

<a id="cli-bw"></a>

<a id="fast-dds-cli-optional"></a>

## Fast DDS CLI (optional)

The [Fast DDS CLI](../../fastddscli/cli/cli.md#ffastddscli-cli) (Command Line Interface) is a tool that provides a set commands and
sub-commands to perform, Fast DDS related, maintenance and configuration tasks.
As an optional tool, its dependencies are not installed by default, but they can be installed by running the
following command:

```winbatch
choco install python
python -m pip install --upgrade psutil pywin32 xmlschema
```

Python3 is required to run the CLI tool, and the [xmlschema](https://pypi.org/project/xmlschema/) dependency is
needed to use the [XML validation command](../../fastddscli/cli/cli.md#cli-xml).
