<a id="dependencies-compatibilities"></a>

<a id="dependencies-and-compatibilities"></a>

# Dependencies and compatibilities

Fast DDS is continuously evolving and improving.
This means that the different software products that are part of the Fast DDS ecosystem are evolving and improving
together with Fast DDS.
This section provides information about the required dependencies for building Fast DDS, as well as about the versions
of the eProsima software products related to Fast DDS.

<a id="currently-supported-versions"></a>

<a id="id1"></a>

## Currently supported versions

The following table shows the currently supported versions of Fast DDS and their corresponding release and EOL dates.

| Version            | Version Branch                                                 | Latest Release                                                       | Release date   | End of Standard Support date   | EOL date             |
|--------------------|----------------------------------------------------------------|----------------------------------------------------------------------|----------------|--------------------------------|----------------------|
| 3.7 **[Pro]**      | `3.7.x` **[Pro]**                                              | `v3.7.0` **[Pro]**                                                   | August 2026    | Defined per customer           | Defined per customer |
| 3.6                | [3.6.x](https://github.com/eProsima/Fast-DDS/tree/3.6.x)       | [v3.6.2](https://github.com/eProsima/Fast-DDS/releases/tag/v3.6.2)   | April 2026     | March 2028 (\*)                | May 2031 (\*)        |
| 3.2                | [3.2.x](https://github.com/eProsima/Fast-DDS/tree/3.2.x) (LTS) | [v3.2.5](https://github.com/eProsima/Fast-DDS/releases/tag/v3.2.5)   | March 2025     | March 2026 (\*)                | December 2026 (\*)   |
| 2.14 (maintenance) | [2.14.x](https://github.com/eProsima/Fast-DDS/tree/2.14.x)     | [v2.14.7](https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.7) | March 2024     | September 2026                 | May 2029 (\*)        |
| 2.6 (maintenance)  | [2.6.x](https://github.com/eProsima/Fast-DDS/tree/2.6.x)       | [v2.6.12](https://github.com/eProsima/Fast-DDS/releases/tag/v2.6.12) | March 2022     | July 2024                      | May 2027             |

(\*): Support may be extended.

<a id="dependencies-compatibilities-platform-support"></a>

<a id="platform-support"></a>

## Platform support

This following definitions reflects the level of support offered by **eprosima Fast DDS** on different platforms:

* **Tier 1**: these platforms are subjected to our unit test suite and other testing tools on a frequent basis including
  continuous integration jobs, nightly jobs, packaging jobs, and performance testing.
  Errors or bugs discovered in these platforms are prioritized for correction by the development team.
  Significant errors discovered in Tier 1 platforms can impact release dates and we strive to resolve all known high
  priority errors in Tier 1 platforms prior to new version releases.
* **Tier 2**: these platforms are subject to periodic CI testing which runs both builds and tests with publicly
  accessible results.
  The CI is expected to be run at least within a week of relevant changes for the current release of *Fast DDS*.
  Installation instructions should be available and up-to-date in order for a platform to be listed in this category.
  Package-level binary packages may not be provided but providing a downloadable archive of the built workspace is
  encouraged.
  Errors may be present in released product versions for Tier 2 platforms.
  Known errors in Tier 2 platforms will be addressed subject to resource availability on a best effort basis and may or
  may not be corrected prior to new version releases.
  One or more entities should be committed to continuing support of the platform.
* **Tier 3**: these platforms are those for which community reports indicate that the release is functional.
  The development team does not run the unit test suite or perform any other tests on platforms in Tier 3.
  Community members may provide assistance with these platforms.

<a id="dependencies-compatibilities-build-system-dependencies"></a>

<a id="build-system-dependencies"></a>

## Build system dependencies

The following table shows the minimum version required of the Fast DDS build system dependencies.

### 3.7.x **[Pro]**

| **CMake**   | 3.20   |
|-------------|--------|

| OS Architecture      | amd64                                  | amd32                                  | arm64                      |
|----------------------|----------------------------------------|----------------------------------------|----------------------------|
| Ubuntu Noble (24.04) | Tier 3: GCC 11.4                       | **───**                                | Tier 3: GCC 11.4           |
| Ubuntu Jammy (22.04) | Tier 1: GCC 11.4                       | **───**                                | Tier 1: GCC 11.4           |
| MacOS Sequoia (15)   | Tier 1: Clang 16                       | **───**                                | **───**                    |
| Windows 10           | Tier 1: MSVC v142 (Visual Studio 2019) | Tier 3: MSVC v142 (Visual Studio 2019) | **───**                    |
| Windows 11           | Tier 1: MSVC v142 (Visual Studio 2019) | Tier 3: MSVC v142 (Visual Studio 2019) | **───**                    |
| Debian Buster (10)   | Tier 3: GCC 8                          | **───**                                | Tier 3: GCC 8              |
| Android 12           | Tier 3: SDK 31                         | **───**                                | Tier 3: SDK 31             |
| Android 13           | Tier 3: SDK 33                         | **───**                                | Tier 3: SDK 33             |
| QNX 7.1              | Tier 3: QCC (over GCC 8.3)             | **───**                                | Tier 3: QCC (over GCC 8.3) |

### 3.6.x

| **CMake**   | 3.28   |
|-------------|--------|

| OS Architecture      | amd64                                                                               | amd32                                                                               | arm64                                       |
|----------------------|-------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------|---------------------------------------------|
| Ubuntu Noble (24.04) | Tier 1: GCC 13.2                                                                    | **───**                                                                             | Tier 3: GCC 13.2                            |
| Ubuntu Jammy (22.04) | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15                                         | **───**                                                                             | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15 |
| MacOS Sequoia (15)   | Tier 1: Clang 16                                                                    | **───**                                                                             | **───**                                     |
| Windows 10           | Tier 3: MSVC v142 (Visual Studio 2019) and <br/><br/>MSVC v141 (Visual Studio 2017) | Tier 3: MSVC v142 (Visual Studio 2019) and <br/><br/>MSVC v141 (Visual Studio 2017) | **───**                                     |
| Windows 11           | Tier 1: MSVC v143 (Visual Studio 2022)                                              | Tier 1: MSVC v143 (Visual Studio 2022)                                              | **───**                                     |
| Debian Buster (10)   | Tier 3: GCC 8                                                                       | **───**                                                                             | Tier 3: GCC 8                               |
| Android 12           | Tier 3: SDK 31                                                                      | **───**                                                                             | Tier 3: SDK 31                              |
| Android 13           | Tier 3: SDK 33                                                                      | **───**                                                                             | Tier 3: SDK 33                              |
| QNX 7.1              | Tier 3: QCC (over GCC 8.3)                                                          | **───**                                                                             | Tier 3: QCC (over GCC 8.3)                  |

### 3.2.x

| **CMake**   | 3.20   |
|-------------|--------|

| OS Architecture      | amd64                                                                                   | amd32                                                                                   | arm64                                       |
|----------------------|-----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|---------------------------------------------|
| Ubuntu Noble (24.04) | Tier 3: GCC 13.2                                                                        | **───**                                                                                 | Tier 3: GCC 13.2                            |
| Ubuntu Jammy (22.04) | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15                                             | **───**                                                                                 | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15 |
| MacOS Sequoia (15)   | Tier 1: Clang 16                                                                        | **───**                                                                                 | **───**                                     |
| Windows 10           | Tier 1: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 3: MSVC v141 (Visual Studio 2017) | Tier 3: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 3: MSVC v141 (Visual Studio 2017) | **───**                                     |
| Windows 11           | Tier 1: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 1: MSVC v143 (Visual Studio 2022) | Tier 3: MSVC v143 (Visual Studio 2022)                                                  | **───**                                     |
| Debian Buster (10)   | Tier 3: GCC 8                                                                           | **───**                                                                                 | Tier 3: GCC 8                               |
| Android 12           | Tier 3: SDK 31                                                                          | **───**                                                                                 | Tier 3: SDK 31                              |
| Android 13           | Tier 3: SDK 33                                                                          | **───**                                                                                 | Tier 3: SDK 33                              |
| QNX 7.1              | Tier 3: QCC (over GCC 8.3)                                                              | **───**                                                                                 | Tier 3: QCC (over GCC 8.3)                  |

### 2.14.x (maintenance)

| **CMake**   | 3.20   |
|-------------|--------|

| OS Architecture      | amd64                                                                                   | amd32                                                                                   | arm64                                       |
|----------------------|-----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|---------------------------------------------|
| Ubuntu Jammy (22.04) | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15                                             | **───**                                                                                 | Tier 1: GCC 11.4 <br/><br/>Tier 3: Clang 15 |
| Ubuntu Focal (20.04) | Tier 3: GCC 9                                                                           | **───**                                                                                 | Tier 3: GCC 9                               |
| MacOS Sequoia (15)   | Tier 1: Clang 16                                                                        | **───**                                                                                 | **───**                                     |
| Windows 10           | Tier 1: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 2: MSVC v141 (Visual Studio 2017) | Tier 2: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 2: MSVC v141 (Visual Studio 2017) | **───**                                     |
| Windows 11           | Tier 3: MSVC v143 (Visual Studio 2022)                                                  | Tier 3: MSVC v143 (Visual Studio 2022)                                                  | **───**                                     |
| Debian Buster (10)   | Tier 3: GCC 8                                                                           | **───**                                                                                 | Tier 3: GCC 8                               |
| Android 12           | Tier 3: SDK 31                                                                          | **───**                                                                                 | Tier 3: SDK 31                              |
| Android 13           | Tier 3: SDK 33                                                                          | **───**                                                                                 | Tier 3: SDK 33                              |
| QNX 7.1              | Tier 3: QCC (over GCC 8.3)                                                              | **───**                                                                                 | Tier 3: QCC (over GCC 8.3)                  |

### 2.6.x (maintenance)

| **CMake**   | 3.16   |
|-------------|--------|

| OS Architecture      | amd64                                                                                   | amd32                                                                                   | arm64                                    |
|----------------------|-----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|------------------------------------------|
| Ubuntu Focal (20.04) | Tier 1: GCC 9 <br/><br/>Tier 3: Clang 12                                                | **───**                                                                                 | Tier 1: GCC 9 <br/><br/>Tier 3: Clang 12 |
| MacOS Sequoia (15)   | Tier 1: Clang 16                                                                        | **───**                                                                                 | **───**                                  |
| Windows 10           | Tier 1: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 2: MSVC v141 (Visual Studio 2017) | Tier 2: MSVC v142 (Visual Studio 2019) <br/><br/>Tier 2: MSVC v141 (Visual Studio 2017) | **───**                                  |
| Debian Buster (10)   | Tier 3: GCC 8                                                                           | **───**                                                                                 | Tier 3: GCC 8                            |

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

<a id="dependencies-compatibilities-library-dependencies"></a>

<a id="library-dependencies"></a>

## Library dependencies

The following table shows the corresponding versions of the Fast DDS library dependencies.

### 3.7.x **[Pro]**

| Product                                                                         | Related version                                                                                                                 |
|---------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|
| [Fast CDR](https://github.com/eProsima/Fast-CDR/)                               | [v2.4.0](https://github.com/eProsima/Fast-CDR/releases/tag/v2.4.0)                                                              |
| [Foonathan Memory Vendor](https://github.com/eProsima/foonathan_memory_vendor/) | [v1.4.1](https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.4.1)                                               |
| [Asio](https://github.com/chriskohlhoff/asio)                                   | [v1.34.2](https://github.com/chriskohlhoff/asio/tree/asio-1-34-2)                                                               |
| [TinyXML2](https://github.com/leethomason/tinyxml2)                             | [v6.0.0](https://github.com/leethomason/tinyxml2/tree/6.0.0)                                                                    |
| [OpenSSL](https://github.com/openssl/openssl)                                   | [v3.3.6 (Windows)](https://github.com/openssl/openssl/releases/tag/openssl-3.3.6), default installation in Ubuntu distributions |

### 3.6.x

| Product                                                                         | Related version                                                                                                                 |
|---------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|
| [Fast CDR](https://github.com/eProsima/Fast-CDR/)                               | [v2.3.6](https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.6)                                                              |
| [Foonathan Memory Vendor](https://github.com/eProsima/foonathan_memory_vendor/) | [v1.4.1](https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.4.1)                                               |
| [Asio](https://github.com/chriskohlhoff/asio)                                   | [v1.34.2](https://github.com/chriskohlhoff/asio/tree/asio-1-34-2)                                                               |
| [TinyXML2](https://github.com/leethomason/tinyxml2)                             | [v6.0.0](https://github.com/leethomason/tinyxml2/tree/6.0.0)                                                                    |
| [OpenSSL](https://github.com/openssl/openssl)                                   | [v3.3.6 (Windows)](https://github.com/openssl/openssl/releases/tag/openssl-3.3.6), default installation in Ubuntu distributions |

### 3.2.x

| Product                                                                         | Related version                                                                   |
|---------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [Fast CDR](https://github.com/eProsima/Fast-CDR/)                               | [v2.3.2](https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.2)                |
| [Foonathan Memory Vendor](https://github.com/eProsima/foonathan_memory_vendor/) | [v1.3.1](https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1) |
| [Asio](https://github.com/chriskohlhoff/asio)                                   | [v1.31.0](https://github.com/chriskohlhoff/asio/tree/asio-1-31-0)                 |
| [TinyXML2](https://github.com/leethomason/tinyxml2)                             | [v6.0.0](https://github.com/leethomason/tinyxml2/tree/6.0.0)                      |
| [OpenSSL](https://github.com/openssl/openssl)                                   | [v3.1.1](https://github.com/openssl/openssl/releases/tag/openssl-3.1.1)           |

### 2.14.x (maintenance)

| Product                                                                         | Related version                                                                   |
|---------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [Fast CDR](https://github.com/eProsima/Fast-CDR/)                               | [v2.2.8](https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.8)                |
| [Foonathan Memory Vendor](https://github.com/eProsima/foonathan_memory_vendor/) | [v1.3.1](https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1) |
| [Asio](https://github.com/chriskohlhoff/asio)                                   | [v1.18.1](https://github.com/chriskohlhoff/asio/tree/asio-1-18-1)                 |
| [TinyXML2](https://github.com/leethomason/tinyxml2)                             | [v6.0.0](https://github.com/leethomason/tinyxml2/tree/6.0.0)                      |
| [OpenSSL](https://github.com/openssl/openssl)                                   | [v3.1.1](https://github.com/openssl/openssl/releases/tag/openssl-3.1.1)           |

### 2.6.x (maintenance)

| Product                                                                         | Related version                                                                   |
|---------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [Fast CDR](https://github.com/eProsima/Fast-CDR/)                               | [v1.0.29](https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.29)              |
| [Foonathan Memory Vendor](https://github.com/eProsima/foonathan_memory_vendor/) | [v1.2.1](https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.2.1) |
| [Asio](https://github.com/chriskohlhoff/asio)                                   | [v1.18.1](https://github.com/chriskohlhoff/asio/tree/asio-1-18-1)                 |
| [TinyXML2](https://github.com/leethomason/tinyxml2)                             | [v6.0.0](https://github.com/leethomason/tinyxml2/tree/6.0.0)                      |
| [OpenSSL](https://github.com/openssl/openssl)                                   | [v1.1.1](https://github.com/openssl/openssl/releases/tag/openssl-1.1.1)           |

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

<a id="dependencies-compatibilities-product-compatibility"></a>

<a id="eprosima-products-compatibility"></a>

## eProsima products compatibility

The following table shows the compatibility between the different versions of the eProsima software products that use
Fast DDS as the core middleware.

### 3.7.x **[Pro]**

| Product                                                               | Related version                                                             |
|-----------------------------------------------------------------------|-----------------------------------------------------------------------------|
| Fast DDS Gen **[Pro]**                                                | v4.4.0                                                                      |
| [Fast DDS Gen - IDL parser](https://github.com/eProsima/IDL-Parser/)  | [v4.4.0](https://github.com/eProsima/IDL-Parser/releases/tag/v4.4.0)        |
| Fast DDS Python **[Pro]**                                             | v2.7.0                                                                      |
| [Shapes Demo](https://github.com/eProsima/ShapesDemo/)                | [v3.6.2](https://github.com/eProsima/ShapesDemo/releases/tag/v3.6.2)        |
| [DDS Monitor](https://github.com/eProsima/DDS-Monitor/)               | [v4.2.0](https://github.com/eProsima/DDS-Monitor/releases/tag/v4.2.0)       |
| [Fast DDS Spy](https://github.com/eProsima/Fast-DDS-spy/)             | [v1.5.1](https://github.com/eProsima/Fast-DDS-spy/releases/tag/v1.5.1)      |
| [DDS Record & Replay](https://github.com/eProsima/DDS-Record-Replay/) | [v1.5.1](https://github.com/eProsima/DDS-Record-Replay/releases/tag/v1.5.1) |
| [DDS Router](https://github.com/eProsima/DDS-Router/)                 | [v3.5.1](https://github.com/eProsima/DDS-Router/releases/tag/v3.5.1)        |
| [DDS Enabler](https://github.com/eProsima/DDS-Enabler/)               | [v1.2.0](https://github.com/eProsima/DDS-Enabler/releases/tag/v1.2.0)       |

### 3.6.x

| Product                                                               | Related version                                                             |
|-----------------------------------------------------------------------|-----------------------------------------------------------------------------|
| [Fast DDS Gen](https://github.com/eProsima/Fast-DDS-Gen/)             | [v4.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.3.0)      |
| [Fast DDS Gen - IDL parser](https://github.com/eProsima/IDL-Parser/)  | [v4.3.0](https://github.com/eProsima/IDL-Parser/releases/tag/v4.3.0)        |
| [Fast DDS Python](https://github.com/eProsima/Fast-DDS-python/)       | [v2.6.2](https://github.com/eProsima/Fast-DDS-python/releases/tag/v2.6.2)   |
| [Shapes Demo](https://github.com/eProsima/ShapesDemo/)                | [v3.6.2](https://github.com/eProsima/ShapesDemo/releases/tag/v3.6.2)        |
| [DDS Monitor](https://github.com/eProsima/DDS-Monitor/)               | [v4.2.0](https://github.com/eProsima/DDS-Monitor/releases/tag/v4.2.0)       |
| [Fast DDS Spy](https://github.com/eProsima/Fast-DDS-spy/)             | [v1.5.1](https://github.com/eProsima/Fast-DDS-spy/releases/tag/v1.5.1)      |
| [DDS Record & Replay](https://github.com/eProsima/DDS-Record-Replay/) | [v1.5.1](https://github.com/eProsima/DDS-Record-Replay/releases/tag/v1.5.1) |
| [DDS Router](https://github.com/eProsima/DDS-Router/)                 | [v3.5.1](https://github.com/eProsima/DDS-Router/releases/tag/v3.5.1)        |
| [DDS Enabler](https://github.com/eProsima/DDS-Enabler/)               | [v1.2.0](https://github.com/eProsima/DDS-Enabler/releases/tag/v1.2.0)       |

### 3.2.x

| Product                                                               | Related version                                                             |
|-----------------------------------------------------------------------|-----------------------------------------------------------------------------|
| [Fast DDS Gen](https://github.com/eProsima/Fast-DDS-Gen/)             | [v4.0.7](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.7)      |
| [Fast DDS Gen - IDL parser](https://github.com/eProsima/IDL-Parser/)  | [v4.0.5](https://github.com/eProsima/IDL-Parser/releases/tag/v4.0.5)        |
| [Fast DDS Python](https://github.com/eProsima/Fast-DDS-python/)       | [v2.2.2](https://github.com/eProsima/Fast-DDS-python/releases/tag/v2.2.2)   |
| [Shapes Demo](https://github.com/eProsima/ShapesDemo/)                | [v3.2.5](https://github.com/eProsima/ShapesDemo/releases/tag/v3.2.5)        |
| [Discovery Server](https://github.com/eProsima/Discovery-Server/)     | [v2.0.3](https://github.com/eProsima/Discovery-Server/releases/tag/v2.0.3)  |
| [DDS Monitor](https://github.com/eProsima/DDS-Monitor/)               | [v3.2.0](https://github.com/eProsima/DDS-Monitor/releases/tag/v3.2.0)       |
| [Fast DDS Spy](https://github.com/eProsima/Fast-DDS-spy/)             | [v1.2.0](https://github.com/eProsima/Fast-DDS-spy/releases/tag/v1.2.0)      |
| [DDS Record & Replay](https://github.com/eProsima/DDS-Record-Replay/) | [v1.2.0](https://github.com/eProsima/DDS-Record-Replay/releases/tag/v1.2.0) |
| [DDS Router](https://github.com/eProsima/DDS-Router/)                 | [v3.2.0](https://github.com/eProsima/DDS-Router/releases/tag/v3.2.0)        |

### 2.14.x (maintenance)

| Product                                                              | Related version                                                            |
|----------------------------------------------------------------------|----------------------------------------------------------------------------|
| [Fast DDS Gen](https://github.com/eProsima/Fast-DDS-Gen/)            | [v3.3.3](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.3)     |
| [Fast DDS Gen - IDL parser](https://github.com/eProsima/IDL-Parser/) | [v3.0.2](https://github.com/eProsima/IDL-Parser/releases/tag/v3.0.2)       |
| [Fast DDS Python](https://github.com/eProsima/Fast-DDS-python/)      | [v1.4.5](https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.4.5)  |
| [Shapes Demo](https://github.com/eProsima/ShapesDemo/)               | [v2.14.8](https://github.com/eProsima/ShapesDemo/releases/tag/v2.14.8)     |
| [Discovery Server](https://github.com/eProsima/Discovery-Server/)    | [v1.2.3](https://github.com/eProsima/Discovery-Server/releases/tag/v1.2.3) |
| [DDS Monitor](https://github.com/eProsima/DDS-Monitor/)              | [v2.1.0](https://github.com/eProsima/DDS-Monitor/releases/tag/v2.1.0)      |
| [DDS Router](https://github.com/eProsima/DDS-Router/)                | [v2.2.0](https://github.com/eProsima/DDS-Router/releases/tag/v2.2.0)       |

### 2.6.x (maintenance)

| Product                                                              | Related version                                                            |
|----------------------------------------------------------------------|----------------------------------------------------------------------------|
| [Fast DDS Gen](https://github.com/eProsima/Fast-DDS-Gen/)            | [v2.1.3](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.1.3)     |
| [Fast DDS Gen - IDL parser](https://github.com/eProsima/IDL-Parser/) | [v1.2.0](https://github.com/eProsima/IDL-Parser/releases/tag/v1.2.0)       |
| [Fast DDS Python](https://github.com/eProsima/Fast-DDS-python/)      | [v1.0.3](https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.0.3)  |
| [Shapes Demo](https://github.com/eProsima/ShapesDemo/)               | [v2.6.12](https://github.com/eProsima/ShapesDemo/releases/tag/v2.6.12)     |
| [Discovery Server](https://github.com/eProsima/Discovery-Server/)    | [v1.2.1](https://github.com/eProsima/Discovery-Server/releases/tag/v1.2.1) |

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
From now on, the v2.6 minor will only receive patches for critical issues and security fixes.
