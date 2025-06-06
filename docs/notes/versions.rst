
.. include:: ../03-exports/roles.include

.. _dependencies_compatibilities:

Dependencies and compatibilities
================================

Fast DDS is continuously evolving and improving.
This means that the different software products that are part of the Fast DDS ecosystem are evolving and improving
together with Fast DDS.
This section provides information about the required dependencies for building Fast DDS, as well as about the versions
of the eProsima software products related to Fast DDS.

.. _currently_supported_versions:

Currently supported versions
----------------------------
The following table shows the currently supported versions of Fast DDS and their corresponding release and EOL dates.

.. list-table::
    :header-rows: 1

    * - Version
      - Version Branch
      - Latest Release
      - Release date
      - End of Standard Support date
      - EOL date
    * - 3.2
      - `3.2.x <https://github.com/eProsima/Fast-DDS/tree/3.2.x>`__ (LTS)
      - `v3.2.2 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.2.2>`__
      - March 2025
      - March 2026 (*)
      - March 2026 (*)
    * - 3.1
      - `3.1.x <https://github.com/eProsima/Fast-DDS/tree/3.1.x>`__
      - `v3.2.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.1.2>`__
      - October 2024
      - April 2025
      - April 2025
    * - 2.14
      - `2.14.x <https://github.com/eProsima/Fast-DDS/tree/2.14.x>`__
      - `v2.14.4 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.4>`__
      - March 2024
      - March 2025 (*)
      - March 2025 (*)
    * - 2.6 (maintenance)
      - `2.6.x <https://github.com/eProsima/Fast-DDS/tree/2.6.x>`__
      - `v2.6.10 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.6.10>`__
      - March 2022
      - July 2024
      - May 2025

(*): Support may be extended.

.. _dependencies_compatibilities_platform_support:

Platform support
----------------

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

.. _dependencies_compatibilities_build_system_dependencies:

Build system dependencies
-------------------------

The following table shows the minimum version required of the Fast DDS build system dependencies.

.. tab-set::

    .. tab-item:: 3.2.x
        :sync: 3.2.x

        .. list-table::

            * - **CMake**
              - 3.20

        .. list-table::
            :header-rows: 1

            * - OS \ Architecture
              - amd64
              - amd32
              - arm64
            * - Ubuntu Noble (24.04)
              - Tier 3: GCC 13.2
              - **───**
              - Tier 3: GCC 13.2
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
              - **───**
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 15
              - **───**
              - **───**
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) |br|
                Tier 3: MSVC v141 (Visual Studio 2017)
              - Tier 3: MSVC v142 (Visual Studio 2019) |br|
                Tier 3: MSVC v141 (Visual Studio 2017)
              - **───**
            * - Windows 11
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - **───**
            * - Debian Buster (10)
              - Tier 3: GCC 8
              - **───**
              - Tier 3: GCC 8
            * - Android 12
              - Tier 3: SDK 31
              - **───**
              - Tier 3: SDK 31
            * - Android 13
              - Tier 3: SDK 33
              - **───**
              - Tier 3: SDK 33
            * - QNX 7.1
              - Tier 3: QCC (over GCC 8.3)
              - **───**
              - Tier 3: QCC (over GCC 8.3)

    .. tab-item:: 3.1.x
        :sync: 3.1.x

        .. list-table::

            * - **CMake**
              - 3.20

        .. list-table::
            :header-rows: 1

            * - OS \ Architecture
              - amd64
              - amd32
              - arm64
            * - Ubuntu Noble (24.04)
              - Tier 3: GCC 13.2
              - **───**
              - Tier 3: GCC 13.2
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
              - **───**
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 15
              - **───**
              - **───**
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) |br|
                Tier 3: MSVC v141 (Visual Studio 2017)
              - Tier 3: MSVC v142 (Visual Studio 2019) |br|
                Tier 3: MSVC v141 (Visual Studio 2017)
              - **───**
            * - Windows 11
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - **───**
            * - Debian Buster (10)
              - Tier 3: GCC 8
              - **───**
              - Tier 3: GCC 8
            * - Android 12
              - Tier 3: SDK 31
              - **───**
              - Tier 3: SDK 31
            * - Android 13
              - Tier 3: SDK 33
              - **───**
              - Tier 3: SDK 33
            * - QNX 7.1
              - Tier 3: QCC (over GCC 8.3)
              - **───**
              - Tier 3: QCC (over GCC 8.3)

    .. tab-item:: 2.14.x
        :sync: 2.14.x

        .. list-table::

            * - **CMake**
              - 3.20

        .. list-table::
            :header-rows: 1

            * - OS \ Architecture
              - amd64
              - amd32
              - arm64
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
              - **───**
              - Tier 1: GCC 11.4 |br|
                Tier 3: Clang 15
            * - Ubuntu Focal (20.04)
              - Tier 3: GCC 9
              - **───**
              - Tier 3: GCC 9
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 15
              - **───**
              - **───**
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) |br|
                Tier 2: MSVC v141 (Visual Studio 2017)
              - Tier 2: MSVC v142 (Visual Studio 2019) |br|
                Tier 2: MSVC v141 (Visual Studio 2017)
              - **───**
            * - Windows 11
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - Tier 3: MSVC v143 (Visual Studio 2022)
              - **───**
            * - Debian Buster (10)
              - Tier 3: GCC 8
              - **───**
              - Tier 3: GCC 8
            * - Android 12
              - Tier 3: SDK 31
              - **───**
              - Tier 3: SDK 31
            * - Android 13
              - Tier 3: SDK 33
              - **───**
              - Tier 3: SDK 33
            * - QNX 7.1
              - Tier 3: QCC (over GCC 8.3)
              - **───**
              - Tier 3: QCC (over GCC 8.3)

    .. tab-item:: 2.6.x (maintenance)
        :sync: 2.6.x

        .. list-table::

            * - **CMake**
              - 3.16

        .. list-table::
            :header-rows: 1

            * - OS \ Architecture
              - amd64
              - amd32
              - arm64
            * - Ubuntu Focal (20.04)
              - Tier 1: GCC 9 |br|
                Tier 3: Clang 12
              - **───**
              - Tier 1: GCC 9 |br|
                Tier 3: Clang 12
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 12
              - **───**
              - **───**
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) |br|
                Tier 2: MSVC v141 (Visual Studio 2017)
              - Tier 2: MSVC v142 (Visual Studio 2019) |br|
                Tier 2: MSVC v141 (Visual Studio 2017)
              - **───**
            * - Debian Buster (10)
              - Tier 3: GCC 8
              - **───**
              - Tier 3: GCC 8

        .. important::
          According to our
          `release support guidelines <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_
          Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
          From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

.. _dependencies_compatibilities_library_dependencies:

Library dependencies
--------------------

The following table shows the corresponding versions of the Fast DDS library dependencies.

.. tab-set::

    .. tab-item:: 3.2.x
        :sync: 3.2.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v2.3.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.0>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.31.0 <https://github.com/chriskohlhoff/asio/tree/asio-1-31-0>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. tab-item:: 3.1.x
        :sync: 3.1.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v2.2.6 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.6>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.31.0 <https://github.com/chriskohlhoff/asio/tree/asio-1-31-0>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. tab-item:: 2.14.x
        :sync: 2.14.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v2.2.2 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.2>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. tab-item:: 2.6.x (maintenance)
        :sync: 2.6.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v1.0.28 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.28>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.2.2 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.2.2>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v1.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-1.1.1>`__

        .. important::
          According to our
          `release support guidelines <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_
          Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
          From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

.. _dependencies_compatibilities_product_compatibility:

eProsima products compatibility
-------------------------------

The following table shows the compatibility between the different versions of the eProsima software products that use
Fast DDS as the core middleware.

.. tab-set::

    .. tab-item:: 3.2.x
        :sync: 3.2.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v4.0.4 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.4>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v4.0.4 <https://github.com/eProsima/IDL-Parser/releases/tag/v4.0.4>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v2.2.0 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v2.2.0>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v3.2.2 <https://github.com/eProsima/ShapesDemo/releases/tag/v3.2.2>`__
            * - `Discovery Server <https://github.com/eProsima/Discovery-Server/>`__
              - `v2.0.0 <https://github.com/eProsima/Discovery-Server/releases/tag/v2.0.0>`__

    .. tab-item:: 3.1.x
        :sync: 3.1.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v4.0.3 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.3>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v4.0.3 <https://github.com/eProsima/IDL-Parser/releases/tag/v4.0.3>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v2.1.0 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v2.1.0>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v3.1.1 <https://github.com/eProsima/ShapesDemo/releases/tag/v3.1.1>`__
            * - `Discovery Server <https://github.com/eProsima/Discovery-Server/>`__
              - `v2.0.0 <https://github.com/eProsima/Discovery-Server/releases/tag/v2.0.0>`__

    .. tab-item:: 2.14.x
        :sync: 2.14.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v3.3.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.1>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v3.0.1 <https://github.com/eProsima/IDL-Parser/releases/tag/v3.0.1>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.4.3 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.4.3>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.14.4 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.14.4>`__
            * - `Discovery Server <https://github.com/eProsima/Discovery-Server/>`__
              - `v1.2.2 <https://github.com/eProsima/Discovery-Server/releases/tag/v1.2.2>`__

    .. tab-item:: 2.6.x (maintenance)
        :sync: 2.6.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v2.1.3 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.1.2>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v1.2.0 <https://github.com/eProsima/IDL-Parser/releases/tag/v1.2.0>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.0.2 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.0.2>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.6.10 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.6.10>`__
            * - `Discovery Server <https://github.com/eProsima/Discovery-Server/>`__
              - `v1.2.1 <https://github.com/eProsima/Discovery-Server/releases/tag/v1.2.1>`__

        .. important::
          According to our
          `release support guidelines <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_
          Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
          From now on, the v2.6 minor will only receive patches for critical issues and security fixes.
