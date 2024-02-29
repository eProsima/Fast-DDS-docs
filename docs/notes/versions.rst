.. _dependencies_compatibilities:

Dependencies and compatibilities
================================

Fast DDS is continuously evolving and improving.
This means that the different software products that are part of the Fast DDS ecosystem are evolving and improving
together with Fast DDS.
This section provides information about the required dependencies for building Fast DDS, and versions of the eProsima
software products related to the Fast DDS.

.. _dependencies_compatibilities_build_system_dependencies:

Build system dependencies
-------------------------

The following table shows the minimum version required of the Fast DDS build system dependencies.

.. tabs::

    .. group-tab:: 2.13.x

        .. list-table::

            * - **CMake**
              - 3.20

        .. list-table::
            :header-rows: 1

            * - Platform
              - Compiler amd64
              - Compiler amd32
              - Compiler arm64
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 11.4 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 15
              - ──────────
              - Tier 1: GCC 11.4 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 15
            * - Ubuntu Focal (20.04)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 9
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 9
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 15 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
              - ──────────
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) :raw-html:`<br />`
                Tier 2: MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: MSVC v142 (Visual Studio 2019),
                :raw-html:`<br />&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp`
                MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
            * - Windows 11
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: MSVC v143 (Visual Studio 2022)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: MSVC v143 (Visual Studio 2022)
              - ──────────
            * - Debian Buster (10)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
            * - Android 12
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 31
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 31
            * - Android 13
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 33
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 33
            * - QNX 7.1
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)

        For further information regarding **tiers**, please check
        `the GitHub platform support section <https://github.com/eProsima/Fast-DDS/blob/2.13.x/PLATFORM_SUPPORT.md>`__.

    .. group-tab:: 2.12.x

        .. list-table::

            * - **CMake**
              - 3.22

        .. list-table::
            :header-rows: 1

            * - Platform
              - Compiler amd64
              - Compiler amd32
              - Compiler arm64
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 11.4 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 15
              - ──────────
              - Tier 1: GCC 11.4 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 15
            * - Ubuntu Focal (20.04)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 9
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 9
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 15 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
              - ──────────
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) :raw-html:`<br />`
                Tier 2: MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: MSVC v142 (Visual Studio 2019),
                :raw-html:`<br />&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp`
                MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
            * - Debian Buster (10)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
            * - Android 12
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 31
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 31
            * - QNX 7.1
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)

        For further information regarding **tiers**, please check
        `the GitHub platform support section <https://github.com/eProsima/Fast-DDS/blob/2.12.x/PLATFORM_SUPPORT.md>`__.

    .. group-tab:: 2.10.x

        .. list-table::

            * - **CMake**
              - 3.16

        .. list-table::
            :header-rows: 1

            * - Platform
              - Compiler amd64
              - Compiler amd32
              - Compiler arm64
            * - Ubuntu Jammy (22.04)
              - Tier 1: GCC 9, GCC 11.3, GCC 12.1 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
              - ──────────
              - Tier 1: GCC 9, GCC 11.3, GCC 12.1 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
            * - Ubuntu Focal (20.04)
              - Tier 1: GCC 9, GCC 11.3, GCC 12.1 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
              - ──────────
              - Tier 1: GCC 9, GCC 11.3, GCC 12.1 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 12 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
              - ──────────
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) :raw-html:`<br />`
                Tier 2: MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: MSVC v142 (Visual Studio 2019),
                :raw-html:`<br />&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp`
                MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
            * - Debian Buster (10)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
            * - Android 11
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 30
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: SDK 30
            * - QNX 7.1
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: QCC (over GCC 8.3)

        For further information regarding **tiers**, please check
        `the GitHub platform support section <https://github.com/eProsima/Fast-DDS/blob/2.10.x/PLATFORM_SUPPORT.md>`__.

    .. group-tab:: 2.6.x

        .. list-table::

            * - **CMake**
              - 3.16

        .. list-table::
            :header-rows: 1

            * - Platform
              - Compiler amd64
              - Compiler amd32
              - Compiler arm64
            * - Ubuntu Focal (20.04)
              - Tier 1: GCC 9 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
              - ──────────
              - Tier 1: GCC 9 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: Clang 12
            * - MacOS Mojave (10.14)
              - Tier 1: Clang 12 :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
              - ──────────
            * - Windows 10
              - Tier 1: MSVC v142 (Visual Studio 2019) :raw-html:`<br />`
                Tier 2: MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: MSVC v142 (Visual Studio 2019),
                :raw-html:`<br />&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp`
                MSVC v141 (Visual Studio 2017) :raw-html:`<br />`
                Tier 3: ─────
              - ──────────
            * - Debian Buster (10)
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8
              - ──────────
              - Tier 1: ───── :raw-html:`<br />`
                Tier 2: ───── :raw-html:`<br />`
                Tier 3: GCC 8

        For further information regarding **tiers**, please check
        `the GitHub platform support section <https://github.com/eProsima/Fast-DDS/blob/2.6.x/PLATFORM_SUPPORT.md>`__.

.. _dependencies_compatibilities_library_dependencies:

Library dependencies
--------------------

The following table shows the corresponding versions of the Fast DDS library dependencies.

.. tabs::

    .. group-tab:: 2.13.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v2.1.3 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.3>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. group-tab:: 2.12.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v2.1.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.0>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. group-tab:: 2.10.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v1.0.27 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.27>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v3.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-3.1.1>`__

    .. group-tab:: 2.6.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast CDR <https://github.com/eProsima/Fast-CDR/>`__
              - `v1.0.24 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.24>`__
            * - `Foonathan Memory Vendor <https://github.com/eProsima/foonathan_memory_vendor/>`__
              - `v1.2.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.2.1>`__
            * - `Asio <https://github.com/chriskohlhoff/asio>`__
              - `v1.18.1 <https://github.com/chriskohlhoff/asio/tree/asio-1-18-1>`__
            * - `TinyXML2 <https://github.com/leethomason/tinyxml2>`__
              - `v6.0.0 <https://github.com/leethomason/tinyxml2/tree/6.0.0>`__
            * - `OpenSSL <https://github.com/openssl/openssl>`__
              - `v1.1.1 <https://github.com/openssl/openssl/releases/tag/openssl-1.1.1>`__

.. _dependencies_compatibilities_product_compatibility:

eProsima products compatibility
-------------------------------

The following table shows the compatibility between the different versions of the eProsima software products that use
Fast DDS as the core middleware.

.. tabs::

    .. group-tab:: 2.13.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v3.2.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v3.0.0 <https://github.com/eProsima/IDL-Parser/releases/tag/v3.0.0>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.4.0 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.4.0>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.13.3 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.13.3>`__

    .. group-tab:: 2.12.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v3.1.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.1.0>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v2.0.0 <https://github.com/eProsima/IDL-Parser/releases/tag/v2.0.0>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.3.1 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.3.1>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.12.1 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.12.1>`__

    .. group-tab:: 2.10.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v2.4.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.4.0>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v1.5.0 <https://github.com/eProsima/IDL-Parser/releases/tag/v1.5.0>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.2.1 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.2.1>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.10.3 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.10.3>`__

    .. group-tab:: 2.6.x

        .. list-table::
            :header-rows: 1

            * - Product
              - Related version
            * - `Fast DDS Gen <https://github.com/eProsima/Fast-DDS-Gen/>`__
              - `v2.1.2 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.1.2>`__
            * - `Fast DDS Gen - IDL parser <https://github.com/eProsima/IDL-Parser/>`__
              - `v1.2.0 <https://github.com/eProsima/IDL-Parser/releases/tag/v1.2.0>`__
            * - `Fast DDS python <https://github.com/eProsima/Fast-DDS-python/>`__
              - `v1.0.2 <https://github.com/eProsima/Fast-DDS-python/releases/tag/v1.0.2>`__
            * - `Shapes Demo <https://github.com/eProsima/ShapesDemo/>`__
              - `v2.6.7 <https://github.com/eProsima/ShapesDemo/releases/tag/v2.6.7>`__
