.. _cmake_options:

CMake compilation options
=========================

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - EPROSIMA_INSTALLER
        - Enables the creation of a build to create Windows installers.
        - ``OFF``
    *   - EPROSIMA_BUILD
        - Activates internal building.
        - ``ON``
    *   - BUILD_SHARED_LIBS
        - Creates shared libraries.
        - ``ON``
    *   - SECURITY
        - Activates security.
        - ``OFF``
    *   - NO_TLS
        - Disables TLS Support.
        - ``OFF``
    *   - SQLITE3_SUPPORT
        - Activates SQLite3 support.
        - ``ON``
    *   - SHM_TRANSPORT_DEFAULT
        - Adds Shared Memory transport (SHM) to the default transports.
        - ``OFF``
    *   - COMPILE_EXAMPLES
        - Builds the Fast DDS examples.
        - ``OFF``
    *   - BUILD_DOCUMENTATION
        - Use doxygen to create the Fast DDS documentation.
        - ``OFF``
    *   - CHECK_DOCUMENTATION
        - Use doxygen to check Fast DDS code documentation.
        - ``OFF``
    *   - INSTALL_EXAMPLES
        - Installs the Fast DDS examples.
        - ``OFF``
    *   - STRICT_REALTIME
        - Enables a strict real-time behaviour.
        - ``OFF``
    *   - INTERNAL_DEBUG
        - Activate developer debug messages.
        - ``OFF``


.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - RTPS_API_TESTS
        - Enables the execution of tests for Fast DDS using the RTPS API.
        - ``OFF``
    *   - FASTRTPS_API_TESTS
        - Enables the execution of tests for Fast DDS using the Fast DDS RTPS-layer API.
        - ``OFF``
    *   - FASTDDS_PIM_API_TESTS
        - Enables the execution of tests for Fast DDS using the Fast DDS DDS-layer API.
        - ``OFF``

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - PERFORMANCE_TESTS
        - Activates the building and execution of performance tests.
        - ``OFF``
    *   - PROFILING_TESTS
        - Activates the building and execution of profiling tests.
        - ``OFF``
    *   - EPROSIMA_BUILD_TESTS
        - Activates the building and execution of unit tests. and integral tests.
        - ``OFF``
    *   - VIDEO_TESTS
        - Activate the building and execution of video performance tests.
        - ``OFF``
    *   - DISABLE_UDPV6_TESTS
        - Disables UDPv6 tests.
        - ``OFF``
