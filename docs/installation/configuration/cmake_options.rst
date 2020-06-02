.. include:: ../includes/aliases.rst

.. _cmake_options:

CMake options
=============

*eProsima Fast DDS* provides numerous CMake options for changing the behavior and configuration of Fast DDS.
These options allow the user to enable/disable certain Fast DDS settings by defining these options to ON/OFF at the
CMake execution.
All available Fast DDS CMake options are shown below, together with their description and and dependency on other
options.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`EPROSIMA_INSTALLER`
        - Creates a build for Windows installers.
        - ``OFF``
    *   - :class:`EPROSIMA_BUILD`
        - Activates internal Fast DDS builds.
          It is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``ON``.
        - ``OFF``
    *   - :class:`BUILD_SHARED_LIBS`
        - Builds internal libraries as shared libraries, i.e. cause :func:`add_library` CMake function |br|
          to create shared libraries if on. All libraries are built shared unless the library was |br|
          explicitly added as a static library.
        - ``ON``
    *   - :class:`SECURITY`
        - Activates the Fast DDS security module. Please refer to :ref:`security` for more information |br|
          on security module.
        - ``OFF``
    *   - :class:`NO_TLS`
        - Disables Transport Layer Security (TLS) Support. Please refer to TLS over TCP section |br|
          for more information on Fast DDS TLS configuration.
        - ``OFF``
    *   - :class:`SHM_TRANSPORT_DEFAULT`
        - Adds Shared Memory transport (SHM) to the default transports. It is set to ``ON`` if |br|
          :class:`EPROSIMA_BUILD` is ``ON``. Please refer to SHM section for more information |br|
          on Fast DDS TLS configuration.
        - ``OFF``
    *   - :class:`COMPILE_EXAMPLES`
        - Builds the Fast DDS examples. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and |br|
          :class:`EPROSIMA_INSTALLER` is ``OFF``. These examples can be found in the
          `eProsima Fast DDS <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_ |br|
          `GitHub repository <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_.
        - ``OFF``
    *   - :class:`BUILD_DOCUMENTATION`
        - Uses doxygen to create the Fast DDS documentation. It is set to ``ON`` if :class:`EPROSIMA_INSTALLER` |br|
          is ``ON`` or if :class:`CHECK_DOCUMENTATION` is ``ON``.
        - ``OFF``
    *   - :class:`CHECK_DOCUMENTATION`
        - Uses doxygen to check Fast DDS code documentation.
        - ``OFF``
    *   - :class:`INSTALL_EXAMPLES`
        - Installs the Fast DDS examples. It is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``ON``.
        - ``OFF``
    *   - :class:`STRICT_REALTIME`
        - Enables a strict real-time behaviour. Please refer to the Real-Time Use Case for |br|
          more information on Fast DDS real-time configuration.
        - ``OFF``
    *   - :class:`INTERNAL_DEBUG`
        - Activate developer debug messages. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``OFF``

Third-party libraries options
-----------------------------

Fast DDS relies on the `eProsima FastCDR <https://github.com/eProsima/Fast-CDR>`_ library for serialization
mechanisms.
Moreover, Fast DDS requires two internal third-party libraries for its proper operation: Asio and TinyXML2.
Asio is a cross-platform C++ library for network and low-level I/O programming; while TinyXML2 parses the XML profile
files, for later translation into C++ source code.
These libraries can also be configured using Fast DDS CMake options.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`THIRDPARTY`
        - Activates the use of Fast CDR and the internal third-party libraries. It is set to ``ON`` |br|
          if EPROSIMA_BUILD is ``ON``.
        - ``OFF``
    *   - :class:`THIRDPARTY_UPDATE`
        - Activates the automatic update of Fast CDR and the internal third-party libraries.
        - ``ON``
    *   - :class:`THIRDPARTY_fastcdr`
        - Activates the use of Fast CDR library.
        - ``OFF``
    *   - :class:`THIRDPARTY_Asio`
        - Activates the use of Asio internal third-party library.
        - ``OFF``
    *   - :class:`THIRDPARTY_TinyXML2`
        - Activates the use of TinyXML2 internal third-party library.
        - ``OFF``
    *   - :class:`THIRDPARTY_android-ifaddrs`
        - Activates the use of android-ifaddrs, an implementation of :func:`getifaddrs` for Android. |br|
          Only if :class:`ANDROID` is set to ``ON``.
        - ``OFF``

Test options
------------

*eProsima Fast DDS* comes with a full set of tests to check the performance of the library.
These tests are divided according to the layer being tested; thus there are DDS-layer tests and RTPS-layer tests.
The building and execution of these tests is specified by the Fast DDS CMake options shown in the table below.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`FASTRTPS_API_TESTS`
        - Enables the execution of black-box tests for Fast DDS using the Fast DDS RTPS-layer API.
        - ``OFF``
    *   - :class:`FASTDDS_PIM_API_TESTS`
        - Enables the execution of black-box tests for Fast DDS using the Fast DDS DDS-layer API.
        - ``OFF``

Fast DDS tests are implemented using the GoogleTest framework.
To run the tests it is required to enable this framework using the :class:`GTEST_INDIVIDUAL` option.
The configuration of the installation directories is also allowed as indicated in the following table.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`GTEST_INDIVIDUAL`
        - Activate the execution of GoogleTest tests.
        - ``OFF``
    *   - :class:`EPROSIMA_GTEST`
        - Activates special set of GTEST_ROOT (the root directory of the GoogleTest installation).
        - ``OFF``
    *   - :class:`EPROSIMA_GMOCK`
        - Activates special set of GMOCK_ROOT (the root directory of the GoogleMock installation).
        - ``OFF``

Furthermore, several CMake options are available to specify which tests are to be executed.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`PERFORMANCE_TESTS`
        - Activates the building and execution of performance tests.
        - ``OFF``
    *   - :class:`PROFILING_TESTS`
        - Activates the building and execution of profiling tests using Valgrind.
        - ``OFF``
    *   - :class:`EPROSIMA_BUILD_TESTS`
        - Activates the building and execution of black-box, unit, xtypes, RTPS communication and |br|
          DDS communication tests. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and
          :class:`EPROSIMA_INSTALLER` |br| is ``OFF``.
        - ``OFF``
    *   - :class:`VIDEO_TESTS`
        - Activate the building and execution of video performance tests.
        - ``OFF``
    *   - :class:`DISABLE_UDPV6_TESTS`
        - Disables UDPv6 tests.
        - ``OFF``



