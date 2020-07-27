.. include:: ../includes/aliases.rst
.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _cmake_options:

CMake options
=============

*eProsima Fast DDS* provides numerous CMake options for changing the behavior and configuration of *Fast DDS*.
These options allow the user to enable/disable certain *Fast DDS* settings by defining these options to ON/OFF at the
CMake execution.
This section is structured as follows:
first, the CMake options for the general configuration of *Fast DDS* are described;
then, the options related to the third party libraries are presented;
finally, the possible options for the building of *Fast DDS* tests are defined.

General options
^^^^^^^^^^^^^^^

.. |SQLITE3_PLUGIN| replace:: :ref:`SQLITE3 Plugin<persistence_sqlite3_builtin_plugin>`

The *Fast DDS* CMake options for configuring general settings are shown below, together with their description and
dependency on other options.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`EPROSIMA_INSTALLER`
        - Creates a build for Windows binary installers. Specifically it adds to the list of |br|
          components to install (:class:`CPACK_COMPONENTS_ALL`) the libraries corresponding |br|
          to the Microsoft Visual C++ compiler (MSVC). Setting :class:`EPROSIMA_INSTALLER` |br|
          to ``ON`` has the following effects on other options:

            - :class:`EPROSIMA_BUILD` is set to ``ON``.
            - :class:`BUILD_DOCUMENTATION` is set to ``ON``.
            - :class:`INSTALL_EXAMPLES` is set to ``ON``.

        - ``OFF``
    *   - :class:`EPROSIMA_BUILD`
        - Activates internal *Fast DDS* builds.
          It is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``ON``. |br|
          Setting :class:`EPROSIMA_BUILD` to ``ON`` has the following effects on other options:

            - :class:`INTERNAL_DEBUG` is set to ``ON``.
            - :class:`SHM_TRANSPORT_DEFAULT` is set to ``ON`` and :class:`EPROSIMA_INSTALLER` is set to ``OFF``.
            - :class:`COMPILE_EXAMPLES` is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``OFF``.
            - :class:`THIRDPARTY` is set to ``ON``.
            - :class:`EPROSIMA_GTEST` is set to ``ON`` if GoogleTest (GTest) library was found.
            - :class:`EPROSIMA_GMOCK` is set to ``ON`` if GoogleMock (GMock) library was found.
            - :class:`EPROSIMA_BUILD_TESTS` is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``OFF``.

        - ``OFF``
    *   - :class:`BUILD_SHARED_LIBS`
        - Builds internal libraries as shared libraries, i.e. cause :func:`add_library` CMake function |br|
          to create shared libraries if on. All libraries are built shared unless the library was |br|
          explicitly added as a static library.
        - ``ON``
    *   - :class:`SECURITY`
        - Activates the *Fast DDS* security module. Please refer to :ref:`security` for more information |br|
          on security module.
        - ``OFF``
    *   - :class:`NO_TLS`
        - Disables Transport Layer Security (TLS) Support. Please refer to :ref:`transport_tcp_tls` for |br|
          more information on *Fast DDS* TLS configuration.
        - ``OFF``
    *   - :class:`SHM_TRANSPORT_DEFAULT`
        - Adds Shared Memory transport (SHM) to the default transports. It is set to ``ON`` if |br|
          :class:`EPROSIMA_BUILD` is ``ON``. Please refer to :ref:`SHM <transport_sharedMemory_sharedMemory>` section
          for more information |br|
          on *Fast DDS* SHM transport.
        - ``OFF``
    *   - :class:`COMPILE_EXAMPLES`
        - Builds the *Fast DDS* examples. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and |br|
          :class:`EPROSIMA_INSTALLER` is ``OFF``. These examples can be found in the
          `eProsima Fast DDS <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_ |br|
          `GitHub repository <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_.
        - ``OFF``
    *   - :class:`INSTALL_EXAMPLES`
        - Installs the *Fast DDS* examples, i.e. adds the *Fast DDS* examples to the list of |br|
          components to install (:class:`CPACK_COMPONENTS_ALL`). It is set to ``ON`` if |br|
          :class:`EPROSIMA_INSTALLER` is ``ON``.
        - ``OFF``
    *   - :class:`BUILD_DOCUMENTATION`
        - Uses doxygen to create the *Fast DDS* API reference documentation. It is set to ``ON`` |br|
          if :class:`EPROSIMA_INSTALLER` is ``ON`` or if :class:`CHECK_DOCUMENTATION` is ``ON``.
        - ``OFF``
    *   - :class:`CHECK_DOCUMENTATION`
        - Downloads *Fast DDS* documentation from Read the Docs media servers. The |br|
          documentation files are extracted in the `doc/manual` directory, updating |br|
          any previous version already downloaded. |br|
          If :class:`CHECK_DOCUMENTATION` is ``ON``, :class:`BUILD_DOCUMENTATION` is set to ``ON``.
        - ``OFF``
    *   - :class:`STRICT_REALTIME`
        - Enables a strict real-time behaviour. Please refer to the Real-Time Use Case for |br|
          more information on *Fast DDS* real-time configuration.
        - ``OFF``
    *   - :class:`INTERNAL_DEBUG`
        - Activates |Log::Kind::Info| debug messages (See :ref:`dds_layer_log_intro`). |br|
          For this option to have any effect, i.e. to print the information messages,  |br|
          *Fast DDS* must be debug built. This is done by setting the :class:`CMAKE_BUILD_TYPE` |br|
          option to `Debug`. Moreover, :class:`INTERNAL_DEBUG` is set to ``ON`` if |br|
          :class:`EPROSIMA_BUILD` is ``ON``.
        - ``OFF``
    *   - :class:`SQLITE3_SUPPORT`
        - Builds the |SQLITE3_PLUGIN|. This enables the |TRANSIENT_DURABILITY_QOS-api| |br|
          and |PERSISTENT_DURABILITY_QOS-api| options for the :ref:`durabilitykind`, which |br|
          depend on the :ref:`persistence_service`.
        - ``ON``


Third-party libraries options
-----------------------------

*Fast DDS* relies on the `eProsima FastCDR <https://github.com/eProsima/Fast-CDR>`_ library for serialization
mechanisms.
Moreover, *Fast DDS* requires two external dependencies for its proper operation: Asio and TinyXML2.
Asio is a cross-platform C++ library for network and low-level I/O programming, while TinyXML2 parses the XML profile
files, so *Fast DDS* can use them (see :ref:`xml_profiles`).
These three libraries (eProsima FastCDR, Asio and TinyXML2) can be installed by the user, or downloaded on the
*Fast DDS* build.
In the latter case, they are referred to as *Fast DDS* internal third-party libraries.
This can be done by setting either :class:`THIRDPARTY` or :class:`EPROSIMA_BUILD` to ``ON``.

These libraries can also be configured using *Fast DDS* CMake options.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`THIRDPARTY`
        - Activates the use of Fast CDR and the internal third-party libraries. It is set to ``ON`` |br|
          if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``OFF``
    *   - :class:`THIRDPARTY_UPDATE`
        - Activates the automatic update of just Fast CDR library.
        - ``ON``
    *   - :class:`THIRDPARTY_fastcdr`
        - Links against the internal Fast CDR library.
        - ``OFF``
    *   - :class:`THIRDPARTY_Asio`
        - Links against the internal Asio internal third-party library.
        - ``OFF``
    *   - :class:`THIRDPARTY_TinyXML2`
        - Links against the internal TinyXML2 internal third-party library.
        - ``OFF``
    *   - :class:`THIRDPARTY_android-ifaddrs`
        - Links against the internal android-ifaddrs, an implementation of :func:`getifaddrs` |br|
          for Android. Only if :class:`ANDROID` is ``ON``.
        - ``OFF``

.. Note::

    :class:`ANDROID` is a CMake environment variable that is set to ``1`` if the target system
    (:class:`CMAKE_SYSTEM_NAME`) is Android.

Test options
------------

*eProsima Fast DDS* comes with a full set of tests for continuous integration.
The types of tests are: unit tests, black-box tests, performance tests, profiling tests, and
XTypes tests.
The building and execution of these tests is specified by the *Fast DDS* CMake options shown in the table below.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Default
    *   - :class:`GTEST_INDIVIDUAL`
        - Activate the individual building of GoogleTest tests, since *Fast DDS* tests are |br|
          implemented using the GoogleTest framework. However, the test are compiled |br|
          if :class:`EPROSIMA_BUILD` is set to ``ON``. Therefore, if :class:`GTEST_INDIVIDUAL` is ``OFF`` and |br|
          :class:`EPROSIMA_BUILD` is ``ON``, the tests are processed as a single major test.
        - ``OFF``
    *   - :class:`EPROSIMA_GTEST`
        - Activates special set of GTEST_ROOT, i.e. the root directory of the GoogleTest |br|
          installation.
        - ``OFF``
    *   - :class:`EPROSIMA_GMOCK`
        - Activates special set of GMOCK_ROOT, i.e. the root directory of the GoogleTest C++ |br|
          mocking framework installation. In the latest version of GoogleTest, GoogleMock is |br|
          integrated into it.
        - ``OFF``
    *   - :class:`FASTRTPS_API_TESTS`
        - Enables the building of black-box tests for the verification of RTPS communications |br|
          using the *Fast DDS* RTPS-layer API.
        - ``OFF``
    *   - :class:`FASTDDS_PIM_API_TESTS`
        - Enables the building of black-box tests for the verification of DDS communications |br|
          using the *Fast DDS* DDS-layer API.
        - ``OFF``
    *   - :class:`PERFORMANCE_TESTS`
        - Activates the building of performance tests, except for the video test, which requires |br|
          both :class:`PERFORMANCE_TESTS` and :class:`VIDEO_TESTS` to be set to ``ON``.
        - ``OFF``
    *   - :class:`PROFILING_TESTS`
        - Activates the building of profiling tests using Valgrind.
        - ``OFF``
    *   - :class:`EPROSIMA_BUILD_TESTS`
        - Activates the building of black-box, unit, xtypes, RTPS communication and |br|
          DDS communication tests. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and
          :class:`EPROSIMA_INSTALLER` |br| is ``OFF``.
        - ``OFF``
    *   - :class:`VIDEO_TESTS`
        - If :class:`PERFORMANCE_TESTS` is ``ON``, it will activate the building of video performance tests.
        - ``OFF``
    *   - :class:`DISABLE_UDPV6_TESTS`
        - Disables UDPv6 tests.
        - ``OFF``


