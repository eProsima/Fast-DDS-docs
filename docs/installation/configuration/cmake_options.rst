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
        - Possible values
        - Default
    *   - :class:`EPROSIMA_INSTALLER`
        - Creates a build for Windows binary installers. Specifically it adds to the list of
          components to install (:class:`CPACK_COMPONENTS_ALL`) the libraries corresponding
          to the Microsoft Visual C++ compiler (MSVC). Setting :class:`EPROSIMA_INSTALLER`
          to ``ON`` has the following effects on other options:

          - :class:`EPROSIMA_BUILD` is set to ``ON``.
          - :class:`BUILD_DOCUMENTATION` is set to ``ON``.
          - :class:`INSTALL_EXAMPLES` is set to ``ON``.

        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`EPROSIMA_BUILD`
        - Activates internal *Fast DDS* builds.
          It is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is
          ``ON``. Setting :class:`EPROSIMA_BUILD` to ``ON`` has the following effects on other
          options:

          - :class:`INTERNAL_DEBUG` is set to ``ON``.
          - :class:`COMPILE_EXAMPLES` is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``OFF``.
          - :class:`THIRDPARTY_fastcdr` is set to ``ON`` if it was not set to ``FORCE``.
          - :class:`THIRDPARTY_Asio` is set to ``ON`` if it was not set to ``FORCE``.
          - :class:`THIRDPARTY_TinyXML2` is set to ``ON`` if it was not set to ``FORCE``.
          - :class:`THIRDPARTY_android-ifaddrs` is set to ``ON`` if it was not set to ``FORCE``.
          - :class:`EPROSIMA_BUILD_TESTS` is set to ``ON`` if :class:`EPROSIMA_INSTALLER` is ``OFF``.

        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`BUILD_SHARED_LIBS`
        - Builds internal libraries as shared libraries, i.e. cause :func:`add_library` CMake
          function to create shared libraries if on. All libraries are built shared unless the
          library was explicitly added as a static library.
        - ``ON`` ``OFF``
        - ``ON``
    *   - :class:`SECURITY`
        - Activates the *Fast DDS* security module. Please refer to :ref:`security` for more
          information on security module.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`NO_TLS`
        - Disables Transport Layer Security (TLS) Support. Please refer to :ref:`transport_tcp_tls`
          for more information on *Fast DDS* TLS configuration.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`SHM_TRANSPORT_DEFAULT`
        - Adds Shared Memory transport (SHM) to the default transports.
          Please refer to
          :ref:`SHM <transport_sharedMemory_sharedMemory>` section for more information on *Fast DDS* SHM transport.
        - ``ON`` ``OFF``
        - ``ON``
    *   - :class:`FASTDDS_STATISTICS`
        - Enables the *Fast DDS* Statistics module. Please refer to :ref:`statistics` for
          more information on this module.
        - ``ON`` ``OFF``
        - ``ON``
    *   - :class:`COMPILE_EXAMPLES`
        - Builds the *Fast DDS* examples. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and
          :class:`EPROSIMA_INSTALLER` is ``OFF``. These examples can be found in the
          `eProsima Fast DDS <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_
          `GitHub repository <https://github.com/eProsima/Fast-DDS/tree/master/examples>`_.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`INSTALL_EXAMPLES`
        - Installs the *Fast DDS* examples, i.e. adds the *Fast DDS* examples to the list of
          components to install (:class:`CPACK_COMPONENTS_ALL`). It is set to ``ON`` if
          :class:`EPROSIMA_INSTALLER` is ``ON``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`BUILD_DOCUMENTATION`
        - Uses doxygen to create the *Fast DDS* API reference documentation. It is set to
          ``ON`` if :class:`EPROSIMA_INSTALLER` is ``ON`` or if :class:`CHECK_DOCUMENTATION` is ``ON``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`CHECK_DOCUMENTATION`
        - Downloads *Fast DDS* documentation from Read the Docs media servers. The
          documentation files are extracted in the `doc/manual` directory, updating
          any previous version already downloaded.
          If :class:`CHECK_DOCUMENTATION` is ``ON``, :class:`BUILD_DOCUMENTATION` is set to ``ON``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`STRICT_REALTIME`
        - Enables a strict real-time behaviour. Please refer to the Real-Time Use Case for
          more information on *Fast DDS* real-time configuration.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`SQLITE3_SUPPORT`
        - Builds the |SQLITE3_PLUGIN|, which enables the |TRANSIENT_DURABILITY_QOS-api|
          and |PERSISTENT_DURABILITY_QOS-api| options for the :ref:`durabilitykind`
          and therefore the :ref:`persistence_service`.
        - ``ON`` ``OFF``
        - ``ON``
    *   - :class:`APPEND_PROJECT_NAME_TO_INCLUDEDIR`
        - When ``ON`` headers are installed to a path ending with a folder called ``fastdds``.
          This avoids include directory search order issues when overriding this package
          from a merged catkin, ament, or colcon workspace.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`USE_THIRDPARTY_SHARED_MUTEX`
        - When ``ON`` a custom implementation of ``shared_mutex`` is used instead of the STL one.
          The C++ Standard has not yet (C++20) imposed any requirements on ``shared_mutex``
          priority policies implementation, as POSIX_ does, thus each platform made its own choices:

          - Windows & Boost defaults to ``PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP``.
          - Linux & Mac defaults to ``PTHREAD_RWLOCK_PREFER_READER_NP``.

          Fast-DDS requires the use of ``PTHREAD_RWLOCK_PREFER_READER_NP`` which is the one enforced
          in its deadlock prevention logic.
          Fast-DDS will test the framework STL implementation (if available) and will only use it if
          it enforces ``PTHREAD_RWLOCK_PREFER_READER_NP``. Otherwise it will automatically fallback to
          a custom implementation. |br| |br|
          This flag will enforce the use of the custom implementation in all cases. |br| |br|
          Note that setting the flag ``OFF`` will not prevent the use of the custom implementation
          in those frameworks that default to ``PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP``. |br| |br|
          This flag prevents spurious thread sanitizer reports on *GCC/Clang* STL implementations.
        - ``ON`` ``OFF``
        - ``OFF`` (Linux & Mac), ``ON`` (Windows)
    *   - :class:`SANITIZER`
        - Adds run-time instrumentation to the code. Supported options are:

          - ``Thread`` enables Thread Sanitizer. |br|
          - ``Address`` enables Address Sanitizer.

        - ``OFF`` |br| ``Address`` |br| ``Thread``
        - ``OFF``

.. _POSIX: https://man7.org/linux/man-pages/man3/pthread_rwlockattr_setkind_np.3.html

Log options
^^^^^^^^^^^

*Fast DDS* uses its own configurable **Log module**  with different verbosity levels.
Please, refer to :ref:`dds_layer_log_intro` section for more information.

This module can be configured using *Fast DDS* CMake arguments regarding the following options.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Possible values
        - Default
    *   - :class:`LOG_CONSUMER_DEFAULT`
        - Selects the default log consumer for the logging module.
          ``AUTO`` has the same behavior as ``STDOUTERR``.
          For more information, please refer to :ref:`Log consumers <dds_layer_log_consumer>`.
        - ``AUTO`` ``STDOUT``
          ``STDOUTERR``
        - ``AUTO``
    *   - :class:`LOG_NO_INFO`
        - Deactivates Info Log level. |br|
          If *Fast DDS* is built in debug mode for Single-Config generators,
          the default value will be ``OFF``.
        - ``ON`` ``OFF``
        - ``ON``
    *   - :class:`FASTDDS_ENFORCE_LOG_INFO`
        - Enables Info Log level even on non ``Debug`` configurations. |br|
          This option only takes action if :class:`LOG_NO_INFO` is set to ``OFF``
          (see :ref:`dds_layer_log_disable`). |br|
          Mind that this may entail a significant performance hit.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`LOG_NO_WARNING`
        - Deactivates Warning Log level.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`LOG_NO_ERROR`
        - Deactivates Error Log level.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`INTERNAL_DEBUG`
        - Activates compilation of log messages
          (See :ref:`dds_layer_log_disable`). |br|
          Moreover, :class:`INTERNAL_DEBUG` is set to ``ON`` if
          :class:`EPROSIMA_BUILD` is ``ON``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`ENABLE_OLD_LOG_MACROS`
        - Enable old log macros
          (See :ref:`old_log_macros_disable`). |br|
        - ``ON`` ``OFF``
        - ``ON``

.. _third-party_libraries-options:

Third-party libraries options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
        - Possible values
        - Default
    *   - :class:`THIRDPARTY_fastcdr`
        - ``ON`` activates the use of the internal Fast CDR third-party library if it is not
          found elsewhere in the system. |br|
          ``FORCE`` activates the use of the internal Fast CDR third-party library
          regardless of whether it can be found elsewhere in the system. |br|
          ``OFF`` deactivates the use of the internal Fast CDR third-party library.
          If it is not set to ``FORCE``, it is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``ON`` ``OFF`` ``FORCE``
        - ``OFF``
    *   - :class:`THIRDPARTY_Asio`
        - ``ON`` activates the use of the internal Asio third-party library if it is not
          found elsewhere in the system. |br|
          ``FORCE`` activates the use of the internal Asio third-party library
          regardless of whether it can be found elsewhere in the system. |br|
          ``OFF`` deactivates the use of the internal Asio third-party library.
          If it is not set to ``FORCE``, it is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``ON`` ``OFF`` ``FORCE``
        - ``OFF``
    *   - :class:`THIRDPARTY_TinyXML2`
        - ``ON`` activates the use of the internal TinyXML2 third-party library if it is not
          found elsewhere in the system. |br|
          ``FORCE`` activates the use of the internal TinyXML2 third-party library
          regardless of whether it can be found elsewhere in the system. |br|
          ``OFF`` deactivates the use of the internal TinyXML2 third-party library.
          If it is not set to ``FORCE``, it is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``ON`` ``OFF`` ``FORCE``
        - ``OFF``
    *   - :class:`THIRDPARTY_android-ifaddrs`
        - android-ifaddrs is an implementation of :func:`getifaddrs` for Android. |br|
          Only used if :class:`ANDROID` is ``1``. |br|
          ``ON`` activates the use of the internal android-ifaddrs third-party library if it is not
          found elsewhere in the system. |br|
          ``FORCE`` activates the use of the internal android-ifaddrs third-party library
          regardless of whether it can be found elsewhere in the system. |br|
          ``OFF`` deactivates the use of the internal android-ifaddrs third-party library.
          If it is not set to ``FORCE``, it is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON``.
        - ``ON`` ``OFF`` ``FORCE``
        - ``OFF``
    *   - :class:`THIRDPARTY`
        - Unless they are otherwise specified, sets value of all third-party
          git submodules :class:`THIRDPARTY_fastcdr`, :class:`THIRDPARTY_Asio`,
          :class:`THIRDPARTY_TinyXML2`, and :class:`THIRDPARTY_android-ifaddrs`.
        - ``ON`` ``OFF`` ``FORCE``
        - ``OFF``
    *   - :class:`THIRDPARTY_UPDATE`
        - Activates the update of all third-party git submodules.
        - ``ON`` ``OFF``
        - ``ON``

.. Note::

    :class:`ANDROID` is a CMake environment variable that is set to ``1`` if the target system
    (:class:`CMAKE_SYSTEM_NAME`) is Android.

Test options
^^^^^^^^^^^^

*eProsima Fast DDS* comes with a full set of tests for continuous integration.
The types of tests are: unit tests, black-box tests, performance tests, profiling tests, and
XTypes tests.
The building and execution of these tests is specified by the *Fast DDS* CMake options shown in the table below.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Possible values
        - Default
    *   - :class:`FASTDDS_PIM_API_TESTS`
        - Enables the building of black-box tests for the verification of DDS communications
          using the *Fast DDS* DDS-layer API.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`FASTDDS_EXAMPLE_TESTS`
        - Enables the building of example tests for the verification of the Fast DDS examples
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`PERFORMANCE_TESTS`
        - Activates the building of performance tests, except for the video test, which requires
          both :class:`PERFORMANCE_TESTS` and :class:`VIDEO_TESTS` to be set to ``ON``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`PROFILING_TESTS`
        - Activates the building of profiling tests using Valgrind.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`EPROSIMA_BUILD_TESTS`
        - Activates the building of black-box, unit, xtypes, RTPS communication and
          DDS communication tests. It is set to ``ON`` if :class:`EPROSIMA_BUILD` is ``ON`` and
          :class:`EPROSIMA_INSTALLER` is ``OFF``.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`VIDEO_TESTS`
        - If :class:`PERFORMANCE_TESTS` is ``ON``, it will activate the building of video performance tests.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`DISABLE_UDPV6_TESTS`
        - Disables UDPv6 tests.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`INSTALL_ANDROID_TESTS`
        - Android cross-compilation only. Marks the tests for installation on the
          connected device/emulator.
        - ``ON`` ``OFF``
        - ``OFF``
    *   - :class:`ANDROID_TESTING_ROOT`
        - Android cross-compilation only. Path on the Android device/emulator to
          use for installing and running the tests.
        - ``Valid Unix filesystem path string``
        - ``""``

Python-Binding
^^^^^^^^^^^^^^

The *Fast DDS Python-Binding* CMake options are shown below, together with their description and values.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Possible values
        - Default
    *   - :class:`USE_PYTHON_STABLE_ABI`
        - Enables building a Python extension module using the Stable ABI (Application Binary Interface),
          allowing the resulting binary to be used across multiple Python 3 minor versions without recompilation.

          When is enabled, the extension is compatible with all newer (>= 3.9) Python 3.x interpreters
          (e.g.: A module built for Python 3.13 will work unmodified on 3.12, 3.11, etc.)

        - ``ON`` ``OFF``
        - ``OFF``