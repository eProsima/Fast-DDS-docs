.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.13.0
==============

.. note::

  This release upgrades the following Fast DDS dependencies:

  * `Fast CDR v2.1.2 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.2>`_
  * `Fast DDS-Gen v3.2.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.0>`_


This release includes the following **features**:

1. Support Monitor Service.
2. Enable configuration of thread settings for all threads (both through the C++ API and XML configuration files).
3. Support `Autofill port` (automatic assignment of a port) for TCP Transport.
3. Support TCP for Discovery server CLI and environment variable.
4. Usage of gtest_discover_tests.
5. Define a super client by environment variable.
6. Support adding interfaces to the interface whitelist by the name.
7. Add new methods to configure Builtin Transport.
8. Support `DataRepresentationQos`.
9. Change serialize function default behavior to omit the data representation.
10. Upgrade Fast CDR submodule to v2.1.2.
11. Update roadmap & platforms support.

This release includes the following **improvements**:

1. Rerun failed tests with ctest option instead of colcon's.
2. Add CCache to all CI jobs.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**

    1. Fix compilation of `XMLProfileParserTests` when building without security.
    2. Improve `IgnoreNonExistentSegment` test for Windows.
    3. Add missing thread includes.
    4. Fix warning in Mac rewarding unnecessary lambda capture.
    5. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets.
    6. Fix gtest discovery timeout.
    7. Mark `on_participant_discovery` overload removal.
    8. Fix uninitialized member in `BuiltinAttributes` class.
    9. Fix set affinity directive for Android.
    10. Fix Monitor Service types & test without security.
    11. Fix TCP deadlock on channel reuse.
    12. Fix dns filter in `CMakeLists` file for tests.
    13. Fix memory issues related to ciphering payload.
    14. Fix a bad-free when receiving a malformed `DATA_FRAG` submessage.
    15. Fix CVE-2023-50257.
    16. Fix compilation of Fast DDS Python tests.
    17. Fix data race on writer destruction while sending hearbeat.
    18. Fix build with TLS, when `SECURITY=OFF` and `NO_TLS=OFF`.

2. CI fixes:

    1. Fix colcon on github CI.
    2. Better handling of trigger events in docs CI.

.. note::
  When upgrading to version 2.13.0 it is **advisable** to regenerate generated source from IDL files
   using `Fast DDS-Gen v3.2.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.1.1>`_.

Previous versions
=================

.. include:: previous_versions/v2.12.1.rst
.. include:: previous_versions/v2.12.0.rst
.. include:: previous_versions/v2.11.2.rst
.. include:: previous_versions/v2.11.1.rst
.. include:: previous_versions/v2.11.0.rst
.. include:: previous_versions/v2.10.2.rst
.. include:: previous_versions/v2.10.1.rst
.. include:: previous_versions/v2.10.0.rst
.. include:: previous_versions/v2.9.2.rst
.. include:: previous_versions/v2.9.1.rst
.. include:: previous_versions/v2.9.0.rst
.. include:: previous_versions/v2.8.2.rst
.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
.. include:: previous_versions/v2.7.2.rst
.. include:: previous_versions/v2.7.1.rst
.. include:: previous_versions/v2.7.0.rst
.. include:: previous_versions/v2.6.6.rst
.. include:: previous_versions/v2.6.5.rst
.. include:: previous_versions/v2.6.4.rst
.. include:: previous_versions/v2.6.3.rst
.. include:: previous_versions/v2.6.2.rst
.. include:: previous_versions/v2.6.1.rst
.. include:: previous_versions/v2.6.0.rst
.. include:: previous_versions/v2.5.2.rst
.. include:: previous_versions/v2.5.1.rst
.. include:: previous_versions/v2.5.0.rst
.. include:: previous_versions/v2.4.2.rst
.. include:: previous_versions/v2.4.1.rst
.. include:: previous_versions/v2.4.0.rst
.. include:: previous_versions/v2.3.6.rst
.. include:: previous_versions/v2.3.5.rst
.. include:: previous_versions/v2.3.4.rst
.. include:: previous_versions/v2.3.3.rst
.. include:: previous_versions/v2.3.2.rst
.. include:: previous_versions/v2.3.1.rst
.. include:: previous_versions/v2.3.0.rst
.. include:: previous_versions/v2.2.1.rst
.. include:: previous_versions/v2.2.0.rst
.. include:: previous_versions/v2.1.4.rst
.. include:: previous_versions/v2.1.3.rst
.. include:: previous_versions/v2.1.2.rst
.. include:: previous_versions/v2.1.1.rst
.. include:: previous_versions/v2.1.0.rst
.. include:: previous_versions/v2.0.3.rst
.. include:: previous_versions/v2.0.2.rst
.. include:: previous_versions/v2.0.1.rst
.. include:: previous_versions/v2.0.0.rst
.. include:: previous_versions/v1.10.1.rst
.. include:: previous_versions/v1.10.0.rst
.. include:: previous_versions/v1.9.5.rst
.. include:: previous_versions/v1.9.4.rst
.. include:: previous_versions/v1.9.3.rst
.. include:: previous_versions/v1.9.2.rst
.. include:: previous_versions/v1.9.1.rst
.. include:: previous_versions/v1.9.0.rst
.. include:: previous_versions/v1.8.5.rst
.. include:: previous_versions/v1.8.4.rst
.. include:: previous_versions/v1.8.3.rst
.. include:: previous_versions/v1.8.2.rst
.. include:: previous_versions/v1.8.1.rst
.. include:: previous_versions/v1.8.0.rst
.. include:: previous_versions/v1.7.3.rst
.. include:: previous_versions/v1.7.2.rst
.. include:: previous_versions/v1.7.1.rst
.. include:: previous_versions/v1.7.0.rst
.. include:: previous_versions/v1.6.0.rst
.. include:: previous_versions/v1.5.0.rst
.. include:: previous_versions/v1.4.0.rst
.. include:: previous_versions/v1.3.1.rst
.. include:: previous_versions/v1.3.0.rst
.. include:: previous_versions/v1.2.0.rst
