.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.7
=============

This release includes the following **features**:

1. Support `Autofill port` (:ref:`automatically set a port<transport_tcp_transportDescriptor>`) for TCP Transport.
2. Define a :ref:`super client<env_vars_ros_super_client>` by environment variable
3. Support :ref:`TCP Discovery server<use-case-tcp-discovery-server>` CLI and environment variable
4. Define methods (:ref:`environment variable<env_vars_builtin_transports>`,
   :ref:`rtps layer<rtps_layer_builtin_transports>`, :ref:`xml<RTPS>`) to
   :ref:`configure transport scenarios<transport_tcp_enabling>`
5. :ref:`Secure discovery server<DS_security>`

This release includes the following **improvements**:

1. Log warning message upon receiver resource creation failure.
2. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
3. Simplify code in `CDRMessage`.
4. Rerun failed tests with ctest option instead of colcon's.
5. Add CCache to all CI jobs.
6. Upgrade CMake minimum requirement to 3.16.3
7. Update PR checklist template. Backports and Description

This release includes the following **bugfixes**:

1. Fix `DomainParticipant::register_remote_type` return when negotiating type.
2. Fix `RemoteBuiltinEndpointHonoring` blackbox test.
3. Allow participant profiles with no rtps tag.
4. Fix bad-free when receiving malformed DATA submessage.
5. Fix clang warnings (https://github.com/eProsima/Fast-DDS/pull/3906)
6. Use STL implementation of `Timed/RecursiveTimedMutex` when `MSVC >= 19.36`.
7. Fix encapsulation format in WLP.
8. Fix the clang build for clang 14.
9. Notify datasharing listener at the end of a successful matching in intraprocess.
10. Updatable disable_positive_acks period.
11. Fix Data Race when updating liveliness changed in WLP.
12. Fix TCP sender resources creation.
13. Add tests for reconnection with same GUID.
14. Fix flow controllers utests compilation when using Fast CDR from thirdparty.
15. Add XML parser bit_bound bounds check.
16. Use `FASTRTPS_NO_LIB` on unittest root folder.
17. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets.
18. Fix mac address overflow on windows.
19. Fix `PubSubAsReliable` test.
20. Fix `FileWatchTest`.
21. Add missing thread include.
22. Fix missing mandatory attribute check in XML parser struct type.
23. Better handling of trigger events in docs CI.
24. Fix memory problem when ciphering payload
25. Fix bad-free when receiving malformed DATA_FRAG submessage
26. Fix data race on writer destruction while sending heartbeat
27. Fix DiscoveryServer list access deadlock
28. Fix c++11 support for fast discovery server tool
29. Fix CVE-2023-50257
30. Fix std::move warning
31. Fix Github Windows CI

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.6.6.rst
.. include:: previous_versions/v2.6.5.rst
.. include:: previous_versions/v2.6.4.rst
.. include:: previous_versions/v2.6.3.rst
.. include:: previous_versions/v2.6.2.rst
.. include:: previous_versions/v2.6.1.rst
.. include:: previous_versions/v2.6.0.rst
.. include:: previous_versions/v2.5.1.rst
.. include:: previous_versions/v2.5.0.rst
.. include:: previous_versions/v2.4.2.rst
.. include:: previous_versions/v2.4.1.rst
.. include:: previous_versions/v2.4.0.rst
.. include:: previous_versions/v2.3.4.rst
.. include:: previous_versions/v2.3.3.rst
.. include:: previous_versions/v2.3.2.rst
.. include:: previous_versions/v2.3.1.rst
.. include:: previous_versions/v2.3.0.rst
.. include:: previous_versions/v2.2.1.rst
.. include:: previous_versions/v2.2.0.rst
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
