.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.3
==============

This release includes the **following features**:

1. Support `Autofill port` (automatically set the port) for TCP Transport
2. Define a super client by environment variable

This release includes the following **improvements**:

1. Log warning upon receiver resource creation failure
2. Simplify code in `CDRMessage`
3. Rerun failed tests with ctest option instead of colcon's
4. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
5. Add CCache to all CI jobs

This release includes the following **bugfixes**:

1. Fix RemoteBuiltinEndpointHonoring blackbox test (#3794)
2. Fix bad-free when receiving malformed DATA submessage (#3861)
3. Fix clang warnings (#3905)
4. Use STL implementation of Timed/RecursiveTimedMutex when MSVC >= 19.36 (#3917)
5. Notify datasharing listener at the end of a successful matching in intraprocess (#3899)
6. Fix the clang build for clang 14 (#3928)
7. Fix HelloWorld DataSharing example idl (#3885)
8. Fix the behaviour of disable_positive_acks period (#3896)
9. Fix DomainParticipant::register_remote_type return when negotiating type (#3797)
10. Fix Data Race when updating liveliness changed in WLP (#3960)
11. Fix TCP sender resources creation (#3963)
12. Fix flow controllers utests compilation when using Fast CDR from thirdparty (#3985)
13. Add XML parser bit_bound bounds check (#3990)
14. Add tests for reconnection with same GUID (#3977)
15. Fix Github Windows CI (#4086)
16. Fix PubSubAsReliable test (#4010)
17. Use FASTRTPS_NO_LIB on unittest root folder (#3872)
18. Fix missing mandatory attribute check in XML parser struct type (#4007)
19. Fix mac address overflow on windows (#4020)
20. Use SO_EXCLUSIVEADDRUSE for Win32 unicast listening sockets (#4072)
21. Fix FileWatchTest for github windows CI (#4023)
22. Add missing thread include (#4064)
23. Update TLS unit test certificates (#4068)
24. Fix documentation CI branch (#4089)
25. Fix TCP deadlock on channel reuse (#4129)

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.10.2.rst
.. include:: previous_versions/v2.10.1.rst
.. include:: previous_versions/v2.10.0.rst
.. include:: previous_versions/v2.9.2.rst
.. include:: previous_versions/v2.9.1.rst
.. include:: previous_versions/v2.9.0.rst
.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
.. include:: previous_versions/v2.7.2.rst
.. include:: previous_versions/v2.7.1.rst
.. include:: previous_versions/v2.7.0.rst
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
