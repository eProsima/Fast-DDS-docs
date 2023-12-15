.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.11.3
==============

This release includes the **following features**:

    1.  Support `Autofill port` (automatically set the port) for TCP Transport
    2. Define a `Superclient` by environment variable

This release includes the following **improvements**:

    1. Allow participant profiles with no rtps tag
    2. Add Log warning message upon receiver resource creation failure, instead of an error
    3. Updatable disable_positive_acks period
    4. Update GitUtils.cmake
    5. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
    6. Rerun failed tests with ctest option instead of colcon's
    7. Add CCache to all CI jobs
    8. Simplify code in `CDRMessage`


This release includes the following **fixes**:

    1. Fix encapsulation format in WLP used for the `ParticipantMessageData`
    2. Fix `DomainParticipant::register_remote_type` return when negotiating type
    13. Fix `RemoteBuiltinEndpointHonoring` blackbox test
    14. Fix bad-free when receiving malformed DATA submessages
    15. Fix clang warnings
    16. Use STL implementation of Timed/RecursiveTimedMutex when `MSVC >= 19.36`
    17. Fix the clang build for clang 14
    18. Fix `HelloWorld` `DataSharing` example idl
    19. Use `FASTRTPS_NO_LIB` on unittest root folder
    20. Fix `Data Race` when updating liveliness changed in WLP
    21. Fix TCP sender resources creation
    22. Fix flow controllers utests compilation when using `Fast CDR` from thirdparty
    23. Add XML parser `bit_bound` bounds check
    24. Fix branch selection on Github CI
    25. Better handling of trigger events in docs CI
    26. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets
    27. Fix `PubSubAsReliable` test
    28. Fix `FileWatchTest` for github windows CI
    29. Fix mac address overflow on windows
    30. Fix missing mandatory attribute check in XML parser struct type
    31. Update TLS unit test certificates
    32. Add missing thread include 
    33. Add tests for reconnection with same `GUID`
    34. Notify datasharing listener also when intraprocess
    35. Fix TCP deadlock on channel reuse

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.11.2.rst
.. include:: previous_versions/v2.11.1.rst
.. include:: previous_versions/v2.11.0.rst
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
