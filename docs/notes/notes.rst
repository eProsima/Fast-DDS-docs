.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.11.3
==============

This release includes the following **features**:

    1. Support ``Autofill port`` (automatically set the port) for TCP Transport
    2. Define a ``Superclient`` by environment variable
    3. TCP support for Discovery server CLI and environment variable
    4. Custom pools on DDS layer
    5. Methods to configure transport scenarios

This release includes the following **improvements**:

    1. Allow participant profiles with no rtps tag
    2. Add Log warning message upon receiver resource creation failure, instead of an error
    3. Updatable disable_positive_acks period
    4. Backport workflows from master
    5. Update GitUtils.cmake
    6. Use foonathan memory manager for reducing allocations in ``SharedMemManager.hpp``
    7. Rerun failed tests with ctest option instead of colcon's
    8. Add CCache to all CI jobs
    9. Simplify code in ``CDRMessage``

This release includes the following **fixes**:

    1. Fix encapsulation format in WLP used for the ``ParticipantMessageData``
    2. Fix ``DomainParticipant::register_remote_type`` return when negotiating type
    3. Fix ``RemoteBuiltinEndpointHonoring`` blackbox test
    4. Fix .repos branches
    5. Fix bad-free when receiving malformed DATA submessages
    6. Fix clang warnings
    7. Use STL implementation of ``Timed/RecursiveTimedMutex`` when ``MSVC >= 19.36``
    8. Fix the clang build for clang 14
    9. Fix ``HelloWorld`` ``DataSharing`` example idl
    10. Use ``FASTRTPS_NO_LIB`` on unittest root folder
    11. Fix ``Data Race`` when updating liveliness changed in WLP
    12. Fix TCP sender resources creation
    13. Fix flow controllers unit tests compilation when using ``Fast CDR`` from thirdparty
    14. Add XML parser ``bit_bound`` bounds check
    15. Fix branch selection on Github CI
    16. Better handling of trigger events in docs CI
    17. Use ``SO_EXCLUSIVEADDRUSE`` for Win32 unicast listening sockets
    18. Fix ``PubSubAsReliable`` test
    19. Fix ``FileWatchTest`` for Github windows CI
    20. Fix mac address overflow on windows
    21. Fix missing mandatory attribute check in XML parser struct type
    22. Update TLS unit test certificates
    23. Add missing thread include
    24. Add tests for reconnection with same ``GUID``
    25. Notify data-sharing listener at the end of a successful matching in intraprocess
    26. Fix TCP deadlock on channel reuse
    27. Fix DNS filter in CMakeLists file for tests
    28. Fix bad-free when receiving malformed DATA_FRAG submessage
    29. Fix memory problem when ciphering payload
    30. Fix CVE-2023-50257
    31. Fix build with TLS, but not security
    32. Fix std::move warning
    33. Update PR template to include check for PR description, title and backports
    34. Fix data race on writer destruction while sending heartbeat
    35. Fix comparison in ``remove_from_pdp_reader_history``
    36. Fix data race in PDPListener and SecurityManager
    37. Fix an uninitialized value when building with GCC 13.2.0
    38. Fix max clash with Windows CI
    39. Discard already processed samples on PDPListener
    40. Remove unnecessary TCP warning
    41. Revert "TCP deadlock on channel reuse"

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
