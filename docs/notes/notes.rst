.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.3
==============

This release includes the following **features** in an ABI compatible manner:

1. Support ``Autofill port`` (:ref:`automatically set a port<transport_tcp_transportDescriptor>`) for TCP Transport.
2. Define a :ref:`super client<env_vars_ros_super_client>` by environment variable
3. Support :ref:`TCP Discovery server<use-case-tcp-discovery-server>` CLI and environment variable
4. Define methods (:ref:`environment variable<env_vars_builtin_transports>`,
   :ref:`rtps layer<rtps_layer_builtin_transports>`, :ref:`xml<RTPS>`) to
   :ref:`configure transport scenarios<transport_tcp_enabling>`
5. Custom pools on DDS layer (:ref:`DataWriter<dds_layer_publisher_datawriter_with_payload_pool_creation>` and
   :ref:`DataReader<dds_layer_subscriber_datareader_with_payload_pool_creation>`)

This release includes the following **improvements**:

1. Log warning upon receiver resource creation failure
2. Simplify code in ``CDRMessage``
3. Backport workflows from master
4. Rerun failed tests with ctest option instead of colcon's
5. Use foonathan memory manager for reducing allocations in ``SharedMemManager.hpp``
6. Add CCache to all CI jobs

This release includes the following **bugfixes**:

1. Fix ``RemoteBuiltinEndpointHonoring`` blackbox test
2. Fix bad-free when receiving malformed DATA submessage
3. Fix clang warnings
4. Use STL implementation of ``Timed/RecursiveTimedMutex`` when ``MSVC >= 19.36``
5. Notify datasharing listener at the end of a successful matching in intraprocess
6. Fix the clang build for clang 14
7. Fix ``HelloWorld`` ``DataSharing`` example idl
8. Fix the behaviour of ``disable_positive_acks`` period
9. Fix ``DomainParticipant::register_remote_type`` return when negotiating type
10. Fix Data Race when updating liveliness changed in WLP
11. Fix TCP sender resources creation
12. Fix flow controllers utests compilation when using ``Fast CDR`` from thirdparty
13. Add XML parser ``bit_bound`` bounds check
14. Add tests for reconnection with same GUID
15. Fix Github Windows CI
16. Fix PubSubAsReliable test
17. Use ``FASTRTPS_NO_LIB`` on unittest root folder
18. Fix missing mandatory attribute check in XML parser struct type
19. Fix mac address overflow on windows
20. Use ``SO_EXCLUSIVEADDRUSE`` for Win32 unicast listening sockets
21. Fix FileWatchTest for github windows CI
22. Add missing thread include
23. Update TLS unit test certificates
24. Select correct .repos file on push events
25. Fix documentation CI branch
26. Fix TCP deadlock on channel reuse
27. Fix dns filter in CMakeLists file for tests
28. Fix bad-free when receiving malformed DATA_FRAG submessage
29. Fix memory problem when ciphering payload
30. Fix build with TLS, but not security
31. Fix CVE-2023-50257
32. Fix data race on writer destruction while sending heartbeat
33. Fix comparison in remove_from_pdp_reader_history
34. Fix data race in PDPListener and SecurityManager
35. Update PR template to include check for PR description, title and backports
36. Fix std::move warning
37. Revert "TCP deadlock on channel reuse"
38. Fix max clash with Windows CI

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
