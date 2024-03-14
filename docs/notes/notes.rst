.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.12.2
==============

This release includes the following **features**:

1. Methods to configure transport scenario
2. Support ``Autofill port`` (automatically set the port) for TCP Transport
3. Support TCP for Discovery server CLI and environment variable
4. Define a super client by environment variable
5. Change serialize function default behaviour to omit the data representation
6. ``LARGE_DATA`` Participants logic with same listening ports
7. TCP Client&Server Participant Decision-Making logic
8. Expose Authentication Handshake Properties
9. Enabling multiple interfaces through whitelist in TCP servers
10. Add macOS and Ubuntu Github CI

This release includes the following **improvements**:

1. Improve environment variable substitution algorithm
2. Upgrade dependency version to last patch version in .repos file
3. Rerun failed tests with ctest option instead of colcon's
4. Remove unnecessary TCP warning
5. Update PR template to include check for PR description, title and backports
6. Improvements in GitHub CI

This release includes the following **fixes**:

1. Fix TCP reconnection after open logical port failure
2. TCP unique client announced local port
3. TCP non-blocking send
4. Fix wrong log info messages on TCP
5. Improve ``IgnoreNonExistentSegment`` test
6. Use ``SO_EXCLUSIVEADDRUSE`` for Win32 unicast listening sockets
7. Fix dns filter in CMakeLists file for tests
8. Fix bad-free when receiving malformed DATA_FRAG submessage
9. Fix memory problem related to ciphering payload
10. Fix CVE-2023-50257
11. Fix build with TLS, but not security
12. Fix comparison in ``remove_from_pdp_reader_history``
13. Fix data race in ``PDPListener`` and ``SecurityManager``
14. Discard already processed samples on ``PDPListener``
15. Fix .repos versions
16. Fix the shared memory cleaning script
17. Fix data race on writer destruction while sending heartbeat
18. Return ``const`` reference to the shared pointer instead of a copy in ``get_log_resources``
19. Ignore ``0x8007`` if coming from other vendor
20. Fix Doxygen docs warnings and prepare for compiling with Doxygen 1.10.0
21. Include variety of terminate process signals handler in discovery server
22. Add missing ``TypeLookup`` listeners
23. Add a keyed fragmented change to the reader data instance only when its completed
24. Fix data race on PDP
25. Check History QoS inconsistencies
26. Make DataWriters always send the key hash on keyed topics
27. Prevent index overflow and correctly assert the end iterator in DataSharing
28. Fix uninitialized member in ``RTPSParticipantAttributes``
29. Remove unnecessary ``std::move`` in ``FileWatch.hpp`` causing warning
30. Add missing thread include
31. Add missing virtual destructor for ``StatisticsAncillary``
32. Downgrade CMake version to 3.20

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
