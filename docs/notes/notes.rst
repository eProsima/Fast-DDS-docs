.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.8.2 (EOL)
===================

This release includes the following **improvements**:

1. Add ASAN CI tests for Fast DDS and Discovery Server
2. Mirror master onto both 2.8.x & 2.9.x
3. Doxygen documentation: add deprecation notice to `ThroughputControllerDescriptor`
4. Several fixes to remove warnings in Ubuntu Jammy (22.04)
5. Improve behavior when `HAVE_STRICT_REALTIME` is not set
6. Using functors in `StatefulWriter.cpp` for_matched_readers
7. Fix build on old compilers
8. Avoid creation of DynamicTypes on example
9. Implement a validity check for `firstSN`

This release includes the following **bugfixes**:

1. Fix bug in Topic creation with different Type Name
2. Fix tests failing with subprocess aborted error
3. Fix communication with asymmetric ignoreParticipantFlags
4. Added `ignore_participant_flags()` to `Blackbox_FastRTPS` `PubSubReader`.
5. Fix Deadlock in `remove_participant` (ResourceEvent thread) when compiled WITH_SECURITY
6. Fix failed tests when compiling with statistics enabled
7. Fix Windows `StatistisQosTests.cpp` linkage and Failed test
8. Fixing deadlock in WLP
9. Fix notification lost
10. Fix `StatelessWriter` ACK check
11. Fix `total_unread_` consistent with reader's history upon `get_first_untaken_info()`
12. Fix chain of trust issues
13. Fixed StatisticsSubmessageData unaligned access
14. Fix build error when `GTEST_INDIVIDUAL` is OFF
15. Correctly handle builtin endpoints mask
16. Added missing mutex to `WLP::remove_local_reader`
17. Handle SIGTERM in fast discovery server
18. Improve auto gaps in data sharing
19. Replaced `SecurityManager` temporary `ProxyDatas` with `ProxyPools`
20. Fix crash when `disable_positive_acks` is enable and the remote reader is best-effort
21. Protect from uncaught exception during SHM Segment creation
22. Fix asio dependency
23. Include the right header when building for iOS

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
.. include:: previous_versions/v2.7.1.rst
.. include:: previous_versions/v2.7.0.rst
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
