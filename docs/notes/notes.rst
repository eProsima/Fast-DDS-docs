.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.2
==============

This release includes the following **improvements**:

1. Fix Data-Sharing delivery when data_count is zero
2. Improve performance of intraprocess plus data-sharing
3. Improve content filter expression parameters checks and verbosity
4. Improve validation on PID_PROPERTY_LIST deserialization
5. Participant ignore local endpoints
6. Pick smallest available participant ID for new participants
7. Improve endpoint QoS XML tags
8. Forward compatibility with boost interprocess 1.74+
9. Cap Thread Sanitizer memory usage to prevent runner shutdown
10. Allow participant XML profile with no <rtps> tag
11. Add unsupported note in API documentation to new ignore DomainParticipantListener callbacks
12. Add documentation version fallback

This release includes the following **bugfixes**:

1. Fixed long-standing reconnection issue on SHM transport
2. Fix null dereference when fuzzing
3. Fix segfault when creating two participant with same fixed id
4. Fix UBSan (Undefined Behavior Sanitizer) issues
5. Fix listener selection for on_requested_deadline_missed
6. Fix build on msvc 19.36.32528
7. Fix XML schema to set Transport descriptor kind as NOT mandatory
8. Fix missing includes
9. Fix overhead time unit
10. Fix request reply example spelling typo
11. Fix topic deletion after endpoint in examples
12. Fix Data-Sharing delivery when data_count is zero
13. Wait for log background thread initialization on the first queued entry
14. Fix alias resolve in DDSSQLFilter
15. Fix partition copy in QoS
16. Fix StatelessWriter locators filtering
17. Fix XMLParser null-dereference in parseLogConfig
18. Fix encapsulation format in WLP
19. Replace uint64_t by 8 in alignas specifier
20. Capture all Fast CDR exceptions
21. Security module: Honor allow_unauthenticated_participants flag
22. Explicitly register type object in ContentFilteredTopicExample
23. Avoid double definition of FASTDDS_ENFORCE_LOG_INFO
24. Fix API Fast DDS v2.10.0 API break calling correctly on_participant_discovery callbacks
25. Remove mutex from TimedEventImpl

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

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
