.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.9.2 (EOL)
===================

This release includes the following ABI compatible **improvements**:

1. **Library improvements**
    1. Improve behavior when ``STRICT_REALTIME`` :ref:`CMake option <cmake_options>` is not enabled.
    2. Using functors for ``for_matched_readers`` parameter.
    3. Improve auto GAPs in Data Sharing.
    4. Improve content filter expression parameters check and verbosity.
    5. Improve validation on ``PID_PROPERTY_LIST`` deserialization.
2. **Fast DDS CLI**
    1. Handle ``SIGTERM`` signal.

This release includes the following **bugfixes**:

1. Security vulnerability
    1. Fix chain of trust issues with a single CA certificate.
2. Library bugfixes
    1. Fix RTPS StatelessWriter ACK check.
    2. UBSan (Undefined Behavior Sanitizer) fixes.
    3. Fix backwards compatibility using SHM communication.
    4. Correctly handle builtin endpoint mask.
    5. Fix crash when enabling DisablePositiveACKsQosPolicy with remote best-effort readers.
    6. Validity check for first sequence number.
    7. ASAN (Address Sanitizer) fixes.
    8. Correctly assign multicast port to multicast initial peers.
    9. Protect against uncaught exception in SHM segment creation.
    10. Initial acknack backoff.
    11. Fix crash when calling
        :cpp:func:`on_requested_deadline_missed() <eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`
        callback.
    12. Security module: Honor ``allow_unauthenticated_participants`` flag.
    13. Fix crashes caused by not capturing every Fast CDR exception.
    14. Correctly resolve aliases in DDSSQLFilter.
    15. Wait for log background thread initialization on the first queued entry.
    16. Fix partition copy in QoS.
    17. Fix Data-Sharing delivery when ``data_count`` is zero.
    18. Fix StatelessWriter locators filtering.
    19. Avoid double definition of ``FASTDDS_ENFORCE_LOG_INFO``.
    20. Fixed long-standing reconnection issue on SHM transport.
3. CI fixes
    1. Fix test building when using ``GTEST_INDIVIDUAL`` :ref:`CMake <cmake_options>` option.
    2. Use correct time unit in latency tests.
4. Synchronization fixes
    1. Take mutex when removing local reader in WLP.
    2. Fix data races in SecurityManager authentication process.
5. Example fixes
    1. Avoid creating entities within callbacks in DynamicHelloWorldExample.
    2. Remove Asio dependency from DeadlineQosExample.
    3. Correct DDS entity deletion order.
    4. Explicitly register TypeObject in ContentFilteredTopicExample.
6. Installer generation
    1. Add documentation fallback when the documentation tag is not found.
7. Non Tier 1 Support
    1. Fix build for GCC5.
    2. Fix build on MSVC 19.36.
    3. Include right header when building for iOS.
    4. Forward compatibility with Boost inter-process 1.74+.
    5. Include missing header files required for compiling with GCC13.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.9.1.rst
.. include:: previous_versions/v2.9.0.rst
.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
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
