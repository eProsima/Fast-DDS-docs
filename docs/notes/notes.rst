.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.5
=============

This release includes the following **improvements**:

1. Improve behavior when ``STRICT_REALTIME`` :ref:`CMake option <cmake_options>` is not enabled.
2. Using functors for ``for_matched_readers`` parameter.
3. Improve auto GAPs in Data Sharing.

This release includes the following **bugfixes**:

1. Fix RTPS StatelessWriter ACK check.
2. Fix ``total_read_`` to be consistent with Reader's History after
   :cpp:func:`DataReader::get_first_untaken_info()<eprosima::fastdds::dds::DataReader::get_first_untaken_info>`.
3. Add deprecation notice to ``ThroughputControllerDescriptor``.
4. UBSan (Undefined Behavior Sanitizer) fixes.
5. Several dependencies fixes upgrading to Ubuntu 22.04.
6. Fix chain of trust issues with a single CA certificate.
7. Correctly handle builtin endpoints mask.
8. Take mutex when removing local reader in WLP.
9. Handle ``SIGTERM`` signal in Fast DDS CLI.
10. Fix data races in SecurityManager authentication process.
11. Avoid creating entities within callbacks in DynamicHelloWorldExample.
12. Remove Asio dependency from DeadlineQoSExample.
13. Validity check for first sequence number.
14. Include right header when building for iOS.
15. Fix build on MSVC 19.
16. Correctly assign multicast port to multicast initial peers.
17. Select correct listener for
    :cpp:func:`on_requested_deadline_missed()<eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`.
18. Forward compatibility with boost inter-process 1.74+.
19. Fix missing includes when building with GCC 13.
20. Honor
    :cpp:var:`allow_unauthenticated_participants<eprosima::fastrtps::rtps::security::ParticipantSecurityAttributes::allow_unauthenticated_participants>`
    flag.
21. Capture all Fast CDR exceptions.
22. Fix example to delete Topic after deleting the corresponding Endpoint.
23. Protect against uncaught exception in SHM segment creation.
24. Initial acknack backoff.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.6.6.rst
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
