.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.4
==============

This release includes the following **features** in an ABI compatible manner:

#. TCP Client and Server Participant Decision Making.
#. :ref:`Authentication Handshake Properties <property_policies_security>` documentation.
#. New :ref:`max_message_size property <property_max_message_size>` to limit output datagrams size.

This release includes the following **improvements**:

#. Return const reference in ``get_log_resources``.
#. Include variety of terminate process signals handler in discovery server.
#. Check History QoS inconsistencies.
#. Make DataWriters always send the key hash on keyed topics.
#. ``LARGE_DATA`` Participants logic with same listening ports.
#. Effectively assert ``AUTOMATIC/MANUAL_BY_PARTICIPANT`` liveliness.
#. Improve checklist on PR template.
#. Allow processing of ``AckNack`` submessages with ``count == 0``.
#. Internal refactor on port handling.
#. Upgrade Fast CDR submodule to v1.0.28

TCP transport improvements:

#. TCP ``non-blocking`` send.
#. Enabling multiple interfaces through whitelist in TCP servers.
#. Set real TCP ``non-blocking-send`` limitation.
#. Clean up TCP send resources on peer disconnection.

Github CI management:

#. Add macOS Github CI.
#. Avoid running GitHub CI if PR has conflicts.
#. Add Ubuntu Github CI.
#. Improve CI version management.
#. Pin CMake version and ``vm.mmap_rnd_bits`` in sanitizer workflows.
#. Only run PRs CI when review is requested.
#. Refactor Github CI sanitizer related jobs.
#. Build ``ShapesDemo`` on Ubuntu Github CI.
#. Fix Python version and environment.
#. Add DNS entries to hosts files on Github workflows.
#. Build Fast DDS Python bindings in Fast DDS Docs Github CI job.

This release includes the following **fixes**:

#. Fix and refactor Windows Github CI.
#. Fix wrong log info messages on TCP.
#. Add a keyed fragmented change to the reader data instance only when complete.
#. Prevent index overflow and correctly assert the end iterator in DataSharing.
#. Fix the shared memory cleaning script.
#. Fix CI documentation workflow label triggering.
#. Add missing virtual destructor for ``StatisticsAncillary``.
#. Migrate apt package installation action to ``eProsima-CI``.
#. Add missing ``TypeLookup`` listeners.
#. Fix doxygen docs warnings. Prepare for compiling with ``Doxygen 1.10.0``.
#. Upgrade dependency version to last patch version in ``.repos`` file.
#. Fix TCP reconnection after open logical port failure.
#. Avoid unhandled asio exceptions.
#. Fix ``CVE-2024-28231``.
#. Fix data race on PDP.
#. Fix flaky Log tests.
#. Fix some flaky MacOS tests.
#. Fix hidden overloaded virtual methods.
#. Fix test filtering in CMake files.
#. Avoid first message loss in TCP.
#. Fix ``CVE-2024-30258 / CVE-2024-30259``.
#. Enforce SHM ports open mode exclusions.
#. Force unlimited :ref:`ResourceLimits <resourcelimitsqospolicy>` if lower or equal to zero.
#. Removed warning in ``ParameterList``.
#. Make :cpp:func:`DataReader::get_first_untaken_info()<eprosima::fastdds::dds::DataReader::get_first_untaken_info>` coherent with ``read()/take()``.
#. Fix leak in ``SecurityManager``.
#. Fix support for ``@key`` annotation in ``DynamicTypes``.
#. Fix leaks in XML parser for ``DynamicTypes``.
#. Fix Discovery Server over TCP.
#. Handle errors when setting socket buffer sizes.
#. Fix :cpp:func:`on_sample_lost<eprosima::fastdds::dds::DataReaderListener::on_sample_lost>` notification on best-effort readers for fragmented samples.
#. Fix DataSharing QoS deserialization.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.10.3.rst
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
.. include:: previous_versions/v2.6.8.rst
.. include:: previous_versions/v2.6.7.rst
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
