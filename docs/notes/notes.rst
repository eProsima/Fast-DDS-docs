.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.8
=============

This release includes the following **features**:

#. :ref:`Authentication Handshake Properties <property_policies_security>` documentation.
#. TCP Client and Server Participant Decision Making.
#. Add macOS and Ubuntu Github CI.

This release includes the following **improvements**:

#. Make DataWriters always send the key hash on keyed topics.
#. Include variety of terminate process signals handler in discovery server.
#. Pin CMake version and ``vm.mmap_rnd_bits`` in sanitizer workflows.
#. Effectively assert ``AUTOMATIC/MANUAL_BY_PARTICIPANT`` liveliness.
#. Pick smallest available participant ID for new participants.
#. Build Fast DDS Python bindings in Fast DDS Docs Github CI job.
#. Check History QoS inconsistencies.
#. Add check for XML API to PR template.
#. ``LARGE_DATA`` Participants logic with same listening ports.

TCP transport improvements:

#. TCP unique client announced local port.
#. Remove unnecessary TCP warning and Fix some tests.
#. TCP ``non-blocking`` send.
#. Enabling multiple interfaces through whitelist in TCP servers.

Github CI management:

#. Refactor Github CI sanitizer related jobs.
#. Avoid running GitHub CI if PR has conflicts.
#. Add manual Ubuntu Github CI.
#. Improve CI version management.
#. Build ``ShapesDemo`` on Ubuntu Github CI.
#. Only run PRs CI when review is requested.

This release includes the following **fixes**:

#. Fix and refactor Windows Github CI.
#. Fix max clash with Windows CI.
#. Fix the shared memory cleaning script.
#. Fix doxygen docs warnings. Prepare for compiling with ``Doxygen 1.10.0``.
#. Prevent index overflow and correctly assert the end iterator in DataSharing.
#. Add a keyed fragmented change to the reader data instance only when its completed.
#. Add missing virtual destructor for ``StatisticsAncillary``.
#. Fix wrong log info messages on TCP.
#. Migrate apt package installation action to ``eProsima-CI``.
#. Fix CI documentation workflow label triggering.
#. Upgrade dependency version to last patch version in ``.repos`` file.
#. Fix ``CVE-2024-28231``
#. Fix data race on PDP.
#. Discard already processed samples on PDPListener.
#. Fix flaky Log tests.
#. Add missing ``TypeLookup`` listeners.
#. Fix hidden overloaded virtual methods.
#. Fix TCP reconnection after open logical port failure.
#. Fix ``CVE-2024-30258 / CVE-2024-30259``
#. Make :cpp:func:`DataReader::get_first_untaken_info()<eprosima::fastdds::dds::DataReader::get_first_untaken_info>` coherent with ``read()/take()``.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.6.7.rst
.. include:: previous_versions/v2.6.6.rst
.. include:: previous_versions/v2.6.5.rst
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
