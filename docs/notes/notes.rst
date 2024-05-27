.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.13.4
==============

This release includes the following **features** in an ABI compatible manner:

#. Expose :ref:`Authentication Handshake Properties<property_policies_security>`

This release includes the following **improvements**:

#. Monitor service properly managing instances
#. Effectively assert ``AUTOMATIC`` / ``MANUAL_BY_PARTICIPANT`` liveliness
#. Add catch of out-of-range exception for thread settings port
#. TCP transport improvements:

    #. ``TCPSendResources`` cleanup
    #. TCP first message loss
    #. Set real TCP ``non_blocking_send`` limitation

This release includes the following **fixes**:

#. Fix hidden overloaded virtual methods
#. Fix Discovery Server over TCP using logical port
#. Protect asio exception fix
#. Fix flaky Log tests
#. Fix CVE-2024-28231
#. Add missing virtual destructor for StatisticsAncillary
#. Increase ack waiting time in ``reliable_on_unack_sample_removed``
#. Fix versions in fastrtps.repos
#. GitHub CI fixes:

    #. Fix CI version management
    #. Add manual Ubuntu Github CI
    #. Avoid running GitHub CI if PR has conflicts
    #. Migrate apt package installation action to eProsima-CI
    #. Only run PRs CI when review requested
    #. Pin CMake version and ``vm.mmap_rnd_bits`` in sanitizer workflows
    #. Improve filtering of DNS tests

.. note::
  When upgrading to version 2.13.4 it is **advisable** to regenerate generated source from IDL files
  using `Fast DDS-Gen v3.2.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1>`_.

Previous versions
=================

.. include:: previous_versions/v2.13.3.rst
.. include:: previous_versions/v2.13.2.rst
.. include:: previous_versions/v2.13.1.rst
.. include:: previous_versions/v2.13.0.rst
.. include:: previous_versions/v2.12.1.rst
.. include:: previous_versions/v2.12.0.rst
.. include:: previous_versions/v2.11.3.rst
.. include:: previous_versions/v2.11.2.rst
.. include:: previous_versions/v2.11.1.rst
.. include:: previous_versions/v2.11.0.rst
.. include:: previous_versions/v2.10.4.rst
.. include:: previous_versions/v2.10.3.rst
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
