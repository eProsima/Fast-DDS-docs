.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.7 (EOL)
====================

This release includes the following **fixes**:

#. Fix `CVE-2025-24807 <https://www.cve.org/CVERecord?id=CVE-2025-24807>`_
#. Release ``participant_stateless`` secure builtin writer history change when authentication has finished
#. Improve ``OpenSSL`` lifecycle handling
#. Fix comparison in ``is_update_allowed``
#. Arithmetic overflow in fragment size calculations
#. Fix ``-Werror=template-id-cdtor``
#. Fix double-locking issue in ``DataSharingListener``
#. Improve ``PDPClient`` initialization
#. Fix unique network flows with TCP transports
#. Decouple transport receivers creation using unique network flows
#. Fix EDP reliability timings
#. Fix ``several_writers_on_unack_sample_removed`` flaky test
#. Reliable volatile change dropped when all history acked
#. Create initial connection for TCP initial peers
#. Filter interested readers on PDP writer
#. Fix cleanup race condition with exclusive and shared lock files
#. Fix TCP discovery server locators translation
#. Use correct algorithm strings on ``PermissionsToken`` and ``IdentityToken``
#. Fix tsan potential deadlock between ``StatefulWriter`` and ``FlowController``
#. Fix log category name macro collision in ``MacOS``
#. Unacknowledged sample removed in KeepAll mode
#. Fix assertion on ``OutputTrafficManager``

This release includes the following **improvements**:

#. Log any errors before ``cancel_init``
#. Improve ``max_allocations`` calculation on SHM transport
#. New property to select preferred key agreement algorithm
#. Update reception timestamp when it is added to the instance
#. Improve Blackbox TCP tests suite
#. Handle socket buffer size setting when system's maximum exceeded
#. Address some compilation warnings with GCC latest
#. Refactor builtin writers & readers creation
#. Update sqlite from 3.36.0 to 3.47.2
#. Regenerate code with Fast DDS Gen v2.5.3

Github CI management:

#. Force Asio thirdparty in MacOS CI
#. Get correct Fast CDR related branch in CI
#. Update submodules when cloning Fast DDS on CI

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.10.6.rst
.. include:: previous_versions/v2.10.5.rst
.. include:: previous_versions/v2.10.4.rst
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
.. include:: previous_versions/v2.6.10.rst
.. include:: previous_versions/v2.6.9.rst
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
