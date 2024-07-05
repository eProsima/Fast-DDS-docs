.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.13.5
==============

This release includes the following **features** in an ABI compatible manner:

#. New :ref:`property_max_message_size` property to limit output datagrams size

This release includes the following **improvements**:

#. Improve ThreadSettingsQoS logging
#. Allow processing of AckNack submessages with ``count == 0``
#. Internal refactor on port handling
#. Do not require ``PYTHON_VERSION`` to be defined in .bat files
#. Use ``%*`` instead of loop in ``.bat`` scripts
#. Consider library behavior changes as ABI breaks in the PR template checklist
#. Refactor IStatusQueryable and make monitor service interfaces private
#. Automatically unmatch remote participants on participant deletion
#. Handle errors when setting socket buffer sizes
#. Github CI management:

    #. Refactor Github CI sanitizer related jobs
    #. Build Fast DDS Python bindings in Fast DDS Docs Github CI job
    #. Build ShapesDemo on Ubuntu Github CI
    #. Fix Python Installation version in Github CI. Address failing system tests environment issues.
    #. Fix sanitizers CI test summary report
    #. Run selected VS tool on Windows CI
    #. Increase sleep to miss the deadline in macOS flaky tests
    #. Fix ShmTransport buffer recovery MacOS flaky test
    #. Set fallback branch for ``get_related_branch_from_repo`` correctly
    #. Add DNS entries to hosts files on github workflows

This release includes the following **fixes**:

#. Add check for XML API to PR template
#. Use absolute paths when loading XML files
#. Fix some leaks in XML DynamicTypes Parser
#. Force unlimited ResourceLimits if lower or equal to zero
#. Enforce SHM ports open mode exclusions
#. Run ``is_plain`` method with the corresponding data representation
#. Removed warning
#. Don't require Fast CDR v2 in examples
#. Make reader ``get_first_untaken_info()`` coherent with ``read()`` / ``take()``
#. Fix leak in ``SecurityManager::participant_volatile_message_secure_writer_``
#. Fix CVE-2024-30258 / CVE-2024-30259
#. Fix support for ``@key`` annotation in Dynamic types
#. Set DataSharing in Writer|ReaderProxyData
#. Fix on_sample_lost notification on best-effort readers for fragmented samples
#. Correct liveliness state in a multiple reader - one writer scenario

.. note::
  When upgrading to version 2.13.5 it is **advisable** to regenerate generated source from IDL files
  using `Fast DDS-Gen v3.2.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1>`_.

Previous versions
=================

.. include:: previous_versions/v2.13.4.rst
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
