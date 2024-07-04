.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.9
=============

.. important::
  According to our
  `release support guidelines <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_
  Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
  From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

This release includes the following **features**:

#. Add XML configuration for FlowControllerDescriptor to 2.x
#. New `max_message_size` property to limit output datagrams size

This release includes the following **improvements**:

#. Update Fast CDR thirdparty submodule
#. Consider library behavior changes as ABI breaks in the PR template checklist
#. Allow processing of AckNack submessages with count == 0
#. Use `%*` instead of loop in `.bat` scripts.
#. Use absolute paths when loading XML files
#. TCPSendResources cleanup

Github CI management:

#. Fix Python Installation version in Github CI. Address failing system tests environment issues
#. Set fallback branch for `get_related_branch_from_repo` correctly
#. Fix sanitizers CI test summary report
#. Protect asio exception
#. Set Fallback branch to 2.6.x
#. Run selected VS tool on Windows CI
#. Add DNS entries to hosts files on Github workflows
#. Refactor Fast DDS Ubuntu CI to include several tests
#. Avoid `CCache` in workflows and nighties
#. Update README.md with GitHub actions Ubuntu CI nightly
#. Label flaky tests with `xfail`

This release includes the following **fixes**:

#. Fix leak in `SecurityManager::participant_volatile_message_secure_writer_`
#. Fix Discovery Server over TCP
#. Fix some leaks in XML DynamicTypes Parser
#. Correct liveliness state in a multiple reader - one writer scenario
#. Fix support for `@key` annotation in Dynamic types
#. Properly delete builtin statistics writers upon `delete_contained_entities()`
#. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API
#. Set DataSharing in `Writer|ReaderProxyData`
#. Only apply content filter to ALIVE changes
#. Handle errors when setting socket buffer sizes
#. Automatically unmatch remote participants on participant deletion
#. Fix on_sample_lost notification on best-effort readers for fragmented samples
#. Handle errors when setting socket buffer sizes
#. Fix DS servers not connecting due to ports logic
#. Manual fix for documentation generation
#. Create `InitialConnection` for TCP initial peers

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.6.8.rst
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
