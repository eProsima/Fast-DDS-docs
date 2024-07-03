.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.9
=============

This release includes the following **features**:

#. Add XML configuration for FlowControllerDescriptor to 2.x (#4907)
#. New `max_message_size` property to limit output datagrams size (#4899)

This release includes the following **improvements**:

#. Update Fast CDR thirdparty submodule (#4733)
#. Consider library behavior changes as ABI breaks in the PR template checklist (#4784)
#. Allow processing of AckNack submessages with count == 0 (#4774)
#. Use `%*` instead of loop in `.bat` scripts. (#4821)
#. Use absolute paths when loading XML files (#4831)
#. TCPSendResources cleanup (#4513)

Github CI management:

#. Fix Python Installation version in Github CI. Address failing system tests environment issues (#4766)
#. Set fallback branch for get_related_branch_from_repo correctly (#4847)
#. Fix sanitizers CI test summary report (#4841)
#. Protect asio exception hotfix (#4533)
#. Set Fallback branch to 2.6.x (#4870)
#. Run selected VS tool on Windows CI (#4868)
#. Add DNS entries to hosts files on github workflows (#4811)
#. Refactor Fast DDS Ubuntu CI to include several tests (#4957)
#. CI - Avoid CCache in workflows and nightlies (#4976)
#. Update README.md with GitHub actions Ubuntu CI nightly (#4983)

This release includes the following **fixes**:

#. Fix leak in `SecurityManager::participant_volatile_message_secure_writer_` (#4726)
#. Fix Discovery Server over TCP (#4656)
#. Fix some leaks in XML DynamicTypes Parser (#4763)
#. Correct liveliness state in a multiple reader - one writer scenario (#4884)
#. Fix support for `@key` annotation in Dynamic types (#4749)
#. Properly delete builtin statistics writers upon `delete_contained_entities()` (#4917)
#. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API (#4928)
#. Set DataSharing in Writer|ReaderProxyData (#4804)
#. Only apply content filter to ALIVE changes (#4904)
#. Handle errors when setting socket buffer sizes (#4825)
#. Automatically unmatch remote participants on participant deletion (#4865)
#. Fix on_sample_lost notification on best-effort readers for fragmented samples (#4607)
#. Handle errors when setting socket buffer sizes (#4852)
#. Fix DS servers not connecting due to ports logic (#4952)
#. Manual fix for documentation generation (#5013)

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
