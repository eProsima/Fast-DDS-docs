.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.5
==============

This release includes the following **features** in an ABI compatible manner:

#. Add XML configuration for :ref:`FlowControllerDescriptor <flowcontrollers_xml>`

This release includes the following **improvements**:

#. Use ``%*`` instead of loop in ``.bat`` scripts
#. Documentation improvements
#. Use absolute paths when loading XML files
#. Reduce tests flakiness
#. Add unsigned specification to literals

Github CI management:

#. Set fallback branch for ``get_related_branch_from_repo`` correctly
#. Fix sanitizers CI test summary report
#. Run selected VS tool on Windows CI
#. Use token for CCache action
#. Refactor Fast DDS Ubuntu CI to include several tests
#. Avoid CCache in some Github workflows
#. Build profiling tests as alternate build in Ubuntu CI
#. Add Ubuntu weekly CI
#. Fix python version in sanitizers CI
#. Fix windows CI and add vanilla build step in Ubuntu CI
#. Use eProsima-CI action to install Qt
#. Update types regeneration script homing path

This release includes the following **fixes**:

#. Automatically unmatch remote participants on participant deletion
#. Only apply content filter to ALIVE changes
#. Fix liveliness state in a multiple reader - one writer scenario
#. Fix topic interference on ``liveliness_changed`` status
#. Fix DS servers not connecting due to ports logic
#. Fix issue with exclusive ownership and unordered samples
#. Fix SecurityManager memory issue
#. Correctly initialize ``MatchingFailureMask`` constants to be used with the ``std::bitset`` API
#. Fix data race in ``TypeObjectFactory::get_instance``
#. Properly delete builtin statistics writers upon ``delete_contained_entities()``
#. Fix secure simple participants with initial peers not matching over TCP
#. Fix access violations on XML parser detected by OSS-fuzz

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

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
