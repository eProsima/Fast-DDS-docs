.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.12.1
==============

This release includes the following **improvements**:

1. Support for linking with Fast CDR v1.
2. The period for the timer within the :ref:`disablepositiveacksqospolicy` is now updatable.
3. Log error message upon receiver resource creation failure.
4. CI and repository improvements.
5. Simplify code in CDRMessage.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**

    1. Fix transient local durability for reliable readers using intra-process and data-sharing.
    2. Use STL implementation of Timed/RecursiveTimedMutex when MSVC >= 19.36.
    3. Fix updatability of immutable DataWriterQos.
    4. Fix the clang build for clang 14.
    5. Fix remote locators filtering when whitelist provided.
    6. Fix Data Race when updating liveliness changed in WLP.
    7. Add XML parser bit_bound bounds check.
    8. Fix missing mandatory attribute check in XML parser struct type.
    9. SHM transport: ignore non-existing segment on pop.
    10. Fix: mac address overflow on Windows.

2. CI fixes:

    1. Fix flow controllers unit tests compilation when using Fast CDR from thirdparty.
    2. PubSubAsReliable test fix.
    3. FileWatchTest fix for github windows CI.

.. note::
  When upgrading to version 2.12.1 it is **advisable** to regenerate generated source from IDL files
   using `Fast DDS-Gen v3.1.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.1.0>`_.

Previous versions
=================

.. include:: previous_versions/v2.12.0.rst
.. include:: previous_versions/v2.11.2.rst
.. include:: previous_versions/v2.11.1.rst
.. include:: previous_versions/v2.11.0.rst
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
