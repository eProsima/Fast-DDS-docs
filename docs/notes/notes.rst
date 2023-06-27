.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.1
==============

This minor release includes several improvements and bugfixes.

.. note::
  Mind that, even though this release is API compatible with previous v2.x versions, it is *NOT* ABI compatible with
  previous versions.
  This means that applications upgrading Fast DDS to v2.11.0 will require recompilation, though not source code
  modification.

.. note::
  It is also advisable to regenerate the type support from the IDL files using
  `Fast DDS-Gen v2.5.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.5.1>`_.
  Furthermore, if upgrading to v2.11.0, it is also recommended to upgrade Fast CDR to
  `v1.1.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.1.0>`_.

This release includes the following **features**:

1. :ref:`Ignore every local endpoint within the DomainParticipant preventing local matching <property_ignore_local_endpoints>`.
2. Extend DynamicDataHelper API providing a ``print`` overload with ``std::ostream`` as parameter.
3. :ref:`typelookup_config`.
4. Static Discovery XSD Schema.

This release includes the following **improvements**:

1. **Fast DDS improvements**
    1. Assign minimum available participant ID to new participants.
    2. Export symbols correctly on ContentFilteredTopic.
    3. Improve content filter expression parameters check and verbosity.
    4. Check TCP headers endianness.
    5. Security module: distinguished names (DN) comparison.
2. **Fast DDS deprecation**
    1. DDS:Crypto:AES-GCM-GMAC configuration using Property Policy QoS (security vulnerability).
3. **CI improvements**
    1. Include BitmapRange unit tests.
    2. Support for running some tests in parallel.
    3. Windows workflow.
4. **Build system**
    1. Improve CMake target loading. Removal of ``FASTDDS_STATIC`` CMake option.
    2. Avoid auto-linkage using CMake.
5. **Dependencies**
    1. Upgrade internal type supports using latest Fast DDS-Gen release v2.5.1.
    2. Upgrade Fast CDR submodule to v1.1.0.
6. **Examples**
    1. Admit XML configuration files in AdvanceConfigurationExample.
    2. New Discovery Server example.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
    1. Fix crash when creating two participants with the same fixed participant ID.
    2. Fix crash when calling
       :cpp:func:`on_requested_deadline_missed() <eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`
       callback.
    3. Fix crashes caused by not capturing every Fast CDR exception.
    4. Correctly resolve aliases in DDSSQLFilter.
    5. Wait for log background thread initialization on the first queued entry.
    6. Fix data race when accessing ``WRITE_PARAM_DEFAULT`` static variable.
    7. Fix partition copy in QoS.
    8. Fix Data-Sharing delivery when data_count is zero (#3065)
    9. Fix API Fast DDS v2.10.0 API break calling correctly
       :cpp:func:`on_participant_discovery() <eprosima::fastdds::dds::DomainParticipantListener::on_participant_discovery>`
       callbacks.
    10. Security module: Honor :ref:`allow_unauthenticated_section` flag.
    11. Fix concurrent access to
        :cpp:func:`load_profiles() <eprosima::fastdds::dds::DomainParticipantFactory::load_profiles>`.
    12. Fix UBSan (Undefined Behavior Sanitizer) issues.
    13. Improve Doxygen documentation about DomainParticipantListener discovery callbacks.
2. XSD fixes
    1. Set TransportDescriptor kind parameter as optional.
    2. Correctly assign QoS to the proper endpoint.
    3. Add missing tags.
3. CI fixes
    1. Fix null dereference in fuzzer code.
    2. Limit Thread Sanitizer memory usage to prevent runner shutdown.
    3. Use correct time unit in latency tests.
    4. Run communication tests.
4. Examples
    1. Correct DDS entity deletion order.
5. Installer generation
    1. Add documentation fallback when the documentation tag is not found.
6. Repository
    1. Remove 2.1.x as active branch.
    2. Remove 2.8.x as active branch.
7. Non Tier 1 support
    1. Fix build on MSVC 19.36.
    2. Forward compatibility with Boost interprocess 1.74+.
    3. Include missing header files required for compiling with GCC 13.
    4. QNX build fixes.
    5. Fix build issues in RPM systems.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.10.1.rst
.. include:: previous_versions/v2.10.0.rst
.. include:: previous_versions/v2.9.1.rst
.. include:: previous_versions/v2.9.0.rst
.. include:: previous_versions/v2.8.2.rst
.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
.. include:: previous_versions/v2.7.2.rst
.. include:: previous_versions/v2.7.1.rst
.. include:: previous_versions/v2.7.0.rst
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
