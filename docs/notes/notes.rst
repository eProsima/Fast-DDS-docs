.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.10.0
==============

This minor release includes several new features, improvements and bugfixes.

.. note::
    Mind that, even though this release is API compatible with previous v2.x versions, it is *NOT* ABI compatible with
    previous versions.
    This means that applications upgrading Fast DDS to v2.10.0 will require recompilation, though not source code
    modification.

.. note::
    It is also advisable to regenerate the type support from the IDL files using
    `Fast DDS-Gen v2.4.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.4.0>`_.
    Furthermore, if upgrading to v2.10.0, it is also recommended to upgrade Fast CDR to
    `v1.0.27 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.27>`_.

This release includes the following **features**:

1. New :ref:`dds_layer_publisher_dataWriterListener_on_unack_sample_removed` in
   :cpp:class:`DataWriterListener <eprosima::fastdds::dds::DataWriterListener>`.
2. :ref:`Secure Discovery Server <DS_security>`.
3. DomainParticipant ignore empty API.
4. RTPS :cpp:func:`ReaderListener::on_incompatible_type <eprosima::fastrtps::rtps::ReaderListener::on_incompatible_type>`
   and :cpp:func:`WriterListener::on_incompatible_type <eprosima::fastrtps::rtps::WriterListener::on_incompatible_type>`
   empty API.

This release includes the following **improvements**:

1. **Fast DDS improvements**
    1. Improve behavior when ``STRICT_REALTIME`` :ref:`CMake option <cmake_options>` is not enabled.
    2. Using functors for ``for_matched_readers`` parameter.
    3. Improve auto GAPs in Data Sharing.
    4. Use standard value for ``PID_RELATED_SAMPLE_IDENTITY``.
2. **Contributions and repository quality**
    1. Update Pull Request template.
    2. Update foonathan_memory quality declaration.
    3. Update XSD schema.
    4. Make netwrok headers private avoiding exposing non-public API.
    5. Improve Doxygen documentation for
       :cpp:class:`ResourceLimitsQosPolicy <eprosima::fastdds::dds::ResourceLimitsQosPolicy>`.
3. **Examples**
    1. New :ref:`Request-Reply example <use-case-request-reply>`.
4. **CI improvements**
    1. New workflow to check documentation build.
    2. ASAN workflow updated to use Ubuntu 22.04.
5. **Dependencies**
    1. Upgrade internal type supports using latest Fast DDS-Gen release v2.4.0.
    2. Upgrade Fast CDR submodule to v1.0.27.
6. **Fast DDS CLI**
    1. Handle ``SIGTERM`` signal.
7. **Community supported platforms**
    1. :ref:`QNX 7.1 build infrastructure <qnx_sources>`.

This release includes the following **fixes**:

1. **Security vulnerability**
    1. Fix chain of trust issues with a single CA certificate.
2. **Bugfixes**
    1. Fix RTPS StatelessWriter ACK check.
    2. ASAN (Address Sanitizer) fixes.
    3. UBSan (Undefined Behavior Sanitizer) fixes.
    4. Export public API correctly in Windows.
    5. Correctly handle builtin endpoints mask.
    6. Fix backwards compatibility using SHM communication.
    7. Protect against uncaught exception in SHM segment creation.
    8. Fix build for GCC 5.
    9. Validity check for first sequence number.
    10. Fix crash when enabling DisablePositiveACKsQoSPolicy with remote best-effort readers.
3. **Synchronization fixes**
    1. Take mutex when removing local reader in WLP.
    2. Fix data races in SecurityManager authentication process.
4. CI fixes
    1. Fix test building when using ``GTEST_INDIVIDUAL`` :ref:`CMake option <cmake_options>`.
    2. Fix overflow in received samples in performance tests.
5. Example fixes
    1. Avoid creating entities within callbacks in DynamicHelloWorldExample.
6. Repository fixes
    1. Remove 2.7.x as active branch.
7. Community supported platforms
    1. Include right header when building for iOS.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.9.1.rst
.. include:: previous_versions/v2.9.0.rst
.. include:: previous_versions/v2.8.1.rst
.. include:: previous_versions/v2.8.0.rst
.. include:: previous_versions/v2.7.2.rst
.. include:: previous_versions/v2.7.1.rst
.. include:: previous_versions/v2.7.0.rst
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
