.. include:: ../03-exports/aliases-api.include

.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.12.0
==============

.. note::

  This release upgrades the following Fast DDS dependencies:

  * `Fast CDR v2.0.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.0.0>`_
  * `Fast DDS-Gen v3.0.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.1>`_

  Please, read also the release notes of
  `Fast DDS-Gen v3.0.0 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.0>`_ to be aware of every possible
  break in the application code.

  As Fast DDS dependencies have been upgraded to new major releases, depending on the types defined in the IDL files,
  it might be required to modify the user application source code besides recompiling it (more information can be found
  in the corresponding release notes).

.. note::

  There is a minor API break with previous v2.x versions: ``MEMBER_INVALID`` identifier was declared using ``#define``.
  In order to prevent polluting the user workspace, it has been transformed into a ``constexpr`` within
  ``eprosima::fastrtps::types`` namespace.

This release includes the following **features**:

1. :ref:`New participant property <property_policies_shm_enforce_metatraffic>` to configure SHM
   transport metatraffic behavior.
2. Exposed custom payload pool on DDS :ref:`DataWriter <dds_layer_publisher_datawriter_with_payload_pool_creation>` and
   :ref:`DataReader <dds_layer_subscriber_datareader_with_payload_pool_creation>` declaration.

    1. Feature example.

3. :ref:`Processing environment variables in XML text <xml_environment_variables>`.
4. Dependencies

    1. Upgrade internal type supports using latest Fast DDS-Gen release v3.0.0.
       This release introduces the following features:

        1. `Support for @optional builtin annotation <optional_members>`.
        2. `Support for @extensibility builtin annotation <extensibility>`.

    2. Upgrade Fast CDR submodule to v2.0.0 introducing XCDR encoding version 2.

This release includes the following **improvements**:

1. ``fixed_string`` comparison operators.
2. Remove mutex from `TimedEventImpl` (#3745, #3760)
3. Performance improvements on intraprocess and datasharing.
4. Improve Shared Memory resilience to crashing participants.
5. Improve scripts shebang portability.
6. Use ``foonathan_memory`` to reduce allocations in SharedMemManager.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
    1. Fixed XMLParser null-dereference when parsing log configuration.
    2. Allow participant XML profiles with no ``<rtps>`` tag.
    3. Fix encapsulation format in Writer Liveliness Protocol.
    4. Fix :cpp:func:`DomainParticipant::register_remote_type<eprosima::fastdds::dds::DomainParticipant::register_remote_type>`
       return when negotiating type.
    5. Fix strict real-time feature when using Flow Controller feature.
    6. Fix ParameterPropertyList increment operators.
    7. Fix bad-free when receiving malformed DATA submessage.
    8. Fix asymmetric whitelist matching.
    9. Fix heap-use-after-free on XMLElementParser.
    10. Fix History remove change return statement.
2. CI fixes
    1. Fix RemoteBuiltinEndpointHonoring blackbox test.
    2. Improve repository workflows.
    3. Use `FASTRTPS_NO_LIB` on unittest root folder.
    4. Fix Windows workflow.
3. Tools
    1. Remove C++11 check in ``fastdds-discovery-server`` CLI tool.
4. Examples
    1. Fix HelloWorldDataSharing data type.
5. Documentation
    1. Doxygen typos.
6. Repository
    1. Remove 2.9.x as active branch.
7. Non Tier 1 support
    1. Fixed SHM in 32-bit architectures.
    2. Fix warning on Win32 architecture.

.. note::
  Upgrading to version 2.12.0 **requires** to regenerate generated source from IDL files using
  `Fast DDS-Gen v3.0.1 <https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.1>`_.

Previous versions
=================

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
