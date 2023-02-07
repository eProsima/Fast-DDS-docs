.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _xml_profiles:

XML profiles
============

*eProsima Fast DDS* allows for loading XML configuration files, each one containing one or more XML profiles.
In addition to the API functions for loading user XML files, *Fast DDS* tries to locate and load several XML files
upon initialization.
*Fast DDS* offers the following options:

*   Load an XML file named *DEFAULT_FASTRTPS_PROFILES.xml* located in the current execution path.
*   Load an XML file which location is defined using the environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE``
    (see :ref:`env_vars_fastrtps_default_profiles_file`).
*   Load the configuration parameters directly from the classes' definitions without looking for the
    *DEFAULT_FASTRTPS_PROFILES.xml* in the working directory (see :ref:`env_vars_skip_default_xml`).
*   Load directly the XML as a string data buffer.

An XML profile is defined by a unique name that is used to reference the XML profile
during the creation of an |Entity|, the :ref:`Trasport <comm-transports-configuration>` configuration, or the
:ref:`DynamicTypes <dds_layer_topic_dynamic_data_types>` definition.

Both options can be complemented, i.e. it is possible to load multiple XML files but these must not have XML profiles
with the same name.
This section explains how to configure DDS entities using XML profiles.
This includes the description of all the configuration values available for each of the XML profiles, as well as how
to create complete XML files.

.. toctree::
    :maxdepth: 2

    /fastdds/xml_configuration/making_xml_profiles
    /fastdds/xml_configuration/domainparticipant
    /fastdds/xml_configuration/datawriter
    /fastdds/xml_configuration/datareader
    /fastdds/xml_configuration/library_settings
    /fastdds/xml_configuration/transports
    /fastdds/xml_configuration/log
    /fastdds/xml_configuration/dynamic_types
    /fastdds/xml_configuration/common
    /fastdds/xml_configuration/example
