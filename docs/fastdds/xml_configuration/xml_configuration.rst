.. include:: includes/aliases.rst

.. _xml_profiles:

XML profiles
============

This section explains how to configure DDS |Entities| using XML profiles.
This includes the description of all the configuration values available for each of the XML profiles, as well as how
to create complete XML files.

*eProsima Fast DDS* permits to load several XML files, each one containing one or more XML profiles.
In addition to the API functions for loading user XML files, at initialization Fast DDS tries to locate and load
several default XML files.
Fast DDS offers the following options to use default XML files:

* Load an XML file named *DEFAULT_FASTRTPS_PROFILES.xml* located in the current execution path.
* Load an XML file which location is defined using the environment variable *FASTRTPS_DEFAULT_PROFILES_FILE*.

An XML profile is defined by a unique name, or ``<transport_id>`` label
in the :ref:`transportdescriptors` case, that is used to reference the XML profile
during the creation of an |Entity|, the :ref:`Trasports <comm-transports-configuration>` configuration, or the
:ref:`DynamicTypes <dds_layer_topic_dynamic_data_types>` definition.


.. toctree::

    /fastdds/xml_configuration/making_xml_profiles
    /fastdds/xml_configuration/library_settings
    /fastdds/xml_configuration/transports
    /fastdds/xml_configuration/dynamic_types
    /fastdds/xml_configuration/participant
    /fastdds/xml_configuration/publisher
    /fastdds/xml_configuration/subscriber
    /fastdds/xml_configuration/common
    /fastdds/xml_configuration/example
