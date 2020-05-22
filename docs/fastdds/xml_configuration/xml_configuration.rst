.. _xml_profiles:

XML profiles
============

This section shows how to configure entity attributes using XML profiles, explaining each field with its available
values and how to compound the complete XML files.

*eProsima Fast DDS* permits to load several XML files, each one containing XML profiles.
In addition to the API functions to load user XML files, at initialization *eProsima Fast DDS* tries to locate and load
several default XML files.
*eProsima Fast DDS* offers the following options to use default XML files:

* Using an XML file with the name *DEFAULT_FASTRTPS_PROFILES.xml* and located in the current execution path.
* Using an XML file which location is defined in the environment variable *FASTRTPS_DEFAULT_PROFILES_FILE*.

An XML profile is defined by a unique name (or ``<transport_id>`` label
in the :ref:`transportdescriptors` case) that is used to reference the XML profile
during the creation of a Fast DDS entity, :ref:`comm-transports-configuration`, or :ref:`dynamic-types`.


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
