.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_xml_profiles_questions:

XML PROFILES Frequently Asked Questions
=======================================

.. collapse::  How are XML profiles defined?

    |br|

    The XML profiles are defined within the ``<dds>`` element, and in turn, within the ``<profiles>`` XML elements. For further information, see :ref:`making_xml_profiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens if an XML file is loaded multiple times?

    |br|

    If the same XML profile file is loaded multiple times, the second loading of the file will result in an error together with the consequent error log. For further information, see :ref:`loadingapplyingprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is stand-alone profiles definition?

    |br|

    The element defining the XML profile is the root element of the XML file. For further information, see :ref:`rootedvsstandalone`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is rooted profile definition?

    |br|

    The element defining the XML profile is the child element of another element. For further information, see :ref:`rootedvsstandalone`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What do DomainParticipant profiles do?

    |br|

    The DomainParticipant profiles allow the definition of the configuration of DomainParticipants through XML files. These profiles are defined within the ``<participant>`` XML tags. For further information, see :ref:`participantprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the domain participant XML attributes?

    |br|

    The ``<participant>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``. For further information, see :ref:`domainparticipantattributes`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is the DomainParticipant configured?

    |br|

    The ``<participant>`` element has two child elements: ``<domainId>`` and ``<rtps>.`` All the DomainParticipant configuration options belong to the ``<rtps>`` element, except for the DDS DomainId which is defined by the ``<domainId>`` element. For further information, see :ref:`domainparticipantconfig`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What do DataWriter profiles do?

    |br|

    The DataWriter profiles allow for configuring DataWriters from an XML file. These profiles are defined within the ``<data_writer>`` XML tags. For further information, see :ref:`publisherprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the XML attributes of the DataWriter?

    |br|

    The ``<data_writer>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``. For further information, see :ref:`publisherprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What do DataReader profiles do?

    |br|

    The DataReader profiles allow declaring DataReaders from an XML file. These profiles are defined within the ``<data_reader>`` XML tags. For further information, see :ref:`subscriberprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the XML attributes of the DataReader?

    |br|

    The ``<data_reader>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``. For further information, see :ref:`subscriberprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What do Topic profiles do?

    |br|

    The ``<topic>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``. For further information, see :ref:`Topic`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the available XML elements for configuring transport layer parameters in Fast DDS, and what is their purpose?

    |br|

    This section defines the XML elements available for configuring the transport layer parameters in Fast DDS. These elements are defined within the XML tag ``<transports_descriptors>``. The ``<transport_descriptors>`` can contain one or more ``<transport_descriptor>`` XML elements. Each ``<transport_descriptor>`` element defines a configuration for a specific type of transport protocol. For further information, see :ref:`transportdescriptors`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Are log profiles available?

    |br|

    eProsima Fast DDS allows for registering and configuring Log consumers using XML configuration files. The logging profiles are defined within the ``<log>`` XML tags. The <log> element has two child elements: ``<use_default>`` and ``<consumer>``. For further information, see :ref:`logprofiles`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the advantage of having Dynamic Language Binding support in XML files?

    |br|

    The topic data types can be modified without the need to modify the source code of the DDS application. For further information, see :ref:`xmldynamictypes`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How are Dynamic Types configured with XML?

    |br|

    Dynamic Types in Fast DDS are configured using XML files, where data types are defined within <types> elements. Each <types> element can contain one or more <type> definitions. Members of these types, such as primitive or complex types like structures, unions, sequences, arrays, and maps, are specified using the <member> tag along with appropriate attributes for the type and its characteristics. Once the XML file is created, it can be loaded into the Fast DDS application using the load_XML_profiles_file() function, allowing the types to be dynamically used without modifying the application’s source code. For further information, see :ref:`xmldynamictypes`.

|
