.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _making_xml_profiles:

Creating an XML profiles file
-----------------------------

An XML file can contain several XML profiles.
These XML profiles are defined within the ``<dds>`` element, and in turn, within the ``<profiles>`` XML elements.
The possible topologies for the definition of XML profiles are specified in :ref:`rootedvsstandalone`.
The available profile types are:

* :ref:`participantprofiles`,
* :ref:`publisherprofiles`,
* :ref:`subscriberprofiles`,
* :ref:`transportdescriptors`,
* :ref:`logprofiles`, and
* :ref:`xmldynamictypes`.

The following sections will show implementation examples for each of these profiles.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CREATING_XML_PROFILES<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-20, 22-31

.. note::

    The :ref:`examplexml` section shows an XML file with all the possible configurations and profile types.
    This example is useful as a quick reference to look for a particular property and how to use it.
    The
    `Fast DDS XSD scheme <https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_
    can be used as a quick reference too.

.. _loadingapplyingprofiles:

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case the user defines the |Entity-api| profiles via XML files, it is required to load these
XML files using the |DomainParticipantFactory::load_XML_profiles_file-api| public member function before creating any
entity.
It is also possible to load directly the XML information as a string data buffer using the
|DomainParticipantFactory::load_XML_profiles_string-api| public member function.
Moreover, |DomainParticipantFactory::create_participant_with_profile-api|,
|DomainParticipant::create_publisher_with_profile-api|, |DomainParticipant::create_subscriber_with_profile-api|,
|Publisher::create_datawriter_with_profile-api|, and |Subscriber::create_datareader_with_profile-api|
member functions expect a profile name as an argument.
*Fast DDS* searches the given profile name over all the loaded XML profiles, applying the profile to the entity
if founded.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: cpp
    :start-after: //XML-LOAD-APPLY-PROFILES
    :end-before: //!--
    :dedent: 8

.. warning::

    It is worth mentioning that if the same XML profile file is loaded multiple times, the second loading of
    the file will result in an error together with the consequent error log.

.. note::

    To load dynamic types from XML files see the :ref:`Usage` subsection of :ref:`xmldynamictypes`.

.. _rootedvsstandalone:

Rooted vs Standalone profiles definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS* offers various options for the definition of XML profiles. These options are:

*   Stand-alone:
    The element defining the XML profile is the root element of the XML file.
    Elements ``<dds>``, ``<profiles>``, ``<types>``, and ``<log>`` can be defined in a stand-alone manner.
*   Rooted:
    The element defining the XML profile is the child element of another element.
    For example, the ``<participant>``, ``<data_reader>``, ``<data_writer>``, and ``<transport_descriptors>`` elements
    must be defined as child elements of the ``<profiles>`` element.

The following is an example of the definition of the ``<types>`` XML profile using the two previously discussed
approaches.

+----------------------------------------------------------------------------------------------------------------------+
| **Stand-alone**                                                                                                      |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->STANDALONE_TYPES_START<-->                                                                     |
|    :end-before: <!-->STANDALONE_TYPES_END<-->                                                                        |
|    :lines: 2-3, 5-12, 14                                                                                             |
+----------------------------------------------------------------------------------------------------------------------+
| **Rooted**                                                                                                           |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->ROOTED_TYPES_START<-->                                                                         |
|    :end-before: <!-->ROOTED_TYPES_END<-->                                                                            |
|    :lines: 2-3, 5-13, 15-16                                                                                          |
+----------------------------------------------------------------------------------------------------------------------+

Modifying predefined XML profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some scenarios may require to modify some of the QoS after loading the XML profiles.
For such cases the :ref:`dds_layer_core_entity_types` which act as factories provide methods to get the QoS from the
XML profile. This allows the user to read and modify predefined XML profiles before applying them to a new entity.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: cpp
    :start-after: //XML-MIX-WITH-CODE
    :end-before: //!--
    :dedent: 8
