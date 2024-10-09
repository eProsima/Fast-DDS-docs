.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _making_xml_profiles:

Creating an XML profiles file
-----------------------------

An XML file can contain several XML profiles.
These XML profiles are defined within the ``<dds>`` element, and in turn, within the ``<profiles>`` XML elements.
The possible topologies for the definition of XML profiles are specified in :ref:`rootedvsstandalone`.

It is worth mentioning that the first element of the xml profile must have the ``xmlns`` attribute with the link
``xmlns="http://www.eprosima.com"``, in both rooted or standalone definitions.
That link defines the reference of the ``xsd`` schema that the ``xml`` document complies with.

The available profile types are:

* :ref:`participantprofiles`,
* :ref:`publisherprofiles`,
* :ref:`subscriberprofiles`,
* :ref:`Topic`,
* :ref:`transportdescriptors`,
* :ref:`intra_process_delivery_xml_profile`,
* :ref:`logprofiles`, and
* :ref:`xmldynamictypes`.

The following sections will show implementation examples for each of these profiles.

.. literalinclude:: /../code/XMLTesterSkipValidation.xml
    :language: xml
    :start-after: <!-->CREATING_XML_PROFILES<-->
    :end-before: <!--><-->

.. note::

    The :ref:`examplexml` section shows an XML file with all the possible configurations and profile types.
    This example is useful as a quick reference to look for a particular property and how to use it.
    The
    `Fast DDS XSD scheme <https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastdds_profiles.xsd>`_
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

For simplicity, the |DomainParticipantFactory::create_participant_with_default_profile-api| method takes the default
profile set in the environment to create a participant.
It requires the XML profile to have been already loaded.
Please, refer to :ref:`xml_profiles` for further information regarding loading profiles.

.. warning::

    It is worth mentioning that if the same XML profile file is loaded multiple times, the second loading of
    the file will result in an error together with the consequent error log.

.. note::

    To load dynamic types from XML files see the :ref:`Usage` subsection of :ref:`xmldynamictypes`.

.. _loading_raw_xml_profiles:

Get QoS from raw XML profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Although the :ref:`standard procedure <loadingapplyingprofiles>` is to first load XML profiles, and then create entities
given the profile name of choice, it is also possible to get the desired QoS from the XML profile directly so it can be
modified before being used.
For this purpose, the following public methods are available:

- |DomainParticipantFactory-api|

    - |DomainParticipantFactory::get_participant_qos_from_xml-api|
    - |DomainParticipantFactory::get_default_participant_qos_from_xml-api|
    - |DomainParticipantFactory::get_participant_extended_qos_from_xml-api|
    - |DomainParticipantFactory::get_default_participant_extended_qos_from_xml-api|

- |DomainParticipant-api|

    - |DomainParticipant::get_publisher_qos_from_xml-api|
    - |DomainParticipant::get_default_publisher_qos_from_xml-api|
    - |DomainParticipant::get_subscriber_qos_from_xml-api|
    - |DomainParticipant::get_default_subscriber_qos_from_xml-api|
    - |DomainParticipant::get_topic_qos_from_xml-api|
    - |DomainParticipant::get_default_topic_qos_from_xml-api|
    - |DomainParticipant::get_requester_qos_from_xml-api|
    - |DomainParticipant::get_default_requester_qos_from_xml-api|
    - |DomainParticipant::get_replier_qos_from_xml-api|
    - |DomainParticipant::get_default_replier_qos_from_xml-api|

- |Publisher-api|

    - |Publisher::get_datawriter_qos_from_xml-api|
    - |Publisher::get_default_datawriter_qos_from_xml-api|

- |Subscriber-api|

    - |Subscriber::get_datareader_qos_from_xml-api|
    - |Subscriber::get_default_datareader_qos_from_xml-api|

For each :code:`qos` kind there exists two method versions; with and without :code:`profile_name` argument.
When provided, the method searches for the given profile name in the loaded XML profiles,
and results in error if not found.
When not provided, the method fills the provided :code:`qos` object with the first retrieved profile of the
pertinent kind, and results in error when none found.
In addition, there exists another method version which looks for the default profile of the pertinent kind
in the provided XML string.

Following is an example of how to get the QoS from a raw XML profile and modify it before creating a new entity.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: cpp
    :start-after: //XML-GET-QOS-FROM-XML
    :end-before: //!--
    :dedent: 8

.. _rootedvsstandalone:

Rooted vs Standalone profiles definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS* offers various options for the definition of XML profiles. These options are:

*   Stand-alone:
    The element defining the XML profile is the root element of the XML file.
    Elements ``<dds>``, ``<profiles>``, ``<library_settings>``, ``<types>``, and ``<log>`` can be defined in a
    stand-alone manner.
*   Rooted:
    The element defining the XML profile is the child element of another element.
    For example, the ``<participant>``, ``<data_reader>``, ``<data_writer>``, ``<topic>``, and
    ``<transport_descriptors>`` elements must be defined as child elements of the ``<profiles>`` element.

The following is an example of the definition of the ``<types>`` XML profile using the two previously discussed
approaches.

+----------------------------------------------------------------------------------------------------------------------+
| **Stand-alone**                                                                                                      |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTesterSkipValidation.xml                                                             |
|    :language: xml                                                                                                    |
|    :start-after: <!-->STANDALONE_TYPES_START<-->                                                                     |
|    :end-before: <!-->STANDALONE_TYPES_END<-->                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **Rooted**                                                                                                           |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTesterSkipValidation.xml                                                             |
|    :language: xml                                                                                                    |
|    :start-after: <!-->ROOTED_TYPES_START<-->                                                                         |
|    :end-before: <!-->ROOTED_TYPES_END<-->                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. note::

    Make sure that the first element of the xml profile must have the ``xmlns`` tag with the link
    ``xmlns="http://www.eprosima.com"``, in both rooted or standalone definitions.

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

.. _xml_environment_variables:

Dynamic content by leveraging environment variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For deployment scenarios that require part of the XML content to be dynamically generated, Fast DDS supports using
environment variables on the text content of any XML tag.
The format for environment variables expansion is ``${ENV_VAR_NAME}``, where ``ENV_VAR_NAME`` follows the restrictions
from `IEEE 1003.1 <https://pubs.opengroup.org/onlinepubs/000095399/basedefs/xbd_chap08.html>`_:

.. note::

    Environment variable names ... consist solely of uppercase letters, digits, and the '_' (underscore) from the
    characters defined in Portable Character Set and do not begin with a digit.

More than one environment variable can be used, and they can be mixed with literal text.

The expansion will take place when the XML file is loaded, so changing the value of an environment variable afterwards
will have no effect.

The following is an example of an XML allowing a participant to exclusively communicate with the participants on a
fixed IP address, taken from ``REMOTE_IP_ADDRESS`` environment variable.
It also gives the participant a name that mixes literal text with the content from two environment variables.

.. literalinclude:: /../code/XMLTesterSkipValidation.xml
    :language: xml
    :start-after: <!-->XML_PROFILE_ENVIRONMENT_VARIABLES<-->
    :end-before: <!--><-->

.. warning::

    The `Fast DDS XSD schema <https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastdds_profiles.xsd>`_
    does not support the environment variables expansion feature, so validation of an XML file with environment
    variables expansion expressions will fail.
