.. include:: includes/aliases.rst

.. _making_xml_profiles:

Creating an XML profiles file
-----------------------------

An XML file can contain several XML profiles.
These XML profiles are defined within the ``<dds>`` element, and in turn, within the ``<profiles>`` XML elements.
The possible topologies for the definition of XML profiles are specified in :ref:`rootedvsstandalone`.
The available profile types are: :ref:`participantprofiles`,
:ref:`publisherprofiles`, :ref:`subscriberprofiles`, :ref:`transportdescriptors`, :ref:`logprofiles`, and
:ref:`xmldynamictypes`.
The following sections will show implementation examples for each of these profiles.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CREATING_XML_PROFILES<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-28, 30-31

.. note::

    The :ref:`examplexml` section shows an XML file with all the possible configurations and profile types.
    This example is useful as a quick reference to look for a particular property and how to use it.
    The
    `Fast DDS XSD scheme <https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_
    can be used as a quick reference too.

.. _loadingapplyingprofiles:

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case the user defines the |Entity| profiles via XML files, it is required to load these
XML files using the |load_XML_profiles_file| public member function before creating any |Entity|.
Moreover, |create_participant_with_profile|, |create_publisher_with_profile|, and |create_subscriber_with_profile|
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
|    :lines: 2, 4-                                                                                                     |
+----------------------------------------------------------------------------------------------------------------------+
| **Rooted**                                                                                                           |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->ROOTED_TYPES_START<-->                                                                         |
|    :end-before: <!-->ROOTED_TYPES_END<-->                                                                            |
|    :lines: 2-3, 5-14, 16                                                                                             |
+----------------------------------------------------------------------------------------------------------------------+




