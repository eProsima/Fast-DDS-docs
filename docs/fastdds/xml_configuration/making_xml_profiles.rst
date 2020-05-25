.. _making_xml_profiles:

Making an XML
-------------

An XML file can contain several XML profiles.
The available profile types are: :ref:`transportdescriptors`, :ref:`xmldynamictypes`, :ref:`participantprofiles`,
:ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PROFILES-TRANSPORT-DESCRIPTORS<-->
    :lines: 1-6, 43-63

The `Fast DDS XSD scheme <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_
uses some structures common to several profiles types.
For readability, the :ref:`commonxml` section groups these common structures.

Finally, The :ref:`examplexml` section shows an XML file with all the possible configurations and profile types.
This example is useful as a quick reference to look for a particular property and how to use it.
The aforementioned
`XSD scheme <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_ can be used
as a quick reference too.

.. _loadingapplyingprofiles:

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before creating any |Entity|, it is required to load XML files using the |load_XML_profiles_file| public
member function.
Moreover, |create_participant_with_profile|, |create_publisher_with_profile|, |create_subscriber_with_profile|,
|create_datawriter_with_profile|, |create_datareader_with_profile|
member functions expects the profile name as an argument.
Fast DDS searches the XML profile using this profile name and applies the XML profile to the entity.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: cpp
    :start-after: //XML-LOAD-APPLY-PROFILES
    :end-before: //!--

To load dynamic types from XML profile files see the :ref:`Usage` subsection of :ref:`xmldynamictypes`.
