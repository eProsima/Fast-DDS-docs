.. _making_xml_profiles:

Making an XML
-------------

An XML file can contain several XML profiles. The available profile types are :ref:`transportdescriptors`,
:ref:`xmldynamictypes`, :ref:`participantprofiles`, :ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PROFILES-TRANSPORT-DESCRIPTORS<-->
    :lines: 1-6, 43-63

The Fast-RTPS XML format uses some structures along several profiles types.
For readability, the :ref:`commonxml` section groups these common structures.

Finally, The :ref:`examplexml` section shows an XML file that uses all the possibilities.
This example is useful as a quick reference to look for a particular property and how to use it.
This `XSD file <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`__ can be used
as a quick reference too.

.. _loadingapplyingprofiles:

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before creating any entity, it's required to load XML files using ``Domain::loadXMLProfilesFile`` function.
``createParticipant``, ``createPublisher`` and ``createSubscriber`` have a version
that expects the profile name as an argument. *eProsima Fast RTPS* searches the XML profile using
this profile name and applies the XML profile to the entity.

.. literalinclude:: /../code/CodeTester.cpp
    :language: cpp
    :start-after: //XML-LOAD-APPLY-PROFILES
    :end-before: //!--

To load dynamic types from its declaration through XML see the :ref:`Usage` section of :ref:`xmldynamictypes`.
