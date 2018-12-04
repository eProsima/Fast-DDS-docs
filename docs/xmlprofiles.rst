.. _xml-profiles:

XML profiles
============

The :ref:`configuration` section shows how to configure entity attributes using XML profiles,
but this section goes deeper on it, explaining each field with its available values and how to compound the complete XML files.

*eProsima Fast RTPS* permits to load several XML files in the same execution, as they can contain several XML profiles.
An XML profile is defined by a unique name (or ``<transport_id>`` label
in the :ref:`transportdescriptors` case) that is used to reference the XML profile
during the creation of a Fast RTPS entity, :ref:`comm-transports-configuration`, or :ref:`dynamic-types`.
During *eProsima Fast RTPS* initialization,
it tries to load an XML file with the name *DEFAULT_FASTRTPS_PROFILES.xml* in the current execution path.

Making an XML
-------------

An XML file can contain several XML profiles. The available profile types are :ref:`transportdescriptors`,
:ref:`xmldynamictypes`, :ref:`participantprofiles`, :ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PROFILES-TRANSPORT-DESCRIPTORS<-->
    :lines: 1-6, 13-33

The Fast-RTPS XML format uses some structs along several profiles types.
For readability, the :ref:`commonxml` section groups these common structs.

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

.. literalinclude:: ../code/CodeTester.cpp
    :language: cpp
    :start-after: //XML-LOAD-APPLY-PROFILES
    :end-before: //!--

To load dynamic types from its declaration through XML see the :ref:`Usage` section of :ref:`xmldynamictypes`.

.. _transportdescriptors:

Transport descriptors
---------------------

This section allows creating transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well-defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiated it will use or create the related transport.

The following XML code shows the complete list of configurable parameters:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-TRANSPORT-DESCRIPTORS<-->
    :lines: 1, 11-39, 48

The XML label ``<transport_descriptors>`` can hold any number of ``<transport_descriptor>``.

+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| Name                         | Description                                                                     | Values                           | Default        |
+==============================+=================================================================================+==================================+================+
| ``<transport_id>``           | Unique name to identify each transport descriptor.                              | ``string``                       |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<type>``                   | Type of the transport descriptor.                                               | :class:`UDPv4`, :class:`UDPv6`,  | :class:`UDPv4` |
|                              |                                                                                 | :class:`TCPv4`, :class:`TCPv6`   |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<sendBufferSize>``         | | Size in bytes of the socket send buffer.                                      | ``uint32``                       | 0              |
|                              | | If the value is zero then FastRTPS will use the default size from             |                                  |                |
|                              | | the configuration of the sockets, using a minimum size of 65536 bytes.        |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<receiveBufferSize>``      | | Size in bytes of the socket receive buffer.                                   | ``uint32``                       | 0              |
|                              | | If the value is zero then FastRTPS will use the default size from             |                                  |                |
|                              | | the configuration of the sockets, using a minimum size of 65536 bytes.        |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<TTL>``                    | *Time To Live*, **only** for UDP transports.                                    | ``uint8``                        | 1              |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<maxMessageSize>``         | The maximum size in bytes of the transport's message buffer.                    | ``uint32``                       | 65500          |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<maxInitialPeersRange>``   | The maximum number of guessed initial peers to try to connect.                  | ``uint32``                       | 4              |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<interfaceWhiteList>``     | Allows defining :ref:`whitelist-interfaces`.                                    | :ref:`whitelist-interfaces`      |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<wan_addr>``               | | Public WAN address when using **TCPv4 transports**.                           | | ``string`` with IPv4 Format    |                |
|                              | | This field is optional if the transport doesn't need to define a WAN address. | | :class:`XXX.XXX.XXX.XXX`.      |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<output_port>``            | | Port used for output bound.                                                   | ``uint16``                       | 0              |
|                              | | If this field isn't defined, the output port will be random.                  |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<keep_alive_frequency_ms>``| Frequency in milliseconds for sending RTCP keepalive requests (TCP **only**).   | ``uint32``                       | 50000          |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<keep_alive_timeout_ms>``  | | Time in milliseconds since sending the last keepalive request                 | ``uint32``                       | 10000          |
|                              | | to consider a connection as broken. (TCP **only**).                           |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<max_logical_port>``       | | The maximum number of logical ports to try during RTCP negotiation.           | ``uint16``                       | 100            |
|                              | | (TCP **only**)                                                                |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<logical_port_range>``     | | The maximum number of logical ports per request to try                        | ``uint16``                       | 20             |
|                              | | during RTCP negotiation (TCP **only**).                                       |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<logical_port_increment>`` | | Increment between logical ports to try during RTCP negotiation.               | ``uint16``                       | 2              |
|                              | | (TCP **only**).                                                               |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+
| ``<ListeningPorts>``         | | Local port to work as TCP acceptor for input connections.                     | ``List <uint16>``                |                |
|                              | | If not set, the transport will work as TCP client only (TCP **only**).        |                                  |                |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+----------------+

There are more examples of transports descriptors in :ref:`comm-transports-configuration`.

.. _xmldynamictypes:

XML Dynamic Types
-----------------

XML Dynamic Types allows creating *eProsima Fast RTPS Dynamic Types* directly defining them through XML.
It allows any application to change TopicDataTypes without modifying its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (``<types>`` tag) can be placed similarly to the profiles tag inside the XML file.
It can be a stand-alone XML Types file or be a child of the Fast-RTPS XML root tag (``<dds>``).
Inside the types tag, there must be one or more type tags (``<type>``).

Stand-Alone:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-- STAND ALONE TYPES START -->
    :end-before: <!-- STAND ALONE TYPES END -->

Rooted:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-- ROOTED TYPES START -->
    :end-before: <!-- ROOTED TYPES END -->

Finally, each ``<type>`` tag can contain one or more :ref:`Type definitions <Type definition>`.
Defining several types inside a ``<type>`` tag or defining each type in its ``<type>`` tag has the same result.

.. _Type definition:

Type definition
^^^^^^^^^^^^^^^

**Enum**

The ``<enum>`` type is defined by its ``name`` and a set of ``literals``,
each of them with its ``name`` and its (optional) ``value``.

Example:

+-----------------------------------------------+-------------------------------------------------------+
| XML                                           | C++                                                   |
+===============================================+=======================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp            |
|   :language: xml                              |     :language: cpp                                    |
|   :start-after: <!-->XML-DYN-ENUM<-->         |     :start-after: //XML-DYN-ENUM                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                                |
|                                               |                                                       |
+-----------------------------------------------+-------------------------------------------------------+

**Typedef**

The ``<typedef>`` type is defined by its ``name`` and its ``value`` or an inner element for complex types.
Typedefs correspond to :class:`Alias` in Dynamic Types glossary.

Example:

+-----------------------------------------------+------------------------------------------------------+
| XML                                           | C++                                                  |
+===============================================+======================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp           |
|   :language: xml                              |     :language: cpp                                   |
|   :start-after: <!-->XML-TYPEDEF<-->          |     :start-after: //XML-TYPEDEF                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                               |
|                                               |                                                      |
+-----------------------------------------------+------------------------------------------------------+

**Struct**

The ``<struct>`` type is defined by its ``name`` and inner *members*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-STRUCT<-->           |     :start-after: //XML-STRUCT                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+


**Union**

The ``<union>`` type is defined by its ``name``, a ``discriminator`` and a set of ``cases``.
Each ``case`` has one or more ``caseValue`` and a *member*.


Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-UNION<-->            |     :start-after: //XML-UNION                       |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+


Member types
^^^^^^^^^^^^

Member types are any type that can belong to a ``<struct>`` or a ``<union>``, or be aliased by a ``<typedef>``.

When used as ``<sequence>``'s elements, ``key`` or ``value`` types of a map, as an aliased type, etc.,
its ``name`` attribute is ignored and can be omitted.

**Basic types**

The tags of the available basic types are:

+--------------------------+--------------------------+--------------------------+
| ``<boolean>``            | ``<longlong>``           | ``<longdouble>``         |
+--------------------------+--------------------------+--------------------------+
| ``<octet>``              | ``<unsignedshort>``      | ``<string>``             |
+--------------------------+--------------------------+--------------------------+
| ``<char>``               | ``<unsignedlong>``       | ``<wstring>``            |
+--------------------------+--------------------------+--------------------------+
| ``<wchar>``              | ``<unsignedlonglong>``   | ``<boundedString>``      |
+--------------------------+--------------------------+--------------------------+
| ``<short>``              | ``<float>``              | ``<boundedWString>``     |
+--------------------------+--------------------------+--------------------------+
| ``<long>``               | ``<double>``             |                          |
+--------------------------+--------------------------+--------------------------+


All of them are defined as follows:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-GENERIC<-->          |     :start-after: //XML-GENERIC                     |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

Except for ``<boundedString>`` and ``<boundedWString>`` that should include an inner element :class:`maxLength`
whose value indicates the maximum length of the string.

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BOUNDEDSTRINGS<-->   |     :start-after: //XML-BOUNDEDSTRINGS              |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Arrays**

Arrays are defined in the same way as any other member type but add the attribute :class:`dimensions`.
The format of this dimensions attribute is the size of each dimension separated by commas.

Example:


+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-ARRAYS<-->           |     :start-after: //XML-ARRAYS                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+


It's IDL analog would be:

.. code-block:: c++

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its :class:`name`, its content :class:`type`, and optionally its :class:`length`.
The type of its content can be defined by its :class:`type` attribute or by a member type.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-SEQUENCES<-->        |     :start-after: //XML-SEQUENCES                   |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

The example shows a sequence with ``length`` ``3`` of sequences with ``length`` ``2`` with ``<long>`` contents.
As IDL would be:

.. code-block:: c++

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner (or content) sequence has no ``name``, as it would be ignored by the parser.

**Maps**

Maps are similar to sequences, but they need to define two types instead of one.
One type defines its :class:`key_type`, and the other type defines its :class:`value_type`.
Again, both types can be defined as attributes or as members, but when defined
as members, they should be contained in another XML element (``<key_type>`` and ``<value_type>`` respectively).

The definition kind of each type can be mixed, this is, one type can be defined as an attribute and the
other as a member.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: ../code/XMLTester.xml     | .. literalinclude:: ../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-MAPS<-->             |     :start-after: //XML-MAPS                        |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

Is equivalent to the IDL:

.. code-block:: c++

    map<long,map<long,long,2>,2> my_map_map;

**Complex types**

Once defined, complex types can be used as members in the same way a basic or array type would be.

Example:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-COMPLEX<-->
    :end-before: <!--><-->

.. _Usage:

Usage
^^^^^

In the application that will make use of *XML Types*, it's mandatory to load the XML file that defines
the types before trying to instantiate *DynamicPubSubTypes* of these types.
It's important to remark that only ``<struct>`` types generate usable *DynamicPubSubType* instances.

.. literalinclude:: ../code/CodeTester.cpp
    :language: cpp
    :start-after: //XML-USAGE
    :end-before: //!--

.. _participantprofiles:

Participant profiles
--------------------

Participant profiles allow declaring :ref:`participantconfiguration` from an XML file.
All the configuration options for the participant belongs to the ``<rtps>`` label.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

List with the possible configuration parameter:

+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| Name                              | Description                                                                   | Values                           | Default |
+===================================+===============================================================================+==================================+=========+
| ``<name>``                        | Participant's name. It's not the same field that ``profile_name``.            | ``string``                       |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<defaultUnicastLocatorList>``   | List of default input unicast locators. It expects a :ref:`LocatorListType`.  | ``LocatorListType``              |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<defaultMulticastLocatorList>`` | List of default input multicast locators. It expects a :ref:`LocatorListType`.| ``LocatorListType``              |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<sendSocketBufferSize>``        | | Size in bytes of the output socket buffer.                                  | ``uint32``                       | 0       |
|                                   | | If the value is zero then FastRTPS will use the default size from           |                                  |         |
|                                   | | the configuration of the sockets, using a minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<listenSocketBufferSize>``      | | Size in bytes of the input socket buffer.                                   | ``uint32``                       | 0       |
|                                   | | If the value is zero then FastRTPS will use the default size from           |                                  |         |
|                                   | | the configuration of the sockets, using a minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<builtin>``                     | Built-in parameters. Explained in the :ref:`builtin` section.                 | :ref:`builtin`                   |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<port>``                        | | Allows defining the port parameters and gains related to the RTPS protocol. | `Port`_                          |         |
|                                   | | Explained in the `Port`_ section.                                           |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<participantID>``               | | Participant's identifier.                                                   | ``int32``                        | 0       |
|                                   | | Typically it will be autogenerated by the ``Domain``.                       |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<throughputController>``        | | Allows defining a maximum throughput.                                       | `Throughput`_                    |         |
|                                   | | Explained in the `Throughput`_ section.                                     |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<userTransports>``              | Transport descriptors to be used by the participant.                          | ``List <string>``                |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<useBuiltinTransports>``        | | Boolean field to indicate to the system that the participant will use       | ``bool``                         | true    |
|                                   | | the default builtin transport independently of its ``<userTransports>``.    |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<propertiesPolicy>``            | | Additional configuration properties.                                        | :ref:`PropertiesPolicyType`      |         |
|                                   | | It expects a :ref:`PropertiesPolicyType`.                                   |                                  |         |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+

.. | ``<userData>``                    | Allows adding custom information.                                             | ``string``                       |         |
.. +-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+

.. _Port:

**Port Configuration**

+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| Name                              | Description                                                                   | Values                           | Default |
+===================================+===============================================================================+==================================+=========+
| ``<portBase>``                    | Base ``port``.                                                                | ``uint16``                       | 7400    |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<domainIDGain>``                | Gain in ``domainId``.                                                         | ``uint16``                       | 250     |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<participantIDGain>``           | Gain in ``participantId``.                                                    | ``uint16``                       | 2       |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<offsetd0>``                    | Multicast metadata offset.                                                    | ``uint16``                       | 0       |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<offsetd1>``                    | Unicast metadata offset.                                                      | ``uint16``                       | 10      |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<offsetd2>``                    | Multicast user data offset.                                                   | ``uint16``                       | 1       |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+
| ``<offsetd3>``                    | Unicast user data offset.                                                     | ``uint16``                       | 11      |
+-----------------------------------+-------------------------------------------------------------------------------+----------------------------------+---------+

.. _builtin:

Built-in parameters
^^^^^^^^^^^^^^^^^^^

This section of the :class:`Participant's rtps` configuration allows defining built-in parameters.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| Name                                   | Description                                                                | Values                              | Default               |
+========================================+============================================================================+=====================================+=======================+
| ``<use_SIMPLE_RTPS_PDP>``              | Indicates if the Participant must use the Simple RTPS Discovery Protocol.  | ``Boolean``                         | :class:`true`         |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<use_WriterLivelinessProtocol>``     | Indicates to use the WriterLiveliness protocol.                            | ``Boolean``                         | :class:`true`         |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<EDP>``                              | | - If set to :class:`SIMPLE`, ``<simpleEDP>`` would be used.              | :class:`SIMPLE`, :class:`STATIC`    | :class:`SIMPLE`       |
|                                        | | - If set to :class:`STATIC`, StaticEDP based on an XML file would be used|                                     |                       |
|                                        | |       with the contents of ``<staticEndpointXMLFilename>``.              |                                     |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<domainId>``                         | DomainId to be used by the RTPSParticipant.                                | ``UInt32``                          | 0                     |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<leaseDuration>``                    | | Indicates how much time remote RTPSParticipants should consider this     | :ref:`DurationType`                 | 130 s                 |
|                                        | | RTPSParticipant alive.                                                   |                                     |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<leaseAnnouncement>``                | | The period for the RTPSParticipant to send its Discovery Message to all  | :ref:`DurationType`                 | 40 s                  |
|                                        | | other discovered RTPSParticipants as well as to all Multicast ports.     |                                     |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<simpleEDP>``                        | Attributes of the SimpleEDP protocol                                       | :ref:`simpleEDP <sedp>`             |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<metatrafficUnicastLocatorList>``    | Metatraffic Unicast Locator List                                           | List of :ref:`LocatorListType`      |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<metatrafficMulticastLocatorList>``  | Metatraffic Multicast Locator List.                                        | List of :ref:`LocatorListType`      |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<initialPeersList>``                 | Initial peers.                                                             | List of :ref:`LocatorListType`      |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<staticEndpointXMLFilename>``        | | StaticEDP XML filename.                                                  | ``string``                          |                       |
|                                        | | Only necessary if ``<EDP>`` is set to :class:`STATIC`                    |                                     |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<readerHistoryMemoryPolicy>``        | Memory policy for builtin readers.                                         | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                                        |                                                                            | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                                        |                                                                            | :class:`DYNAMIC`                    |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<writerHistoryMemoryPolicy>``        | Memory policy for builtin writers.                                         | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                                        |                                                                            | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                                        |                                                                            | :class:`DYNAMIC`                    |                       |
+----------------------------------------+----------------------------------------------------------------------------+-------------------------------------+-----------------------+

.. _sedp:

**simpleEDP**

+------------------------------+----------------------------------------------------------------------------------+---------------------+--------------------+
| Name                         | Description                                                                      | Values              | Default            |
+==============================+==================================================================================+=====================+====================+
| ``<PUBWRITER_SUBREADER>``    | Indicates if the participant must use Publication Writer and Subcription Reader. | ``Boolean``         | :class:`true`      |
+------------------------------+----------------------------------------------------------------------------------+---------------------+--------------------+
| ``<PUBREADER_SUBWRITER>``    | Indicates if the participant must use Publication Reader and Subcription Writer. | ``Boolean``         | :class:`true`      |
+------------------------------+----------------------------------------------------------------------------------+---------------------+--------------------+


.. _publisherprofiles:

Publisher profiles
------------------

Publisher profiles allow declaring :ref:`Publisher configuration <pubsubconfiguration>` from an XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in the :ref:`loadingapplyingprofiles` section.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PUBLISHER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| Name                              | Description                                                             | Values                              | Default               |
+===================================+=========================================================================+=====================================+=======================+
| ``<topic>``                       | :ref:`TopicType` configuration of the pubsliher.                        | :ref:`TopicType`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<qos>``                         | Publisher :ref:`CommonQOS` configuration.                               | :ref:`CommonQOS`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<times>``                       | It allows configuring some time related parameters of the publisher .   | :ref:`Times <pubtimes>`             |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<unicastLocatorList>``          | List of input unicast locators. It expects a :ref:`LocatorListType`.    | List of :ref:`LocatorListType`      |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<multicastLocatorList>``        | List of input multicast locators. It expects a :ref:`LocatorListType`.  | List of :ref:`LocatorListType`      |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<throughputController>``        | Limits the output bandwidth of the publisher.                           | `Throughput`_                       |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<historyMemoryPolicy>``         | Memory allocation kind for pubsliher's history.                         | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                                   |                                                                         | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                                   |                                                                         | :class:`DYNAMIC`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<propertiesPolicy>``            | Additional configuration properties.                                    | :ref:`PropertiesPolicyType`         |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<userDefinedID>``               | Used for StaticEndpointDiscovery.                                       | ``Int16``                           | -1                    |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<entityID>``                    | EntityId of the *endpoint*.                                             | ``Int16``                           | -1                    |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+

.. _pubtimes:

**Times**

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<initialHeartbeatDelay>``  | Initial heartbeat delay.                                                      | :ref:`DurationType`              | ~45 ms             |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<heartbeatPeriod>``        | Periodic HB period.                                                           | :ref:`DurationType`              | 3 s                |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<nackResponseDelay>``      | Delay to apply to the response of a ACKNACK message.                          | :ref:`DurationType`              | ~45 ms             |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<nackSupressionDuration>`` | | This time allows the RTPSWriter to ignore nack messages                     | :ref:`DurationType`              | 0 ms               |
|                              | | too soon after the data has been sent.                                      |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _subscriberprofiles:

Subscriber profiles
-------------------

Subscriber profiles allow declaring :ref:`Subscriber configuration <pubsubconfiguration>` from an XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-SUBSCRIBER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| Name                              | Description                                                             | Values                              | Default               |
+===================================+=========================================================================+=====================================+=======================+
| ``<topic>``                       | :ref:`TopicType` configuration of the subscriber.                       | :ref:`TopicType`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<qos>``                         | Subscriber :ref:`CommonQOS` configuration.                              | :ref:`CommonQOS`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<times>``                       | It allows configuring some time related parameters of the subscriber.   | :ref:`Times <subtimes>`             |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<unicastLocatorList>``          | List of input unicast locators. It expects a :ref:`LocatorListType`.    | List of :ref:`LocatorListType`      |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<multicastLocatorList>``        | List of input multicast locators. It expects a :ref:`LocatorListType`.  | List of :ref:`LocatorListType`      |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<expectsInlineQos>``            | It indicates if QOS is expected inline.                                 | ``Boolean``                         | :class:`false`        |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<historyMemoryPolicy>``         | Memory allocation kind for subscriber's history.                        | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                                   |                                                                         | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                                   |                                                                         | :class:`DYNAMIC`                    |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<propertiesPolicy>``            | Additional configuration properties.                                    | :ref:`PropertiesPolicyType`         |                       |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<userDefinedID>``               | Used for StaticEndpointDiscovery.                                       | ``Int16``                           | -1                    |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+
| ``<entityID>``                    | EntityId of the *endpoint*.                                             | ``Int16``                           | -1                    |
+-----------------------------------+-------------------------------------------------------------------------+-------------------------------------+-----------------------+

.. _subtimes:

**Times**

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<initialAcknackDelay>``    | Initial AckNack delay.                                                        | :ref:`DurationType`              | ~45 ms             |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<heartbeatResponseDelay>`` | Delay to be applied when a hearbeat message is received.                      | :ref:`DurationType`              | ~4.5 ms            |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _commonxml:

Common
------

In the above profiles, some types are used in several different places. To avoid too many details, some of that
places have a tag like :class:`LocatorListType` that indicates that field is defined in this section.

.. _LocatorListType:

LocatorListType
^^^^^^^^^^^^^^^

It represents a list of :class:`Locator_t`.
LocatorListType is normally used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that expect a list of locators and give it sense,
for example, in ``<defaultUnicastLocatorList>``:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LOCATOR-LIST<-->
    :end-before: <!--><-->

In this example, there are three different locators in ``<defaultUnicastLocatorList>``.

Let's see each Locator's fields in detail:

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<kind>``                   | Locator's kind.                                                               | :class:`UDPv4`, :class:`UDPv6`,  | :class:`UDPv4`     |
|                              |                                                                               | :class:`TCPv4`, :class:`TCPv6`   |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<port>``                   | Physical port number of the locator.                                          | ``Uint32``                       | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<port_>``                  | | It allows to access low-level TCP port details.                             | :ref:`TCP Ports <tcpports>`      |                    |
|                              | | It is detailed in :ref:`TCP Ports <tcpports>`                               |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<address>``                | IPv4 address of the locator                                                   | ``string`` with IPv4 Format      | :class:`0.0.0.0`   |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<addresses_>``             | | It allows managing low-level details in address of TCPv4 locators.          | :ref:`TCP Addresses <tcpaddrs>`  |                    |
|                              | | It is detailed in :ref:`TCP Addresses <tcpaddrs>`                           |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<ipv6_address>``           | IPv6 address of the locator                                                   | ``string`` with IPv6 Format      | :class:`::`        |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _tcpports:

**TCP Ports**

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<physical_port>``          | TCP port.                                                                     | ``UInt16``                       | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<logical_port>``           | RTPS logical port.                                                            | ``UInt16``                       | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _tcpaddrs:

**TCP Addresses**

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<unique_lan_id>``          | The LAN ID uniquely identifies the LAN the locator belongs to.                | ``string`` (16 bytes)            |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<wan_address>``            | WAN IPv4 address.                                                             | ``string`` with IPv4 Format      | :class:`0.0.0.0`   |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<ip_address>``             | WAN IPv4 address.                                                             | ``string`` with IPv4 Format      | :class:`0.0.0.0`   |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+


.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType (XML label ``<propertiesPolicy>``) allows defining a set of generic properties.
It's useful at defining extended or custom configuration parameters.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PROPERTIES-POLICY<-->
    :end-before: <!--><-->

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<name>``                   | Name to identify the property.                                                | ``string``                       |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<value>``                  | Property's value.                                                             | ``string``                       |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<propagate>``              | Indicates if it is going to be serialized along with the object it belongs to.| ``Boolean``                      | :class:`false`     |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time and it's commonly used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that give it sense, like ``<leaseAnnouncement>`` or ``<leaseDuration>``.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

Duration time can be defined through a constant value directly (:class:`INFINITE`, :class:`ZERO`, or :class:`INVALID`),
or by ``<seconds>`` plus ``<fraction>`` labels:

- :class:`INFINITE`: Constant value, represents an infinite period of time.

- :class:`ZERO`: Constant value, represents 0.0 seconds.

- :class:`INVALID`: Constant value, represents an invalid period of time.

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<seconds>``                | Number of seconds.                                                            | ``Int32``                        | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<fraction>``               | Fractions of a second. A fraction is :class:`1/(2^32)` seconds.               | ``UInt32``                       | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _TopicType:

Topic Type
^^^^^^^^^^

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange messages.
There is a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->

+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                     | Values                           | Default            |
+==============================+=================================================================================+==================================+====================+
| ``<kind>``                   | It defines the Topic's kind                                                     | :class:`NO_KEY`,                 | :class:`NO_KEY`    |
|                              |                                                                                 | :class:`WITH_KEY`                |                    |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<name>``                   | It defines the Topic's name. Must be unique.                                    | ``string``                       |                    |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<dataType>``               | It references the Topic's data type.                                            | ``string``                       |                    |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<historyQos>``             | | It controls the behavior of *Fast RTPS* when the value of an instance changes | :ref:`HistoryQos <hQos>`         |                    |
|                              | | before it is finally communicated to some of its existing DataReader entities.|                                  |                    |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<resourceLimitsQos>``      | | It controls the resources that *Fast RTPS* can use in order to meet the       | :ref:`ResourceLimitsQos <rLsQos>`|                    |
|                              | | requirements imposed by the application and other QoS settings.               |                                  |                    |
+------------------------------+---------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _hQos:

**HistoryQoS**

It controls the behavior of *Fast RTPS* when the value of an instance changes before it is finally
communicated to some of its existing DataReader entities.

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<kind>``                   | See description below.                                                        | :class:`KEEP_LAST`,              | :class:`KEEP_LAST` |
|                              |                                                                               | :class:`KEEP_ALL`                |                    |
+------------------------------+                                                                               +----------------------------------+--------------------+
| ``<depth>``                  |                                                                               | ``UInt32``                       | 1000               |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

| If the ``<kind>`` is set to :class:`KEEP_LAST`, then *Fast RTPS* will only attempt to keep the latest values of the instance and discard the older ones.
| If the ``<kind>`` is set to :class:`KEEP_ALL`, then *Fast RTPS* will attempt to maintain and deliver all the values of the instance to existing subscribers.
| The setting of ``<depth>`` must be consistent with the :ref:`ResourceLimitsQos <rLsQos>` ``<max_samples_per_instance>``. For these two QoS to be consistent, they must verify that ``depth <= max_samples_per_instance``.

.. _rLsQos:

**ResourceLimitsQos**

It controls the resources that *Fast RTPS* can use in order to meet the requirements imposed by the
application and other QoS settings.

+-------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                          | Description                                                                   | Values                           | Default            |
+===============================+===============================================================================+==================================+====================+
| ``<max_samples>``             | It must verify that ``max_samples >= max_samples_per_instance``.              | ``UInt32``                       | 5000               |
+-------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<max_instances>``           | It defines the maximum number of instances.                                   | ``UInt32``                       | 10                 |
+-------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<max_samples_per_instance>``| It must verify that :ref:`HistoryQos <hQos>`                                  | ``UInt32``                       | 400                |
|                               | ``depth <= max_samples_per_instance``.                                        |                                  |                    |
+-------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<allocated_samples>``       | It controls the maximum number of samples to be stored.                       | ``UInt32``                       | 100                |
+-------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _CommonQOS:

QOS
^^^

The quality of service (QoS) handles the restrictions applied to the application.

.. AFTER DURABILITY
    <durabilityService>
        <!-- DURABILITY_SERVICE -->
    </durabilityService>

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-QOS<-->
    :end-before: <!--><-->

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<durability>``             | It is defined on :ref:`SettingDataDurability` section.                        | :class:`VOLATILE`,               | :class:`VOLATILE`  |
|                              |                                                                               | :class:`TRANSIENT_LOCAL`,        |                    |
|                              |                                                                               | :class:`TRANSIENT`               |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<liveliness>``             | Defines the liveliness of the participant.                                    | :ref:`LivelinessType`            |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<reliability>``            | It is defined on :ref:`reliability` section.                                  | :class:`RELIABLE`,               | :class:`RELIABLE`  |
|                              |                                                                               | :class:`BEST_EFFORT`             |                    |
|                              |                                                                               |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<partition>``              | | It allows the introduction of a logical partition concept                   | ``List <string>``                |                    |
|                              | | inside the ‘physical’ partition induced by a domain.                        |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

..
    .. note::

        - :class:`DURATION` means it expects a :ref:`DurationType`.

    ..  - :class:`DURABILITY_SERVICE` means that the label is a :ref:`DurabilityServiceType` block.::

        - :class:`LIVELINESS` means that the label is a :ref:`LiveLinessType` block.


    .. NOT YET SUPPORTED
        - ``<deadline>``: Period of the samples deadline as :ref:`DurationType` within a ``<period>`` tag.

        - ``<latencyBudget>``: Latency budget os the samples as :ref:`DurationType` within a ``<duration>`` tag.

        - ``<lifespan>``: lifespan as :ref:`DurationType`

        - ``<userData>``: Allows adding custom information.

        - ``<timeBasedFilter>``: Allows filtering by time. It's a :ref:`DurationType` within a ``<minimum_separation>`` tag.

        - ``<ownership>``: ``<kind>`` determines whether an instance of the Topic is owned by a single Publisher. If the selected ownership is :class:`EXCLUSIVE` the Publisher will use the Ownership strength value as the strength of its publication. Only the publisher with the highest strength can publish in the same Topic with the same Key.

        - ``<destinationOrder>``: ``<kind>`` determines the destination timestamp. :class:`BY_RECEPTION_TIMESTAMP` for reception and :class:`BY_SOURCE_TIMESTAMP` for the source.

        - ``<presentation>``:
            * ``<access_scope>`` defines the scope of presentation and can be :class:`INSTANCE`, :class:`TOPIC`, or :class:`GROUP`.

            * ``<coherent_access>`` Boolean value to set if the access must be coherent.

            * ``<ordered_access>`` Boolean value to set if the access must be ordered.

        - ``<topicData>``: Allows adding custom topic data.

        - ``<groupData>``: Allows adding custom group data.



.. .. _DurabilityServiceType:

    DurabilityServiceType
    ^^^^^^^^^^^^^^^^^^^^^

    Durability defines the behavior regarding samples that existed on the topic before a subscriber joins.

    .. code-block:: xml

        <durabilityService>
            <service_cleanup_delay>
                <!-- DURATION -->
            </service_cleanup_delay>
            <history_kind>KEEP_LAST</history_kind> <!-- string -->
            <history_depth></history_depth> <!-- unint32 -->
            <max_samples></max_samples> <!-- unint32 -->
            <max_instances></max_instances> <!-- unint32 -->
            <max_samples_per_instance></max_samples_per_instance> <!-- unint32 -->
        </durabilityService>

    - ``<history_kind>``: History handling kind. It accepts :class:`KEEP_LAST` and :class:`KEEP_ALL` values.

    - ``<history_depth>``: Allows establishing the depth of the history.

    - ``<max_samples>``: The maximum number of samples to be stored.

    - ``<max_instances>``: The maximum number of history instances.

    - ``<max_samples_per_instance>``: Allows establishing the maximum number of samples per history instance.

.. _LivelinessType:

LivelinessType
^^^^^^^^^^^^^^

This parameter defines who is responsible for issues of liveliness packets.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LIVELINESS<-->
    :end-before: <!--><-->


+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<kind>``                   | Specifies how to manage liveliness.                                           | :class:`AUTOMATIC`,              | :class:`AUTOMATIC` |
|                              |                                                                               | :class:`MANUAL_BY_TOPIC`,        |                    |
|                              |                                                                               | :class:`MANUAL_BY_TOPIC`         |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<leaseDuration>``          | | Amount of time that the remote RTPSParticipants should consider this        | :ref:`DurationType`              | 130 s              |
|                              | | RTPSParticipant to be alive since the last message.                         |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<announcement_period>``    | | The period to send its Discovery Message to all other                       | :ref:`DurationType`              | 40 s               |
|                              | | discovered RTPSParticipants as well as to all Multicast ports.              |                                  |                    |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _Throughput:

Throughput Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Throughput Configuration allows to limit the output bandwidth.

+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| Name                         | Description                                                                   | Values                           | Default            |
+==============================+===============================================================================+==================================+====================+
| ``<bytesPerPeriod>``         | Packet size in bytes that this controller will allow in a given period.       | ``UInt32``                       | 4294967295         |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+
| ``<periodMillisecs>``        | Window of time in which no more than ``<bytesPerPeriod>`` bytes are allowed.  | ``UInt32``                       | 0                  |
+------------------------------+-------------------------------------------------------------------------------+----------------------------------+--------------------+

.. _examplexml:

Example
-------

In this section, there is a full XML example with all possible configuration.
It can be used as a quick reference, but it may not be valid due to incompatibility or exclusive properties.
Don't take it as a working example.


.. AFTER PUBLISHER->DURABILITY
     <durabilityService>
        <service_cleanup_delay>
            <seconds>10</seconds>
            <fraction>0</fraction>
        </service_cleanup_delay>
        <history_kind>KEEP_LAST</history_kind>
        <history_depth>20</history_depth>
        <max_samples>10</max_samples>
        <max_instances>2</max_instances>
        <max_samples_per_instance>10</max_samples_per_instance>
    </durabilityService>

.. AFTER SUBSCRIBER->DURABILITY
    <durabilityService>
        <service_cleanup_delay>
            <!-- DURATION -->
        </service_cleanup_delay>
        <history_kind>KEEP_LAST</history_kind>
        <history_depth>50</history_depth>
        <max_samples>20</max_samples>
        <max_instances>3</max_instances>
        <max_samples_per_instance>5</max_samples_per_instance>
    </durabilityService>

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-EXAMPLE<-->
    :end-before: <!--><-->
    :lines: 2,4-
