.. _xml-profiles:

XML profiles
============

The :ref:`configuration` section shows how to configure entity attributes using XML profiles,
but this section goes deeper on it, explaining each field with its available values and how to compound the complete XML
files.

*eProsima Fast RTPS* permits to load several XML files, each one containing XML profiles.
In addition to the API functions to load user XML files, at initialization *eProsima Fast RTPS* tries to locate and load
several default XML files.
*eProsima Fast RTPS* offers the following options to use default XML files:

* Using an XML file with the name *DEFAULT_FASTRTPS_PROFILES.xml* and located in the current execution path.
* Using an XML file which location is defined in the environment variable *FASTRTPS_DEFAULT_PROFILES_FILE*.

An XML profile is defined by a unique name (or ``<transport_id>`` label
in the :ref:`transportdescriptors` case) that is used to reference the XML profile
during the creation of a Fast RTPS entity, :ref:`comm-transports-configuration`, or :ref:`dynamic-types`.

Making an XML
-------------

An XML file can contain several XML profiles. The available profile types are :ref:`transportdescriptors`,
:ref:`xmldynamictypes`, :ref:`participantprofiles`, :ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PROFILES-TRANSPORT-DESCRIPTORS<-->
    :lines: 1-6, 11-32

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

+-------------------------------+-----------------------------------+---------------------------------+----------------+
| Name                          | Description                       | Values                          | Default        |
+===============================+===================================+=================================+================+
| ``<transport_id>``            | Unique name to identify each      | ``string``                      |                |
|                               | transport descriptor.             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<type>``                    | Type of the transport descriptor. | :class:`UDPv4`, :class:`UDPv6`, | :class:`UDPv4` |
|                               |                                   | :class:`TCPv4`, :class:`TCPv6`  |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<sendBufferSize>``          | Size in bytes of the socket       | ``uint32``                      | 0              |
|                               | send buffer.                      |                                 |                |
|                               | If the value is zero then         |                                 |                |
|                               | FastRTPS will use the default     |                                 |                |
|                               | size from the configuration of    |                                 |                |
|                               | the sockets, using a minimum      |                                 |                |
|                               | size of 65536 bytes.              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<receiveBufferSize>``       | Size in bytes of the socket       | ``uint32``                      | 0              |
|                               | receive buffer.                   |                                 |                |
|                               | If the value is zero then         |                                 |                |
|                               | FastRTPS will use the default     |                                 |                |
|                               | size from the configuration of    |                                 |                |
|                               | the sockets, using a minimum      |                                 |                |
|                               | size of 65536 bytes.              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<TTL>``                     | *Time To Live*, **only**          | ``uint8``                       | 1              |
|                               | for UDP transports .              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<maxMessageSize>``          | The maximum size in bytes         | ``uint32``                      | 65500          |
|                               | of the transport's message        |                                 |                |
|                               | buffer.                           |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<maxInitialPeersRange>``    | The maximum number of             | ``uint32``                      | 4              |
|                               | guessed initial peers             |                                 |                |
|                               | to try to connect.                |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<interfaceWhiteList>``      | Allows defining                   | :ref:`whitelist-interfaces`     |                |
|                               | :ref:`whitelist-interfaces`.      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<wan_addr>``                | Public WAN address when           |  ``string`` with IPv4 Format    |                |
|                               | using **TCPv4 transports**.       |                                 |                |
|                               | This field is optional if         |  :class:`XXX.XXX.XXX.XXX`.      |                |
|                               | the transport doesn't need        |                                 |                |
|                               | to define a WAN address.          |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<output_port>``             | Port used for output bound.       | ``uint16``                      | 0              |
|                               | If this field isn't defined,      |                                 |                |
|                               | the output port will be random    |                                 |                |
|                               | (UDP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<keep_alive_frequency_ms>`` | Frequency in milliseconds         | ``uint32``                      | 50000          |
|                               | for sending RTCP keep-alive       |                                 |                |
|                               | requests (TCP **only**).          |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<keep_alive_timeout_ms>``   | Time in milliseconds since        | ``uint32``                      | 10000          |
|                               | sending the last keep-alive       |                                 |                |
|                               | request to consider a connection  |                                 |                |
|                               | as broken. (TCP **only**).        |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<max_logical_port>``        | The maximum number of logical     | ``uint16``                      | 100            |
|                               | ports to try during RTCP          |                                 |                |
|                               | negotiations.                     |                                 |                |
|                               | (TCP **only**)                    |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_range>``      | The maximum number of logical     | ``uint16``                      | 20             |
|                               | ports per request to try          |                                 |                |
|                               | during RTCP negotiation           |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_increment>``  | Increment between logical         | ``uint16``                      |  2             |
|                               | ports to try during RTCP          |                                 |                |
|                               | negotiation.                      |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<listening_ports>``         | Local port to work as TCP         | ``List <uint16>``               |                |
|                               | acceptor for input connections.   |                                 |                |
|                               | If not set, the transport will    |                                 |                |
|                               | work as TCP client only           |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+

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

.. literalinclude:: ../code/XMLTesterAux.xml
    :language: xml
    :start-after: <!-- ROOTED TYPES START -->
    :end-before: <!-- ROOTED TYPES END -->

Finally, each ``<type>`` tag can contain one or more :ref:`Type definitions <Type definition>`.
Defining several types inside a ``<type>`` tag or defining each type in its ``<type>`` tag has the same result.

.. _Type definition:

Type definition
^^^^^^^^^^^^^^^

**Enum**

The ``<enum>`` type is defined by its ``name`` and a set of ``enumerators``,
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
``<typedef>`` corresponds to :class:`Alias` in Dynamic Types glossary.

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
Each ``case`` has one or more ``caseDiscriminator`` and a ``member``.


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

**Basic types**

The identifiers of the available basic types are:

+------------------------+------------------------+------------------------+
| ``boolean``            | ``int64``              | ``float128``           |
+------------------------+------------------------+------------------------+
| ``byte``               | ``uint16``             | ``string``             |
+------------------------+------------------------+------------------------+
| ``char``               | ``uint32``             | ``wstring``            |
+------------------------+------------------------+------------------------+
| ``wchar``              | ``uint64``             |                        |
+------------------------+------------------------+------------------------+
| ``int16``              | ``float32``            |                        |
+------------------------+------------------------+------------------------+
| ``int32``              | ``float64``            |                        |
+------------------------+------------------------+------------------------+


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

**Arrays**

Arrays are defined in the same way as any other member type but add the attribute ``arrayDimensions``.
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

.. code-block:: c

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its ``name``, its content ``type``, and its ``sequenceMaxLength``.
The type of its content should be defined by its ``type`` attribute.

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

The example shows a sequence with ``sequenceMaxLength`` ``3`` of sequences with ``sequenceMaxLength`` ``2``
with ``<int32>`` contents.
As IDL would be:

.. code-block:: c

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner sequence has been defined before.

**Maps**

Maps are similar to sequences, but they need to define two types instead of one.
One type defines its ``key_type``, and the other type defines its elements types.
Again, both types can be defined as attributes or as members, but when defined
as members, they should be contained in another XML element (``<key_type>`` and ``<type>`` respectively).

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

.. code-block:: c

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

+-----------------------------------+-----------------------------------+-----------------------------+---------+
| Name                              | Description                       | Values                      | Default |
+===================================+===================================+=============================+=========+
| ``<name>``                        | Participant's name.               | ``string``                  |         |
|                                   | It's not the same                 |                             |         |
|                                   | field that ``profile_name``.      |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<defaultUnicastLocatorList>``   | List of default input             | ``LocatorListType``         |         |
|                                   | unicast locators.                 |                             |         |
|                                   | It expects a                      |                             |         |
|                                   | :ref:`LocatorListType`.           |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<defaultMulticastLocatorList>`` | List of default input             | ``LocatorListType``         |         |
|                                   | multicast locators.               |                             |         |
|                                   | It expects                        |                             |         |
|                                   | a :ref:`LocatorListType`.         |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<sendSocketBufferSize>``        | Size in bytes of the output       | ``uint32``                  | 0       |
|                                   | socket buffer.                    |                             |         |
|                                   | If the value is zero then         |                             |         |
|                                   | FastRTPS will use the default     |                             |         |
|                                   | size from  the configuration      |                             |         |
|                                   | of the sockets, using a           |                             |         |
|                                   | minimum size of 65536 bytes.      |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<listenSocketBufferSize>``      | Size in bytes of the input        | ``uint32``                  | 0       |
|                                   | socket buffer.                    |                             |         |
|                                   | If the value is zero then         |                             |         |
|                                   | FastRTPS will use the default     |                             |         |
|                                   | size from  the configuration      |                             |         |
|                                   | of the sockets, using a           |                             |         |
|                                   | minimum size of 65536 bytes.      |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<builtin>``                     | Built-in parameters.              | :ref:`builtin`              |         |
|                                   | Explained in the                  |                             |         |
|                                   | :ref:`builtin` section.           |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<port>``                        | Allows defining the port          | `Port`_                     |         |
|                                   | parameters and gains related      |                             |         |
|                                   | to the RTPS protocol.             |                             |         |
|                                   | Explained in the `Port`_ section. |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<participantID>``               | Participant's identifier.         | ``int32``                   | 0       |
|                                   | Typically it will be              |                             |         |
|                                   | automatically generated           |                             |         |
|                                   | by the ``Domain``.                |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<throughputController>``        | Allows defining a maximum         | `Throughput`_               |         |
|                                   | throughput.                       |                             |         |
|                                   | Explained in the                  |                             |         |
|                                   | `Throughput`_ section.            |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<userTransports>``              | Transport descriptors             | ``List <string>``           |         |
|                                   | to be used by the                 |                             |         |
|                                   | participant.                      |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<useBuiltinTransports>``        | Boolean field to indicate to      | ``bool``                    | true    |
|                                   | the system that the participant   |                             |         |
|                                   | will use  the default builtin     |                             |         |
|                                   | transport independently of its    |                             |         |
|                                   | ``<userTransports>``.             |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+
| ``<propertiesPolicy>``            | Additional configuration          | :ref:`PropertiesPolicyType` |         |
|                                   | properties.                       |                             |         |
|                                   | It expects a                      |                             |         |
|                                   | :ref:`PropertiesPolicyType`.      |                             |         |
+-----------------------------------+-----------------------------------+-----------------------------+---------+

.. | ``<userData>``    | Allows adding custom information.  | ``string``  |         |
.. +-------------------+------------------------------------+-------------+---------+

.. _Port:

**Port Configuration**

+-------------------------+-----------------------------+------------+---------+
| Name                    | Description                 | Values     | Default |
+=========================+=============================+============+=========+
| ``<portBase>``          | Base ``port``.              | ``uint16`` | 7400    |
+-------------------------+-----------------------------+------------+---------+
| ``<domainIDGain>``      | Gain in ``domainId``.       | ``uint16`` | 250     |
+-------------------------+-----------------------------+------------+---------+
| ``<participantIDGain>`` | Gain in ``participantId``.  | ``uint16`` | 2       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd0>``          | Multicast metadata offset.  | ``uint16`` | 0       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd1>``          | Unicast metadata offset.    | ``uint16`` | 10      |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd2>``          | Multicast user data offset. | ``uint16`` | 1       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd3>``          | Unicast user data offset.   | ``uint16`` | 11      |
+-------------------------+-----------------------------+------------+---------+

.. _builtin:

Built-in parameters
^^^^^^^^^^^^^^^^^^^

This section of the :class:`Participant's rtps` configuration allows defining built-in parameters.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

.. Some large words outside of table. Then table fit maximum line length
.. |usewriliv| replace:: ``<use_WriterLivelinessProtocol>``
.. |metuniloc| replace:: ``<metatrafficUnicastLocatorList>``
.. |metmulloc| replace:: ``<metatrafficMulticastLocatorList>``
.. |loclist| replace:: List of :ref:`LocatorListType`
.. |mempol| replace:: :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC`, :class:`DYNAMIC`
.. |staendxml| replace:: ``<staticEndpointXMLFilename>``
.. |readhistmem| replace:: ``<readerHistoryMemoryPolicy>``
.. |writhistmem| replace:: ``<writerHistoryMemoryPolicy>``


+---------------------------+---------------------------------------+-------------------------+-----------------------+
| Name                      | Description                           | Values                  | Default               |
+===========================+=======================================+=========================+=======================+
| ``<use_SIMPLE_RTPS_PDP>`` | Indicates if the Participant          | ``Boolean``             | :class:`true`         |
|                           | must use the Simple RTPS Discovery    |                         |                       |
|                           | Protocol.                             |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |usewriliv|               | Indicates to use the                  | ``Boolean``             | :class:`true`         |
|                           | WriterLiveliness protocol.            |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<EDP>``                 | - If set to :class:`SIMPLE`,          | :class:`SIMPLE`,        |  :class:`SIMPLE`      |
|                           |   ``<simpleEDP>`` would be used.      | :class:`STATIC`         |                       |
|                           |                                       |                         |                       |
|                           | - If set to :class:`STATIC`,          |                         |                       |
|                           |   StaticEDP based on an XML           |                         |                       |
|                           |   file would be used with the         |                         |                       |
|                           |   contents of                         |                         |                       |
|                           |   ``<staticEndpointXMLFilename>``.    |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<domainId>``            | DomainId to be used by                | ``UInt32``              | 0                     |
|                           | the RTPSParticipant.                  |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<leaseDuration>``       | Indicates how much time remote        |  :ref:`DurationType`    | 130 s                 |
|                           | RTPSParticipants should consider this |                         |                       |
|                           | RTPSParticipant alive.                |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<leaseAnnouncement>``   | The period for the RTPSParticipant    |  :ref:`DurationType`    | 40 s                  |
|                           | to send its Discovery Message to all  |                         |                       |
|                           | other discovered RTPSParticipants     |                         |                       |
|                           | as well as to all Multicast ports.    |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<simpleEDP>``           | Attributes of the SimpleEDP           | :ref:`simpleEDP <sedp>` |                       |
|                           | protocol                              |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |metuniloc|               | Metatraffic Unicast Locator           | |loclist|               |                       |
|                           | List                                  |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |metmulloc|               | Metatraffic Multicast Locator         | |loclist|               |                       |
|                           | List                                  |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| ``<initialPeersList>``    | Initial peers.                        | |loclist|               |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |staendxml|               | StaticEDP XML filename.               | ``string``              |                       |
|                           | Only necessary if ``<EDP>``           |                         |                       |
|                           | is set to :class:`STATIC`             |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |readhistmem|             | Memory policy for builtin             | |mempol|                | :class:`PREALLOCATED` |
|                           | readers.                              |                         |                       |
|                           |                                       |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+
| |writhistmem|             | Memory policy for builtin             | |mempol|                | :class:`PREALLOCATED` |
|                           | writers.                              |                         |                       |
|                           |                                       |                         |                       |
+---------------------------+---------------------------------------+-------------------------+-----------------------+

.. _sedp:

**simpleEDP**

+---------------------------+---------------------------------------------+-------------+---------------+
| Name                      | Description                                 | Values      | Default       |
+===========================+=============================================+=============+===============+
| ``<PUBWRITER_SUBREADER>`` | Indicates if the participant must use       | ``Boolean`` | :class:`true` |
|                           | Publication Writer and Subscription Reader. |             |               |
+---------------------------+---------------------------------------------+-------------+---------------+
| ``<PUBREADER_SUBWRITER>`` | Indicates if the participant must use       | ``Boolean`` | :class:`true` |
|                           | Publication Reader and Subscription Writer. |             |               |
+---------------------------+---------------------------------------------+-------------+---------------+


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

+----------------------------+---------------------------+-------------------------------------+-----------------------+
| Name                       | Description               | Values                              | Default               |
+============================+===========================+=====================================+=======================+
| ``<topic>``                | :ref:`TopicType`          | :ref:`TopicType`                    |                       |
|                            | configuration of the      |                                     |                       |
|                            | publisher.                |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<qos>``                  | Publisher                 | :ref:`CommonQOS`                    |                       |
|                            | :ref:`CommonQOS`          |                                     |                       |
|                            | configuration.            |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<times>``                | It allows configuring     | :ref:`Times <pubtimes>`             |                       |
|                            | some time related         |                                     |                       |
|                            | parameters of the         |                                     |                       |
|                            | publisher.                |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<unicastLocatorList>``   | List of input unicast     | List of :ref:`LocatorListType`      |                       |
|                            | locators. It expects      |                                     |                       |
|                            | a :ref:`LocatorListType`. |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<multicastLocatorList>`` | List of input multicast   | List of :ref:`LocatorListType`      |                       |
|                            | locators. It expects      |                                     |                       |
|                            | a :ref:`LocatorListType`. |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<throughputController>`` | Limits the output         | `Throughput`_                       |                       |
|                            | bandwidth of              |                                     |                       |
|                            | the publisher.            |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<historyMemoryPolicy>``  | Memory allocation kind    | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                            | for publisher's history.  | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                            |                           | :class:`DYNAMIC`                    |                       |
|                            |                           |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<propertiesPolicy>``     | Additional configuration  | :ref:`PropertiesPolicyType`         |                       |
|                            | properties.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<userDefinedID>``        | Used for                  | ``Int16``                           | -1                    |
|                            | StaticEndpointDiscovery.  |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<entityID>``             | EntityId of the           | ``Int16``                           | -1                    |
|                            | *endpoint*.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+

.. _pubtimes:

**Times**

+------------------------------+-------------------------------+---------------------+---------+
| Name                         | Description                   | Values              | Default |
+==============================+===============================+=====================+=========+
| ``<initialHeartbeatDelay>``  | Initial heartbeat delay.      | :ref:`DurationType` | ~45 ms  |
+------------------------------+-------------------------------+---------------------+---------+
| ``<heartbeatPeriod>``        | Periodic HB period.           | :ref:`DurationType` | 3 s     |
+------------------------------+-------------------------------+---------------------+---------+
| ``<nackResponseDelay>``      | Delay to apply to the         | :ref:`DurationType` | ~45 ms  |
|                              | response of a ACKNACK         |                     |         |
|                              | message.                      |                     |         |
+------------------------------+-------------------------------+---------------------+---------+
| ``<nackSupressionDuration>`` | This time allows the          | :ref:`DurationType` | 0 ms    |
|                              | RTPSWriter to ignore          |                     |         |
|                              | nack messages too soon        |                     |         |
|                              | after the data has been sent. |                     |         |
+------------------------------+-------------------------------+---------------------+---------+

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

+----------------------------+---------------------------+-------------------------------------+-----------------------+
| Name                       | Description               | Values                              | Default               |
+============================+===========================+=====================================+=======================+
| ``<topic>``                | :ref:`TopicType`          | :ref:`TopicType`                    |                       |
|                            | configuration of the      |                                     |                       |
|                            | subscriber.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<qos>``                  | Subscriber                | :ref:`CommonQOS`                    |                       |
|                            | :ref:`CommonQOS`          |                                     |                       |
|                            | configuration.            |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<times>``                | It allows configuring     | :ref:`Times <subtimes>`             |                       |
|                            | some time related         |                                     |                       |
|                            | parameters of the         |                                     |                       |
|                            | subscriber.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<unicastLocatorList>``   | List of input unicast     | List of :ref:`LocatorListType`      |                       |
|                            | locators. It expects a    |                                     |                       |
|                            | :ref:`LocatorListType`.   |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<multicastLocatorList>`` | List of input multicast   | List of :ref:`LocatorListType`      |                       |
|                            | locators. It expects a    |                                     |                       |
|                            | :ref:`LocatorListType`.   |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<expectsInlineQos>``     | It indicates if QOS is    | ``Boolean``                         | :class:`false`        |
|                            | expected inline.          |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<historyMemoryPolicy>``  | Memory allocation kind    | :class:`PREALLOCATED`,              | :class:`PREALLOCATED` |
|                            | for subscriber's history. | :class:`PREALLOCATED_WITH_REALLOC`, |                       |
|                            |                           | :class:`DYNAMIC`                    |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<propertiesPolicy>``     | Additional configuration  | :ref:`PropertiesPolicyType`         |                       |
|                            | properties.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<userDefinedID>``        | Used for                  | ``Int16``                           | -1                    |
|                            | StaticEndpointDiscovery.  |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+
| ``<entityID>``             | EntityId of the           | ``Int16``                           | -1                    |
|                            | *endpoint*.               |                                     |                       |
+----------------------------+---------------------------+-------------------------------------+-----------------------+

.. _subtimes:

**Times**

+------------------------------+----------------------------------+---------------------+---------+
| Name                         | Description                      | Values              | Default |
+==============================+==================================+=====================+=========+
| ``<initialAcknackDelay>``    | Initial AckNack delay.           | :ref:`DurationType` | ~45 ms  |
+------------------------------+----------------------------------+---------------------+---------+
| ``<heartbeatResponseDelay>`` | Delay to be applied when         | :ref:`DurationType` | ~4.5 ms |
|                              | a heartbeat message is received. |                     |         |
+------------------------------+----------------------------------+---------------------+---------+

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
for example, in ``<defaultUnicastLocatorList>``.
The locator kind is defined by its own tag and can take the values ``<udpv4>``, ``<tcpv4>``, ``<udpv6>``, and
``<tcpv6>``:

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LOCATOR-LIST<-->
    :end-before: <!--><-->

In this example, there are one locator of each kind in ``<defaultUnicastLocatorList>``.

Let's see each possible Locator's field in detail:

+---------------------+----------------------------------+----------------------------------+------------------+
| Name                | Description                      | Values                           | Default          |
+=====================+==================================+==================================+==================+
| ``<port>``          | RTPS port number of the locator. | ``Uint32``                       | 0                |
|                     | *Physical port* in UDP,          |                                  |                  |
|                     | *logical port* in TCP.           |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<physical_port>`` | TCP's *physical port*.           | ``Uint32``                       | 0                |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<address>``       | IP address of the locator.       | ``string`` with expected format  | ""               |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<unique_lan_id>`` | The LAN ID uniquely identifies   | ``string`` (16 bytes)            |                  |
|                     | the LAN the locator belongs to   |                                  |                  |
|                     | (**TCPv4 only**).                |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+
| ``<wan_address>``   | WAN IPv4 address                 | ``string`` with IPv4 Format      | :class:`0.0.0.0` |
|                     | (**TCPv4 only**).                |                                  |                  |
+---------------------+----------------------------------+----------------------------------+------------------+


.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType (XML label ``<propertiesPolicy>``) allows defining a set of generic properties.
It's useful at defining extended or custom configuration parameters.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PROPERTIES-POLICY<-->
    :end-before: <!--><-->

+-----------------+-------------------------------------------+-------------+----------------+
| Name            | Description                               | Values      | Default        |
+=================+===========================================+=============+================+
| ``<name>``      | Name to identify the property.            | ``string``  |                |
+-----------------+-------------------------------------------+-------------+----------------+
| ``<value>``     | Property's value.                         | ``string``  |                |
+-----------------+-------------------------------------------+-------------+----------------+
| ``<propagate>`` | Indicates if it is going to be serialized | ``Boolean`` | :class:`false` |
|                 | along with the object it belongs to.      |             |                |
+-----------------+-------------------------------------------+-------------+----------------+

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time and it's commonly used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that give it sense, like ``<leaseAnnouncement>`` or
``<leaseDuration>``.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

Duration time can be defined through a constant value directly (:class:`INFINITE`, :class:`ZERO`, or :class:`INVALID`),
or by ``<seconds>`` plus ``<fraction>`` labels:

- :class:`INFINITE`: Constant value, represents an infinite period of time.

- :class:`ZERO`: Constant value, represents 0.0 seconds.

- :class:`INVALID`: Constant value, represents an invalid period of time.

+----------------+-----------------------------------------------------------------+------------+---------+
| Name           | Description                                                     | Values     | Default |
+================+=================================================================+============+=========+
| ``<seconds>``  | Number of seconds.                                              | ``Int32``  | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+
| ``<fraction>`` | Fractions of a second. A fraction is :class:`1/(2^32)` seconds. | ``UInt32`` | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+

.. _TopicType:

Topic Type
^^^^^^^^^^

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange
messages.
There is a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. literalinclude:: ../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->

+-------------------------+-------------------------------+-----------------------------------+-----------------+
| Name                    | Description                   | Values                            | Default         |
+=========================+===============================+===================================+=================+
| ``<kind>``              | It defines the Topic's        | :class:`NO_KEY`,                  | :class:`NO_KEY` |
|                         | kind                          | :class:`WITH_KEY`                 |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<name>``              | It defines the Topic's        | ``string``                        |                 |
|                         | name. Must be unique.         |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<dataType>``          | It references the             | ``string``                        |                 |
|                         | Topic's data type.            |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<historyQos>``        | It controls the behavior      | :ref:`HistoryQos <hQos>`          |                 |
|                         | of *Fast RTPS* when the value |                                   |                 |
|                         | of an instance changes before |                                   |                 |
|                         | it is finally communicated to |                                   |                 |
|                         | some of its existing          |                                   |                 |
|                         | DataReader entities.          |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<resourceLimitsQos>`` | It controls the resources     | :ref:`ResourceLimitsQos <rLsQos>` |                 |
|                         | that *Fast RTPS* can use      |                                   |                 |
|                         | in order to meet the          |                                   |                 |
|                         | requirements imposed          |                                   |                 |
|                         | by the application            |                                   |                 |
|                         | and other QoS settings.       |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+

.. _hQos:

**HistoryQoS**

It controls the behavior of *Fast RTPS* when the value of an instance changes before it is finally
communicated to some of its existing DataReader entities.

+-------------+------------------------+---------------------+--------------------+
| Name        | Description            | Values              | Default            |
+=============+========================+=====================+====================+
| ``<kind>``  | See description below. | :class:`KEEP_LAST`, | :class:`KEEP_LAST` |
|             |                        | :class:`KEEP_ALL`   |                    |
+-------------+------------------------+---------------------+--------------------+
| ``<depth>`` |                        | ``UInt32``          | 1000               |
+-------------+------------------------+---------------------+--------------------+

| If the ``<kind>`` is set to :class:`KEEP_LAST`, then *Fast RTPS* will only attempt to keep the latest values of the
  instance and discard the older ones.
| If the ``<kind>`` is set to :class:`KEEP_ALL`, then *Fast RTPS* will attempt to maintain and deliver all the values
  of the instance to existing subscribers.
| The setting of ``<depth>`` must be consistent with the :ref:`ResourceLimitsQos <rLsQos>`
  ``<max_samples_per_instance>``.
  For these two QoS to be consistent, they must verify that ``depth <= max_samples_per_instance``.

.. _rLsQos:

**ResourceLimitsQos**

It controls the resources that *Fast RTPS* can use in order to meet the requirements imposed by the
application and other QoS settings.

+--------------------------------+---------------------------------------------------------+------------+---------+
| Name                           | Description                                             | Values     | Default |
+================================+=========================================================+============+=========+
| ``<max_samples>``              | It must verify that                                     | ``UInt32`` | 5000    |
|                                | ``max_samples >= max_samples_per_instance``.            |            |         |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<max_instances>``            | It defines the maximum number of instances.             | ``UInt32`` | 10      |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<max_samples_per_instance>`` | It must verify that :ref:`HistoryQos <hQos>`            | ``UInt32`` | 400     |
|                                | ``depth <= max_samples_per_instance``.                  |            |         |
+--------------------------------+---------------------------------------------------------+------------+---------+
| ``<allocated_samples>``        | It controls the maximum number of samples to be stored. | ``UInt32`` | 100     |
+--------------------------------+---------------------------------------------------------+------------+---------+

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

+-------------------+---------------------------------------+---------------------------------+-------------------+
| Name              | Description                           | Values                          | Default           |
+===================+=======================================+=================================+===================+
| ``<durability>``  | It is defined on                      | :class:`VOLATILE`,              | :class:`VOLATILE` |
|                   | :ref:`SettingDataDurability` section. | :class:`TRANSIENT_LOCAL`,       |                   |
|                   |                                       | :class:`TRANSIENT`              |                   |
|                   |                                       |                                 |                   |
+-------------------+---------------------------------------+---------------------------------+-------------------+
| ``<liveliness>``  | Defines the liveliness of the         | :ref:`LivelinessType`           |                   |
|                   | participant.                          |                                 |                   |
+-------------------+---------------------------------------+---------------------------------+-------------------+
| ``<reliability>`` | It is defined on :ref:`reliability`   | :class:`RELIABLE`,              | :class:`RELIABLE` |
|                   | section.                              | :class:`BEST_EFFORT`            |                   |
|                   |                                       |                                 |                   |
|                   |                                       |                                 |                   |
+-------------------+---------------------------------------+---------------------------------+-------------------+
| ``<partition>``   |                                       | It allows the introduction of   | ``List <string>`` |
|                   |                                       | a logical partition concept     |                   |
|                   |                                       | inside the ‘physical’ partition |                   |
|                   |                                       | induced by a domain.            |                   |
+-------------------+---------------------------------------+---------------------------------+-------------------+

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

    .. literalinclude:: ../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_DURABILITYSERVICE<-->
       :end-before: <!--><-->

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


+---------------------------+----------------------------------+---------------------------+--------------------+
| Name                      | Description                      | Values                    | Default            |
+===========================+==================================+===========================+====================+
| ``<kind>``                | Specifies how                    | :class:`AUTOMATIC`,       | :class:`AUTOMATIC` |
|                           | to manage liveliness.            | :class:`MANUAL_BY_TOPIC`, |                    |
|                           |                                  | :class:`MANUAL_BY_TOPIC`  |                    |
+---------------------------+----------------------------------+---------------------------+--------------------+
| ``<leaseDuration>``       | Amount of time that the remote   | :ref:`DurationType`       | 130 s              |
|                           | RTPSParticipants should consider |                           |                    |
|                           | this RTPSParticipant to be alive |                           |                    |
|                           | since the last message.          |                           |                    |
+---------------------------+----------------------------------+---------------------------+--------------------+
| ``<announcement_period>`` | The period to send its Discovery | :ref:`DurationType`       | 40 s               |
|                           | Message to all other             |                           |                    |
|                           | discovered RTPSParticipants as   |                           |                    |
|                           | well as to all Multicast ports.  |                           |                    |
+---------------------------+----------------------------------+---------------------------+--------------------+

.. _Throughput:

Throughput Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Throughput Configuration allows to limit the output bandwidth.

+-----------------------+-----------------------------------------------------------+------------+------------+
| Name                  | Description                                               | Values     | Default    |
+=======================+===========================================================+============+============+
| ``<bytesPerPeriod>``  | Packet size in bytes that this controller will allow in   | ``UInt32`` | 4294967295 |
|                       | a given period.                                           |            |            |
+-----------------------+-----------------------------------------------------------+------------+------------+
| ``<periodMillisecs>`` | Window of time in which no more than ``<bytesPerPeriod>`` | ``UInt32`` | 0          |
|                       | bytes are allowed.                                        |            |            |
+-----------------------+-----------------------------------------------------------+------------+------------+

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
