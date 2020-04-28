.. _xml_profiles:

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

.. _librarySettingsAttributes:

Library settings
----------------

This section is devoted to general settings that are not constraint to specific entities
(like participants, subscribers, publishers) or functionality (like transports or types).
All of them are gathered under the ``library_settings`` profile.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-LIBRARY-SETTINGS<-->
    :end-before: <!--><-->

Currently only the :ref:`intraprocess-delivery` feature is comprised here.

.. _transportdescriptors:

Transport descriptors
---------------------

This section allows creating transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well-defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiated it will use or create the related transport.

The following XML code shows the complete list of configurable parameters:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-TRANSPORT-DESCRIPTORS<-->
    :lines: 1, 11-39, 53

The XML label ``<transport_descriptors>`` can hold any number of ``<transport_descriptor>``.

+-------------------------------+-----------------------------------+---------------------------------+----------------+
| Name                          | Description                       | Values                          | Default        |
+===============================+===================================+=================================+================+
| ``<transport_id>``            | Unique name to identify each      | ``string``                      |                |
|                               | transport descriptor.             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<type>``                    | Type of the transport descriptor. | :class:`UDPv4`, :class:`UDPv6`, | :class:`UDPv4` |
|                               |                                   | :class:`TCPv4`, :class:`TCPv6`, |                |
|                               |                                   | :class:`SHM`                    |                |
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
| ``<non_blocking_send>``       | Whether to set the non-blocking   | ``bool``                        | false          |
|                               | send mode on the socket           |                                 |                |
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
|                               | for sending                       |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | keep-alive                        |                                 |                |
|                               | requests (TCP **only**).          |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<keep_alive_timeout_ms>``   | Time in milliseconds since        | ``uint32``                      | 10000          |
|                               | sending the last keep-alive       |                                 |                |
|                               | request to consider a connection  |                                 |                |
|                               | as broken. (TCP **only**).        |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<max_logical_port>``        | The maximum number of logical     | ``uint16``                      | 100            |
|                               | ports to try during               |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | negotiations. (TCP **only**)      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_range>``      | The maximum number of logical     | ``uint16``                      | 20             |
|                               | ports per request to try          |                                 |                |
|                               | during :ref:`RTCP<rtcpdefinition>`|                                 |                |
|                               | negotiations. (TCP **only**)      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_increment>``  | Increment between logical         | ``uint16``                      |  2             |
|                               | ports to try during               |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | negotiation. (TCP **only**)       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<listening_ports>``         | Local port to work as TCP         | ``List <uint16>``               |                |
|                               | acceptor for input connections.   |                                 |                |
|                               | If not set, the transport will    |                                 |                |
|                               | work as TCP client only           |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<tls>``                     | Allows to define TLS related      | :ref:`tcp-tls`                  |                |
|                               | parameters and options            |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<segment_size>``            | Size (in bytes) of the            | ``uint32``                      | 262144         |
|                               | shared-memory segment.            |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<port_queue_capacity>``     | Capacity (in number of messages)  | ``uint32``                      | 512            |
|                               | available to every Listener       |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<healthy_check_timeout_ms>``| Maximum time-out (in milliseconds)| ``uint32``                      | 1000           |
|                               | used when checking whether a      |                                 |                |
|                               | Listener is alive & OK            |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<rtps_dump_file>``          | Complete path (including file)    | ``string``                      | empty          |
|                               | where RTPS messages will be       |                                 |                |
|                               | stored for debugging purposes.    |                                 |                |
|                               | An empty string indicates no      |                                 |                |
|                               | trace will be performed           |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+



.. _rtcpdefinition:

RTCP is the control protocol for communications with RTPS over TCP/IP connections.

There are more examples of transports descriptors in :ref:`comm-transports-configuration`.

.. _tcp-tls:

TLS Configuration
^^^^^^^^^^^^^^^^^

Fast-RTPS allows configuring TLS parameters through the ``<tls>`` tag of its Transport Descriptor.
The full list of options is listed here:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TCP-TLS<-->
    :end-before: <!--><-->

+-------------------------------+-----------------------------------+---------------------------------+----------------+
| Name                          | Description                       | Values                          | Default        |
+===============================+===================================+=================================+================+
| ``<password>``                | Password of the private_key_file  | ``string``                      |                |
|                               | if provided (or RSA).             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<private_key_file>``        | Path to the private key           | ``string``                      |                |
|                               | certificate file.                 |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<rsa_private_key_file>``    | Path to the private key           | ``string``                      |                |
|                               | RSA certificate file.             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<cert_chain_file>``         | Path to the public certificate    | ``string``                      |                |
|                               | chain file.                       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<tmp_dh_file>``             | Path to the Diffie-Hellman        | ``string``                      |                |
|                               | parameters file                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_file>``             | Path to the CA (Certification-    | ``string``                      |                |
|                               | Authority) file.                  |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_mode>``             | Establishes the verification      | ``VERIFY_NONE``,                |                |
|                               | mode mask.                        | ``VERIFY_PEER``,                |                |
|                               |                                   | ``VERIFY_FAIL_IF_NO_PEER_CERT``,|                |
|                               |                                   | ``VERIFY_CLIENT_ONCE``          |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<options>``                 | Establishes the SSL Context       | ``DEFAULT_WORKAROUNDS``,        |                |
|                               | options mask                      | ``NO_COMPRESSION``,             |                |
|                               |                                   | ``NO_SSLV2``,                   |                |
|                               |                                   | ``NO_SSLV3``,                   |                |
|                               |                                   | ``NO_TLSV1``,                   |                |
|                               |                                   | ``NO_TLSV1_1``,                 |                |
|                               |                                   | ``NO_TLSV1_2``,                 |                |
|                               |                                   | ``NO_TLSV1_3``,                 |                |
|                               |                                   | ``SINGLE_DH_USE``               |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_paths>``            | Paths where the system will       |  ``string``                     |                |
|                               | look for verification files.      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_depth>``            | Maximum allowed depth for         | ``uint32``                      |                |
|                               | verify intermediate certificates. |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<default_verify_path>``     | Default paths where the system    |  ``boolean``                    | ``false``      |
|                               | will look for verification files. |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<handshake_role>``          | Role that the transport will      | ``DEFAULT``,                    | ``DEFAULT``    |
|                               | take on handshaking.              | ``SERVER``,                     |                |
|                               | On default, the acceptors act as  | ``CLIENT``                      |                |
|                               | ``SERVER`` and the connectors as  |                                 |                |
|                               | ``CLIENT``.                       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+

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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-- STAND ALONE TYPES START -->
    :end-before: <!-- STAND ALONE TYPES END -->

Rooted:

.. literalinclude:: /../code/XMLTesterAux.xml
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp           |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp          |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-STRUCT<-->           |     :start-after: //XML-STRUCT                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

Structs can inherit from another structs:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-STRUCT-INHERIT<-->   |     :start-after: //XML-STRUCT-INHERIT              |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-UNION<-->            |     :start-after: //XML-UNION                       |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Bitset**

The ``<bitset>`` type is defined by its ``name`` and inner *bitfields*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITSET<-->           |     :start-after: //XML-BITSET                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

A bitfield without name is an inaccessible set of bits. Bitfields can specify their management type to ease their
modification and access. The bitfield's bit_bound is mandatory and cannot be bigger than 64.

Bitsets can inherit from another bitsets:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITSET-INHERIT<-->   |     :start-after: //XML-BITSET-INHERIT              |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Bitmask**

The ``<bitmask>`` type is defined by its ``name`` and inner *bit_values*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITMASK<-->          |     :start-after: //XML-BITMASK                     |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

The bitmask can specify its bit_bound, this is, the number of bits that the type will manage. Internally will be
converted to the minimum type that allows to store them. The maximum allowed bit_bound is 64.
Bit_values can define their position inside the bitmask.


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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
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
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-COMPLEX<-->
    :end-before: <!--><-->

.. _Usage:

Usage
^^^^^

In the application that will make use of *XML Types*, it's mandatory to load the XML file that defines
the types before trying to instantiate *DynamicPubSubTypes* of these types.
It's important to remark that only ``<struct>`` types generate usable *DynamicPubSubType* instances.

.. literalinclude:: /../code/CodeTester.cpp
    :language: cpp
    :start-after: //XML-USAGE
    :end-before: //!--

.. _participantprofiles:

Participant profiles
--------------------

Participant profiles allow declaring :ref:`participantconfiguration` from an XML file.
All the configuration options for the participant, except from ``domainId``, belong to the ``<rtps>`` label.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->

List with the configuration tags:

+-----------------------------------+-----------------------------------+----------------------------------+---------+
| Name                              | Description                       | Values                           | Default |
+===================================+===================================+==================================+=========+
| ``<domainId>``                    | DomainId to be used by            | ``UInt32``                       | 0       |
|                                   | the RTPSParticipant.              |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<rtps>``                        | RTPS parameters. Explained        |  `RTPS`_                         |         |
|                                   | in `RTPS`_.                       |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+

.. _RTPS:

RTPS Configuration
^^^^^^^^^^^^^^^^^^


List with the possible configuration parameter:

+-----------------------------------+-----------------------------------+----------------------------------+---------+
| Name                              | Description                       | Values                           | Default |
+===================================+===================================+==================================+=========+
| ``<name>``                        | Participant's name.               | ``string_255``                   |         |
|                                   | It's not the same                 |                                  |         |
|                                   | field that ``profile_name``.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<defaultUnicastLocatorList>``   | List of default input             | ``LocatorListType``              |         |
|                                   | unicast locators.                 |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`LocatorListType`.           |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<defaultMulticastLocatorList>`` | List of default input             | ``LocatorListType``              |         |
|                                   | multicast locators.               |                                  |         |
|                                   | It expects                        |                                  |         |
|                                   | a :ref:`LocatorListType`.         |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<sendSocketBufferSize>``        | Size in bytes of the output       | ``uint32``                       | 0       |
|                                   | socket buffer.                    |                                  |         |
|                                   | If the value is zero then         |                                  |         |
|                                   | FastRTPS will use the default     |                                  |         |
|                                   | size from  the configuration      |                                  |         |
|                                   | of the sockets, using a           |                                  |         |
|                                   | minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<listenSocketBufferSize>``      | Size in bytes of the input        | ``uint32``                       | 0       |
|                                   | socket buffer.                    |                                  |         |
|                                   | If the value is zero then         |                                  |         |
|                                   | FastRTPS will use the default     |                                  |         |
|                                   | size from  the configuration      |                                  |         |
|                                   | of the sockets, using a           |                                  |         |
|                                   | minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<builtin>``                     | Built-in parameters.              | :ref:`builtin`                   |         |
|                                   | Explained in the                  |                                  |         |
|                                   | :ref:`builtin` section.           |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<port>``                        | Allows defining the port          | `Port`_                          |         |
|                                   | parameters and gains related      |                                  |         |
|                                   | to the RTPS protocol.             |                                  |         |
|                                   | Explained in the `Port`_ section. |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<participantID>``               | Participant's identifier.         | ``int32``                        | 0       |
|                                   | Typically it will be              |                                  |         |
|                                   | automatically generated           |                                  |         |
|                                   | by the ``Domain``.                |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<throughputController>``        | Allows defining a maximum         | `Throughput`_                    |         |
|                                   | throughput.                       |                                  |         |
|                                   | Explained in the                  |                                  |         |
|                                   | `Throughput`_ section.            |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<userTransports>``              | Transport descriptors             | ``List <string>``                |         |
|                                   | to be used by the                 |                                  |         |
|                                   | participant.                      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<useBuiltinTransports>``        | Boolean field to indicate to      | ``bool``                         | true    |
|                                   | the system that the participant   |                                  |         |
|                                   | will use  the default builtin     |                                  |         |
|                                   | transport independently of its    |                                  |         |
|                                   | ``<userTransports>``.             |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<propertiesPolicy>``            | Additional configuration          | :ref:`PropertiesPolicyType`      |         |
|                                   | properties.                       |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`PropertiesPolicyType`.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<allocation>``                  | Configuration regarding           | :ref:`ParticipantAllocationType` |         |
|                                   | allocation behavior.              |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`ParticipantAllocationType`  |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+

.. | ``<userData>``    | Allows adding custom information.  | ``string``  |         |
.. +-------------------+------------------------------------+-------------+---------+

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

    - For :class:`ALLOCATION` details, please refer to :ref:`ParticipantAllocationType`.

.. _Port:

Port Configuration
""""""""""""""""""

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

.. _ParticipantAllocationType:

Participant allocation parameters
"""""""""""""""""""""""""""""""""

This section of the :class:`Participant's rtps` configuration allows defining parameters related with allocation
behavior on the participant.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT-ALLOCATION<-->
    :end-before: <!--><-->

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<max_unicast_locators>``
     - Maximum number of unicast locators expected on a remote entity.
       It is recommended to use the maximum number of network interfaces found on any machine the
       participant will connect to.
     - ``UInt32``
     - 4
   * - ``<max_multicast_locators>``
     - Maximum number of multicast locators expected on a remote entity.
       May be set to zero to disable multicast traffic.
     - ``UInt32``
     - 1
   * - ``<total_participants>``
     - Participant :ref:`CommonAlloc` related to the total number of participants in the domain (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_readers>``
     - Participant :ref:`CommonAlloc` related to the total number of readers on each participant (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_writers>``
     - Participant :ref:`CommonAlloc` related to the total number of writers on each participant (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<max_partitions>``
     - Maximum size of the partitions submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_user_data>``
     - Maximum size of the user data submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_properties>``
     - Maximum size of the properties submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -

.. _builtin:

Built-in parameters
"""""""""""""""""""

This section of the :class:`Participant's rtps` configuration allows defining built-in parameters.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

.. Some large words outside of table. Then table fit maximum line length
.. |usewriliv| replace:: ``<use_WriterLivelinessProtocol>``
.. |metuniloc| replace:: ``<metatrafficUnicastLocatorList>``
.. |metmulloc| replace:: ``<metatrafficMulticastLocatorList>``
.. |loclist| replace:: List of :ref:`LocatorListType`
.. |mempol| replace:: :ref:`historyMemoryPolicy <mempol>`
.. |readhistmem| replace:: ``<readerHistoryMemoryPolicy>``
.. |writhistmem| replace:: ``<writerHistoryMemoryPolicy>``
.. |readpaysize| replace:: ``<readerPayloadSize>``
.. |writpaysize| replace:: ``<writerPayloadSize>``
.. |mutTries| replace:: ``<mutation_tries>``
.. |mempoldefault| replace:: :class:`PREALLOCATED_WITH_REALLOC`

+--------------------------------+----------------------------------+-------------------------+-----------------------+
| Name                           | Description                      | Values                  | Default               |
+================================+==================================+=========================+=======================+
| ``<discovery_config>``         | This is the main tag where       |                         |                       |
|                                | discovery-related settings can   | :ref:`discovery_config  |                       |
|                                | be configured.                   | <dconf>`                |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| ``<avoid_builtin_multicast>``  | Restricts metatraffic multicast  | ``Boolean``             | :class:`true`         |
|                                | traffic to PDP only.             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |usewriliv|                    | Indicates to use the             | ``Boolean``             | :class:`true`         |
|                                | WriterLiveliness protocol.       |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |metuniloc|                    | Metatraffic Unicast Locator      | |loclist|               |                       |
|                                | List                             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |metmulloc|                    | Metatraffic Multicast Locator    | |loclist|               |                       |
|                                | List                             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| ``<initialPeersList>``         | Initial peers.                   | |loclist|               |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |readhistmem|                  | Memory policy for builtin        | |mempol|                | |mempoldefault|       |
|                                | readers.                         |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |writhistmem|                  | Memory policy for builtin        | |mempol|                | |mempoldefault|       |
|                                | writers.                         |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |readpaysize|                  | Maximum payload size for         | ``UInt32``              | 512                   |
|                                | builtin readers.                 |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |writpaysize|                  | Maximum payload size for         | ``UInt32``              | 512                   |
|                                | builtin writers.                 |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |mutTries|                     | Number of different ports        | ``UInt32``              | 100                   |
|                                | to try if reader's physical port |                         |                       |
|                                | is already in use.               |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+

.. _dconf:

**discovery_config**

.. More large words outside of table. Then table fit maximum line length
.. |staendxml| replace:: ``<staticEndpointXMLFilename>``
.. |protocol| replace:: :class:`SIMPLE`, :class:`CLIENT`, :class:`SERVER`, :class:`BACKUP`
.. |igpartf| replace:: ``<ignoreParticipantFlags>``
.. |filterlist| replace:: :ref:`ignoreParticipantFlags <Participantfiltering>`

+----------------------------+---------------------------------------+-------------------------+----------------------+
| Name                       | Description                           | Values                  | Default              |
+============================+=======================================+=========================+======================+
| ``<discoveryProtocol>``    | Indicates which kind of PDP protocol  | |protocol|              | :class:`SIMPLE`      |
|                            | the participant must use.             |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| |igpartf|                  | Restricts metatraffic using           | |filterlist|            | :class:`NO_FILTER`   |
|                            | several filtering criteria.           |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<EDP>``                  | - If set to :class:`SIMPLE`,          | :class:`SIMPLE`,        | :class:`SIMPLE`      |
|                            |   ``<simpleEDP>`` would be used.      | :class:`STATIC`         |                      |
|                            | - If set to :class:`STATIC`,          |                         |                      |
|                            |   StaticEDP based on an XML           |                         |                      |
|                            |   file would be used with the         |                         |                      |
|                            |   contents of                         |                         |                      |
|                            |   ``<staticEndpointXMLFilename>``.    |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<simpleEDP>``            | Attributes of the SimpleEDP           | :ref:`simpleEDP <sedp>` |                      |
|                            | protocol                              |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<leaseDuration>``        | Indicates how long this               |  :ref:`DurationType`    |  20 s                |
|                            | RTPSParticipant should consider       |                         |                      |
|                            | remote RTPSParticipants alive.        |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<leaseAnnouncement>``    | The period for the RTPSParticipant    |  :ref:`DurationType`    |  3 s                 |
|                            | to send its Discovery Message to all  |                         |                      |
|                            | other discovered RTPSParticipants     |                         |                      |
|                            | as well as to all Multicast ports.    |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<initialAnnouncements>`` | Allows the user to configure          |  :ref:`Initial          |                      |
|                            | the number and period of the initial  |  Announcements          |                      |
|                            | RTPSparticipant's Discovery messages. |  <InitAnnounce>`        |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| |staendxml|                | StaticEDP XML filename.               | ``string``              |                      |
|                            | Only necessary if ``<EDP>``           |                         |                      |
|                            | is set to :class:`STATIC`             |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+

.. _Participantfiltering:

**ignoreParticipantFlags**

+-----------------------------------------------------------+------------------------+
| Possible values                                           | Description            |
+===========================================================+========================+
| :class:`NO_FILTER`                                        | All Discovery traffic  |
|                                                           | is processed           |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_HOST`                            | Discovery traffic from |
|                                                           | another host is        |
|                                                           | discarded              |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_PROCESS`                         | Discovery traffic from |
|                                                           | another process on the |
|                                                           | same host is discarded |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_SAME_PROCESS`                              | Discovery traffic from |
|                                                           | participant's own      |
|                                                           | process is discarded.  |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_PROCESS | FILTER_SAME_PROCESS`   | Discovery traffic from |
|                                                           | participant's own host |
|                                                           | is discarded.          |
+-----------------------------------------------------------+------------------------+

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

.. _InitAnnounce:

**Initial Announcements**

+---------------------------+---------------------------------------------+---------------------+---------------+
| Name                      | Description                                 | Values              | Default       |
+===========================+=============================================+=====================+===============+
| ``<count>``               | Number of Discovery Messages to send at the | ``Uint32``          | 5             |
|                           | period specified by ``<period>``.           |                     |               |
|                           | After these announcements, the              |                     |               |
|                           | RTPSParticipant will continue sending its   |                     |               |
|                           | Discovery Messages at the                   |                     |               |
|                           | ``<leaseAnnouncement>`` rate.               |                     |               |
+---------------------------+---------------------------------------------+---------------------+---------------+
| ``<period>``              | The period for the RTPSParticipant to send  | :ref:`DurationType` | 100 ms        |
|                           | its first ``<count>`` Discovery Messages.   |                     |               |
+---------------------------+---------------------------------------------+---------------------+---------------+


.. _publisherprofiles:

Publisher profiles
------------------

Publisher profiles allow declaring :ref:`Publisher configuration <pubsubconfiguration>` from an XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in the :ref:`loadingapplyingprofiles` section.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PUBLISHER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.


.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<topic>``
     - :ref:`TopicType` configuration of the publisher.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - Publisher :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It allows configuring some time related parameters of the publisher.
     - :ref:`Times <pubtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<throughputController>``
     - Limits the output bandwidth of the publisher.
     - `Throughput`_
     -
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for publisher's history.
     - :ref:`historyMemoryPolicy <mempol>`
     - :class:`PREALLOCATED`
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for StaticEndpointDiscovery.
     - ``Int16``
     - -1
   * - ``<entityID>``
     - EntityId of the *endpoint*.
     - ``Int16``
     - -1
   * - ``<matchedSubscribersAllocation>``
     - Publisher :ref:`CommonAlloc` related to the number of matched subscribers.
     - :ref:`CommonAlloc`
     -

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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-SUBSCRIBER<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<topic>``
     - :ref:`TopicType` configuration of the subscriber.
     - :ref:`TopicType`
     -
   * - ``<qos>``
     - Subscriber :ref:`CommonQOS` configuration.
     - :ref:`CommonQOS`
     -
   * - ``<times>``
     - It allows configuring some time related parameters of the subscriber.
     - :ref:`Times <subtimes>`
     -
   * - ``<unicastLocatorList>``
     - List of input unicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<multicastLocatorList>``
     - List of input multicast locators. It expects a :ref:`LocatorListType`.
     - List of :ref:`LocatorListType`
     -
   * - ``<expectsInlineQos>``
     - It indicates if QOS is expected inline.
     - ``Boolean``
     - :class:`false`
   * - ``<historyMemoryPolicy>``
     - Memory allocation kind for subscriber's history.
     - :ref:`historyMemoryPolicy <mempol>`
     - :class:`PREALLOCATED`
   * - ``<propertiesPolicy>``
     - Additional configuration properties.
     - :ref:`PropertiesPolicyType`
     -
   * - ``<userDefinedID>``
     - Used for StaticEndpointDiscovery.
     - ``Int16``
     - -1
   * - ``<entityID>``
     - EntityId of the *endpoint*.
     - ``Int16``
     - -1
   * - ``<matchedPublishersAllocation>``
     - Subscriber :ref:`CommonAlloc` related to the number of matched publishers.
     - :ref:`CommonAlloc`
     -

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

.. literalinclude:: /../code/XMLTester.xml
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

.. literalinclude:: /../code/XMLTester.xml
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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

Duration time can be defined through ``<sec>`` plus ``<nanosec>`` labels (see table below). An infinite value can be
specified by using the values :class:`DURATION_INFINITY`, :class:`DURATION_INFINITE_SEC` and
:class:`DURATION_INFINITE_NSEC`.

+----------------+-----------------------------------------------------------------+------------+---------+
| Name           | Description                                                     | Values     | Default |
+================+=================================================================+============+=========+
| ``<sec>``      | Number of seconds.                                              | ``Int32``  | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+
| ``<nanosec>``  | Number of nanoseconds.                                          | ``UInt32`` | 0       |
+----------------+-----------------------------------------------------------------+------------+---------+

.. _TopicType:

Topic Type
^^^^^^^^^^

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange
messages.
There is a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->

+-------------------------+-------------------------------+-----------------------------------+-----------------+
| Name                    | Description                   | Values                            | Default         |
+=========================+===============================+===================================+=================+
| ``<kind>``              | It defines the Topic's        | :class:`NO_KEY`,                  | :class:`NO_KEY` |
|                         | kind                          | :class:`WITH_KEY`                 |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<name>``              | It defines the Topic's        | ``string_255``                    |                 |
|                         | name. Must be unique.         |                                   |                 |
+-------------------------+-------------------------------+-----------------------------------+-----------------+
| ``<dataType>``          | It references the             | ``string_255``                    |                 |
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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-QOS<-->
    :end-before: <!--><-->

+--------------------------+----------------------------------+-------------------------------+------------------------+
| Name                     | Description                      | Values                        | Default                |
+==========================+==================================+===============================+========================+
| ``<durability>``         | It is defined in                 |:class:`VOLATILE`,             | :class:`VOLATILE`      |
|                          | :ref:`SettingDataDurability`     |:class:`TRANSIENT_LOCAL`       |                        |
|                          | section.                         |:class:`TRANSIENT`             |                        |
|                          |                                  |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<liveliness>``         | Defines the liveliness of the    | :ref:`liveliness-qos`         |                        |
|                          | publisher.                       |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<reliability>``        | It is defined in                 | :class:`RELIABLE`,            | :class:`RELIABLE`      |
|                          | :ref:`reliability` section.      | :class:`BEST_EFFORT`          |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<partition>``          | It allows the introduction of    |                               | ``List <string>``      |
|                          | a logical partition concept      |                               |                        |
|                          | inside the `physical` partition  |                               |                        |
|                          | induced by a domain.             |                               |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<deadline>``           | It is defined in                 |                               |                        |
|                          | :ref:`deadline-qos`              | Deadline period as a          | :class:`c_TimeInfinite`|
|                          | section.                         | :ref:`DurationType`           |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<lifespan>``           | It is defined in                 | Lifespan duration as a        | :class:`c_TimeInfinite`|
|                          | :ref:`lifespan-qos` section.     | :ref:`DurationType`           |                        |
+--------------------------+----------------------------------+-------------------------------+------------------------+
| ``<disablePositiveAcks>``| It is defined in                 |                               | It is disabled by      |
|                          | section                          |                               | default and            |
|                          | :ref:`disable-positive-acks-qos` |                               | ``duration`` is set    |
|                          |                                  |                               | to                     |
|                          |                                  |                               | :class:`c_TimeInfinite`|
+--------------------------+----------------------------------+-------------------------------+------------------------+

..
    .. note::

        - :class:`DURATION` means it expects a :ref:`DurationType`.

    ..  - :class:`DURABILITY_SERVICE` means that the label is a :ref:`DurabilityServiceType` block.::

        - :class:`LIVELINESS` means that the label is a :ref:`LiveLinessType` block.


    .. NOT YET SUPPORTED
        - ``<latencyBudget>``: Latency budget os the samples as :ref:`DurationType` within a ``<duration>`` tag.

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

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->XML_DURABILITYSERVICE<-->
       :end-before: <!--><-->

    - ``<history_kind>``: History handling kind. It accepts :class:`KEEP_LAST` and :class:`KEEP_ALL` values.

    - ``<history_depth>``: Allows establishing the depth of the history.

    - ``<max_samples>``: The maximum number of samples to be stored.

    - ``<max_instances>``: The maximum number of history instances.

    - ``<max_samples_per_instance>``: Allows establishing the maximum number of samples per history instance.

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

.. _CommonAlloc:

Allocation Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Allocation Configuration allows to control the allocation behavior of internal collections for which the number
of elements depends on the number of entities in the system.

For instance, there are collections inside a publisher which depend on the number of subscribers matching with it.

See :ref:`realtime-allocations` for detailed information on how to tune allocation related parameters.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<initial>``
     - Number of elements for which space is initially allocated.
     - ``UInt32``
     - 0
   * - ``<maximum>``
     - Maximum number of elements for which space will be allocated.
     - ``UInt32``
     - 0 (means no limit)
   * - ``<increment>``
     - Number of new elements that will be allocated when more space is necessary.
     - ``UInt32``
     - 1

.. _MessageMaxSize:

Submessage Size Limit
^^^^^^^^^^^^^^^^^^^^^

While some submessages have a fixed size (for example, SequenceNumber), others have a variable size depending on the
data they contain.
Processing a submessage requires having a memory chunk large enough to contain a copy of the submessage data.
That is easy to handle when dealing with fixed variable submessages, as size is known and memory can be allocated
beforehand.
For variable size submessages on the other hand, two different strategies can be used:

    - Set a maximum size for the data container, which will be allocated beforehand during the participant's setup.
      This avoids dynamic allocations during message communication.
      However, any submessages with a larger payload than the
      defined maximum will not fit in, and will therefore be discarded.
    - Do not set any maximum size for the data container, and instead allocate the required memory dynamically upon
      submessage arrival (according to the size declared on the submessage header).
      This allows for any size of submessages, at the cost of dynamic allocations during message decoding.

.. _mempol:

History Memory Policy Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Controls the allocation behavior of the change histories.

* **PREALLOCATED**: As the history gets larger, memory is allocated in chunks. Each chunk accommodates
  a number of changes, and no more allocations are done until that chunk is full. Provides minimum
  number of dynamic allocations at the cost of increased memory footprint. Maximum payload size of
  changes must be appropriately configured, as history will not be able to accommodate changes
  with larger payload after the allocation.
* **PREALLOCATED_WITH_REALLOC**: Like PREALLOCATED, but preallocated memory can be reallocated
  to accommodate changes with larger payloads than the defined maximum.
* **DYNAMIC**: Every change gets a fresh new allocated memory of the correct size.
  It minimizes the memory footprint, at the cost of increased number of dynamic allocations.
* **DYNAMIC_REUSABLE**: Like DYNAMIC, but instead of deallocating the memory when the change is removed
  from the history, it is reused for a future change, reducing the amount of dynamic allocations.
  If the new change has larger payload, it will be reallocated to accommodate the new size.

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
            <nanosec>0</nanosec>
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

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-EXAMPLE<-->
    :end-before: <!--><-->
    :lines: 2,4-
