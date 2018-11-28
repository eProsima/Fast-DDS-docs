.. _xml-profiles:

XML profiles
============

The :ref:`configuration` section shows how to configure entity attributes using XML profiles, 
but this section goes deeper on it, explaining each field with its available values and how to compound the complete XML files.

*eProsima Fast RTPS* permits to load as several XML files in the same execution, and they can contain several XML profiles.
An XML profile is defined by a unique name (or ``<transport_id>`` label
in the :ref:`transportdescriptors` case) that is used to reference the XML profile
when during the creation of a Fast RTPS entity, :ref:`comm-transports-configuration`, or :ref:`dynamic-types`.
*eProsima Fast RTPS* will also try to find in current execution path and
load an XML file with the name *DEFAULT_FASTRTPS_PROFILES.xml*, during its initialization.

Making an XML
-------------

An XML file can contain several XML profiles. Each profile can be divided into :ref:`transportdescriptors`,
:ref:`xmldynamictypes`, :ref:`participantprofiles`, :ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- PROFILES START -->
    :end-before: <!-- PROFILES END -->
    :dedent: 4

The Fast-RTPS XML format uses some structs along several profiles types.
For commodity, these common structs have been grouped in section :ref:`commonxml`.

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

.. code-block:: c++

   eprosima::fastrtps::Domain::loadXMLProfilesFile("my_profiles.xml");

   Participant *participant = Domain::createParticipant("participant_xml_profile");
   Publisher *publisher = Domain::createPublisher(participant, "publisher_xml_profile");
   Subscriber *subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");

To load dynamic types from its declaration through XML see the :ref:`Usage` section of :ref:`xmldynamictypes`.

.. _transportdescriptors:

Transport descriptors
---------------------

This section allows creating transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well-defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiated it will use or create the related transport.

The complete list of configurable parameters is shown in the following XML code:

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- TRANSPORTS START -->
    :end-before: <!-- TRANSPORTS END -->
    :dedent: 4

The XML label ``<transport_descriptors>`` can hold any number of ``<transport_descriptor>``.

- ``<transport_id>``: Unique name to identify each transport descriptor.

- ``<type>``: Type of the transport descriptor. The supported types are UDPv4, UDPv6, TCPv4, and TCPv6.

- ``<sendBufferSize>``: Size in bytes of the socket send buffer.

- ``<receiveBufferSize>``: Size in bytes of the socket receive buffer.

- ``<TTL>``: *Time To Live*, **only** for UDP transports.

- ``<maxMessageSize>``: The maximum size in bytes of the transport's message buffer.

- ``<maxInitialPeersRange>``: Establishes the maximum number of guessed initial peers to try to connect (default **4**).

- ``<interfaceWhiteList>``: Allows defining :ref:`whitelist-interfaces`.

- ``<wan_addr>``: Public WAN address when using **TCPv4 transports**. This field is optional if the transport doesn't need to define a WAN address.

- ``<output_port>``: Port used for output bound. If this field isn't defined, the output port will be random.

- ``<keep_alive_frequency_ms>``: Frequency in milliseconds for sending RTCP keepalive requests (**only** TCP).

- ``<keep_alive_timeout_ms>``: Time in milliseconds since sending the last keepalive request to consider a connection as broken. (**only** TCP).

- ``<max_logical_port>``: The maximum number of logical ports to try during RTCP negotiation (**only** TCP).

- ``<logical_port_range>``: The maximum number of logical ports per request to try during RTCP negotiation (**only** TCP).

- ``<logical_port_increment>``: Increment between logical ports to try during RTCP negotiation (**only** TCP).

- ``<ListeningPorts>``: Local port to work as TCP acceptor for input connections. If not set, the transport will work as TCP client only (**only** TCP).

There are more examples of transports descriptors in :ref:`comm-transports-configuration`.

.. _xmldynamictypes:

XML Dynamic Types
-----------------

XML Dynamic Types allows creating Dynamic Types to *eProsima Fast RTPS* directly defining them through XML.
It allows any application to change TopicDataTypes without modifying its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (``<types>``, types tag) can be placed similarly to the profiles tag inside the XML file.
It can be a stand-alone XML Types file or be a child of the Fast-RTPS XML root tag (``<dds>``).
Inside the types tag, there must be one or more type tags (``<type>``).

Stand-Alone:

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- STAND ALONE TYPES START -->
    :end-before: <!-- STAND ALONE TYPES END -->
    :dedent: 4

Rooted:

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- ROOTED TYPES START -->
    :end-before: <!-- ROOTED TYPES END -->
    :dedent: 4

Finally, each type of tag can contain one or more Type definition.
Defining several types inside a *type* tag or defining each type in its own *type* tag has the same result.

Type definition
^^^^^^^^^^^^^^^

**Enum**

The enum type is defined by its name and a set of literals, each of them with its name and its (optional) value.

Example:

    +-----------------------------------------------+-----------------------------------------------------------------------------------------------------------+
    | XML                                           | C++                                                                                                       |
    +===============================================+===========================================================================================================+
    | .. code-block:: xml                           | .. code-block:: c++                                                                                       |
    |                                               |                                                                                                           |
    |   <enum name="MyEnum">                        |     DynamicTypeBuilder_ptr enum_builder = DynamicTypeBuilderFactory::GetInstance()->CreateEnumBuilder();  |
    |       <literal name="A" value="0"/>           |     enum_builder->SetName("MyEnum");                                                                      |
    |       <literal name="B" value="1"/>           |     enum_builder->AddEmptyMember(0, "A");                                                                 |
    |       <literal name="C" value="2"/>           |     enum_builder->AddEmptyMember(1, "B");                                                                 |
    |   </enum>                                     |     enum_builder->AddEmptyMember(2, "C");                                                                 |
    |                                               |     DynamicType_ptr enum_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(enum_builder.get()); |
    |                                               |                                                                                                           |
    +-----------------------------------------------+-----------------------------------------------------------------------------------------------------------+

**Typedef**

The typedef type is defined by its name and its value or an inner element for complex types.
Typedefs correspond to Alias in Dynamic Types glossary.

Example:

    +-----------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+
    | XML                                           | C++                                                                                                                                           |
    +===============================================+===============================================================================================================================================+
    | .. code-block:: xml                           | .. code-block:: c++                                                                                                                           |
    |                                               |                                                                                                                                               |
    |   <typedef name="MyAlias1" value="MyEnum"/>   |     DynamicTypeBuilder_ptr alias_builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(enum_builder.get(), "MyAlias1");      |
    |                                               |     DynamicType_ptr alias_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(alias_builder.get());                                   |
    |                                               |                                                                                                                                               |
    |   <typedef name="MyAlias2">                   |     std::vector<uint32_t> sequence_lengths = { 2, 2 };                                                                                        |
    |       <long dimensions="2,2"/>                |     DynamicTypeBuilder_ptr int_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                                      |
    |   </typedef>                                  |     DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(int_builder.get(), sequence_lengths); |
    |                                               |     DynamicTypeBuilder_ptr alias_builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(array_builder.get(), "MyAlias2");     |
    |                                               |     DynamicType_ptr alias_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(alias_builder.get());                                   |
    |                                               |                                                                                                                                               |
    +-----------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+

**Struct**

The struct type is defined by its name and inner members.

Example:

    +-------------------------------------+----------------------------------------------------------------------------------------------------------------+
    | XML                                 | C++                                                                                                            |
    +=====================================+================================================================================================================+
    | .. code-block:: xml                 | .. code-block:: c++                                                                                            |
    |                                     |                                                                                                                |
    |   <struct name="MyStruct">          |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();      |
    |       <long name="first"/>          |     DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder(); |
    |       <longlong name="second"/>     |     DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();   |
    |   </struct>                         |                                                                                                                |
    |                                     |     struct_builder->SetName("MyStruct");                                                                       |
    |                                     |     struct_builder->AddMember(0, "first", long_builder);                                                       |
    |                                     |     struct_builder->AddMember(1, "second", long_long_builder);                                                 |
    |                                     |     DynamicType_ptr struct_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(struct_builder.get());  |
    |                                     |                                                                                                                |
    +-------------------------------------+----------------------------------------------------------------------------------------------------------------+


**Union**

The union type is defined by its name, a discriminator and a set of cases.
Each case has one or more caseValue and a member.


Example:

    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                    | C++                                                                                                                            |
    +========================================+================================================================================================================================+
    | .. code-block:: xml                    | .. code-block:: c++                                                                                                            |
    |                                        |                                                                                                                                |
    |   <union name="MyUnion">               |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                      |
    |       <discriminator type="octet"/>    |     DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();                 |
    |       <case>                           |     DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();                   |
    |           <caseValue value="0"/>       |     DynamicTypeBuilder_ptr octet_builder = DynamicTypeBuilderFactory::GetInstance()->CreateByteBuilder();                      |
    |           <caseValue value="1"/>       |     DynamicTypeBuilder_ptr union_builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(octet_builder.get());  |
    |            <long name="first"/>        |                                                                                                                                |
    |       </case>                          |     union_builder->SetName("MyUnion");                                                                                         |
    |       <case>                           |     union_builder->AddMember(0, "first", long_builder, "", { 0, 1 }, false);                                                   |
    |           <caseValue value="2"/>       |     union_builder->AddMember(1, "second", struct_builder, "", { 2 }, false);                                                   |
    |           <MyStruct name="second"/>    |     union_builder->AddMember(2, "third", long_long_builder, "", { }, true);                                                    |
    |       </case>                          |     DynamicType_ptr union_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(union_builder.get());                    |
    |       <case>                           |                                                                                                                                |
    |           <caseValue value="default"/> |                                                                                                                                |
    |           <longlong name="third"/>     |                                                                                                                                |
    |       </case>                          |                                                                                                                                |
    |   </union>                             |                                                                                                                                |
    |                                        |                                                                                                                                |
    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+


Member types
^^^^^^^^^^^^

Member types are any type that can belong to a struct or a union, or be aliased by a typedef.

When used as sequence's elements, key or value types of a map, as an aliased type, etc., its name attribute
is ignored and can be omitted.

**Basic types**

The tags of the available basic types are:

- ``<boolean>``
- ``<octet>``
- ``<char>``
- ``<wchar>``
- ``<short>``
- ``<long>``
- ``<longlong>``
- ``<unsignedshort>``
- ``<unsignedlong>``
- ``<unsignedlonglong>``
- ``<float>``
- ``<double>``
- ``<longdouble>``
- ``<string>``
- ``<wstring>``
- ``<boundedString>``
- ``<boundedWString>``

All of them are defined simply:

    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                    | C++                                                                                                                            |
    +========================================+================================================================================================================================+
    | .. code-block:: xml                    | .. code-block:: c++                                                                                                            |
    |                                        |                                                                                                                                |
    |   <longlong name="my_long"/>           |     DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();                 |
    |                                        |     long_long_builder->SetName("my_long");                                                                                     |
    |                                        |     DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(long_long_builder.get());            |
    |                                        |                                                                                                                                |
    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+

Except for ``<boundedString>`` and ``<boundedWString>`` that should include an inner element :class:`maxLength`
whose value indicates the maximum length of the string.

    +--------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                        | C++                                                                                                                            |
    +============================================+================================================================================================================================+
    | .. code-block:: xml                        | .. code-block:: c++                                                                                                            |
    |                                            |                                                                                                                                |
    |   <boundedString name="my_large_string">   |     DynamicTypeBuilder_ptr string_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(41925);              |
    |       <maxLength value="41925"/>           |     string_builder->SetName("my_large_string");                                                                                |
    |   </boundedString>                         |     DynamicType_ptr string_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(string_builder.get());                  |
    |                                            |                                                                                                                                |
    |   <boundedWString name="my_large_wstring"> |     DynamicTypeBuilder_ptr wstring_builder = DynamicTypeBuilderFactory::GetInstance()->CreateWstringBuilder(20925);            |
    |       <maxLength value="20925"/>           |     wstring_builder->SetName("my_large_wstring");                                                                              |
    |   </boundedWString>                        |     DynamicType_ptr wstring_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(wstring_builder.get());                |
    |                                            |                                                                                                                                |
    +--------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+

**Arrays**

Arrays are defined exactly the same way as any other member type, but adds the attribute :class:`dimensions`.
The format of this dimensions attribute is the size of each dimension separated by commas.

Example:


    +--------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------+
    | XML                                              | C++                                                                                                                                    |
    +==================================================+========================================================================================================================================+
    | .. code-block:: xml                              | .. code-block:: c++                                                                                                                    |
    |                                                  |                                                                                                                                        |
    |   <long name="long_array" dimensions="2,3,4"/>   |     std::vector<uint32_t> lengths = { 2, 3, 4 };                                                                                       |
    |                                                  |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                              |
    |                                                  |     DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(long_builder.get(), lengths);  |
    |                                                  |     array_builder->SetName("long_array");                                                                                              |
    |                                                  |     DynamicType_ptr array_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(array_builder.get());                            |
    |                                                  |                                                                                                                                        |
    +--------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------+


It's IDL analog would be:

.. code-block:: c++

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its :class:`name`, its content :class:`type` and its (optional) :class:`length`.
The type of its content can be defined by its :class:`type` attribute or by a member type.

Example:

    +-------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------+
    | XML                                                   | C++                                                                                                                                     |
    +=======================================================+=========================================================================================================================================+
    | .. code-block:: xml                                   | .. code-block:: c++                                                                                                                     |
    |                                                       |                                                                                                                                         |
    |   <sequence name="my_sequence_sequence" length="3">   |     uint32_t child_len = 2;                                                                                                             |
    |       <sequence type="long" length="2"/>              |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                               |
    |   </sequence>                                         |     DynamicTypeBuilder_ptr seq_builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(long_builder.get(), child_len);|
    |                                                       |     uint32_t length = 3;                                                                                                                |
    |                                                       |     DynamicTypeBuilder_ptr seq_seq_builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(seq_builder.get(),length); |
    |                                                       |     seq_seq_builder->SetName("my_sequence_sequence");                                                                                   |
    |                                                       |     DynamicType_ptr seq_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(seq_seq_builder.get());                             |
    |                                                       |                                                                                                                                         |
    +-------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------+

The example shows a sequence with length 3 of sequences with length 2 with long contents.
As IDL would be:

.. code-block:: c++

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner (or content) sequence has no name, as it would be ignored by the parser.

**Maps**

Maps are similar to sequences but they need to define two types instead one.
One for its :class:`key_type` and anotherfor its :class:`value_type`.
Again, both types can be defined as attributes or as members, but in this cases, when defined
as members, they are content in another XML element ``<key_type>`` and ``<value_type>`` respectively.

The definition kind of each type can be mixed, this is, one type can be defined as an attribute and the
other as a member.

Example:

    +---------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------+
    | XML                                                           | C++                                                                                                                         |
    +===============================================================+=============================================================================================================================+
    | .. code-block:: xml                                           | .. code-block:: c++                                                                                                         |
    |                                                               |                                                                                                                             |
    |   <map name="my_map_map" key_type="long" length="2">          |     uint32_t length = 2;                                                                                                    |
    |       <value_type>                                            |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                   |
    |           <map key_type="long" value_type="long" length="2"/> |     DynamicTypeBuilder_ptr map_builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(long_builder.get(),     |
    |       </value_type>                                           |     long_builder.get(), length);                                                                                            |
    |   </map>                                                      |                                                                                                                             |
    |                                                               |     DynamicTypeBuilder_ptr map_map_builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(long_builder.get(), |
    |                                                               |     map_builder.get(), length);                                                                                             |
    |                                                               |     map_map_builder->SetName("my_map_map");                                                                                 |
    |                                                               |     DynamicType_ptr map_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(map_map_builder.get());                 |
    |                                                               |                                                                                                                             |
    +---------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------+

Is equivalent to the IDL:

.. code-block:: c++

    map<long,map<long,long,2>,2> my_map_map;

**Complex types**

Once defined, complex types can be used as members in the same way a basic or array type would be.

Example:

.. code-block:: xml

    <struct name="OtherStruct">
        <MyEnum name="my_enum"/>
        <MyStruct name="my_struct" dimensions="5"/>
    </struct>

.. _Usage:

Usage
^^^^^

In the application that will make use of XML Types, it's mandatory to load the XML file that defines the types before trying to instantiate DynamicPubSubTypes of these types.
It's important to remark that only Structs generate usable DynamicPubSubType instances.

.. code-block:: cpp

    // Load the XML File
    XMLP_ret ret = XMLProfileManager::loadXMLFile("types.xml");
    // Create the "MyStructPubSubType"
    DynamicPubSubType *pbType = XMLProfileManager::CreateDynamicPubSubType("MyStruct");
    // Create a "MyStruct" instance
    DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(pbType->GetDynamicType());

.. _participantprofiles:

Participant profiles
--------------------

Participant profiles allow declaring :ref:`participantconfiguration` from an XML file and 
the configuration options for the participant belongs to the ``<rtps>`` label.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile in order to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- PARTICIPANT START -->
    :end-before: <!-- PARTICIPANT END -->
    :dedent: 4

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

This is the list of each possible configuration parameter:

- ``<name>``: Participant's name. It's not the same that ``profile_name``.

- ``<defaultUnicastLocatorList>``: List of default input unicast locators. It expects a :ref:`LocatorListType`.

- ``<defaultMulticastLocatorList>``: List of default input multicast locators. It expects a :ref:`LocatorListType`.

- ``<sendSocketBufferSize>``: Size in bytes of the output socket buffer.

- ``<listenSocketBufferSize>``: Size in bytes of the input socket buffer.

- ``<builtin>``: Built-in parameters. Explained in the :ref:`builtin` section.

- ``<port>``: Allows defining the port parameters and gains related to the RTPS protocol. It has several subfields:

    * ``<portBase>``: Base ``port`` (*default 7400*).

    * ``<domainIDGain>``: Gain in ``domainId`` (*default 250*).

    * ``<participantIDGain>``: Gain in ``participantId`` (*default 2*).

    * ``<offsetd0>``: Multicast metadata offset (*default 0*).

    * ``<offsetd1>``: Unicast metadata offset (*default 10*).

    * ``<offsetd2>``: Multicast user data offset (*default 1*).

    * ``<offsetd3>``: Unicast user data offset (*default 11*).

- ``<userData>``: Allows adding custom information.

- ``<participantID>``: Participant's identifier. Typically it will be autogenerated by the ``Domain``.

- ``<throughputController>``: Allows defining a maximum throughput:

    * ``<bytesPerPeriod>``: The maximum bytes to send by period.
    * ``<periodMillisecs>``: Period in milliseconds.

- ``<userTransports>``: Transport descriptors to be used by the participant, as a list of ``<id>``.

    .. code-block:: xml

        <id>TransportId1</id> <!-- string -->
        <id>TransportId2</id> <!-- string -->

- ``<useBuiltinTransports>``: Boolean field to indicate to the system that the participant will use the default builtin transport independently of its ``<userTransports>``.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.


.. _builtin:

Built-in parameters
^^^^^^^^^^^^^^^^^^^

This section of the :class:`Participant's rtps` configuration allows defining built-in parameters.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- BUILTIN START -->
    :end-before: <!-- BUILTIN END -->
    :dedent: 4

- ``<use_SIMPLE_RTPS_PDP>``: Boolean attribute to establish if the participant must use the simple RTPS discovery protocol.

- ``<use_WriterLivelinessProtocol>``: Boolean attribute to establish the usage of the writer liveliness protocol.

- ``<EDP>``: It establishes the type of EDP protocol. It can take :class:`SIMPLE` or :class:`STATIC` values.

- ``<domainId>``: Sets the domain identifier.

- ``<leaseDuration>``: :ref:`DurationType` to set duration of lease period.

- ``<leaseAnnouncement>``: :ref:`DurationType` to set announcement of lease period.

- ``<simpleEDP>``: If :class:`EDP` is set to :class:`SIMPLE`, allows configuring the use of :class:`PUBWRITER_SUBREADER` and :class:`PUBREADER_SUBWRITER`.

    * ``<PUBWRITER_SUBREADER>``: Boolean value to determine if :class:`PUBWRITER_SUBREADER` must be used.

    * ``<PUBREADER_SUBWRITER>``: Boolean value to determine if :class:`PUBREADER_SUBWRITER` must be used.

- ``<metatrafficUnicastLocatorList>``: List of metatraffic unicast locators. It expects a :ref:`LocatorListType`.

- ``<metatrafficMulticastLocatorList>``: List of metatraffic multicast locators. It expects a :ref:`LocatorListType`.

- ``<initialPeersList>``: List of initial peers locators. It expects a :ref:`LocatorListType`.

- ``<staticEndpointXMLFilename>``: If :class:`EDP` is set to :class:`STATIC`, allows setting the XML file path that contains the endpoint configuration.

- ``<readerHistoryMemoryPolicy>``: Memory allocation kind for reader's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<writerHistoryMemoryPolicy>``: Memory allocation kind for writer's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.


.. _publisherprofiles:

Publisher profiles
------------------

Publisher profiles allow declaring :ref:`Publisher configuration <pubsubconfiguration>` from XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in the :ref:`loadingapplyingprofiles` section.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- PUBLISHER START -->
    :end-before: <!-- PUBLISHER END -->
    :dedent: 4

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

- ``<topic>``: :ref:`TopicType` configuration of the subscriber.

- ``<qos>``: Subscriber :ref:`CommonQOS` configuration.

- ``<times>``:  Allows configuring some time related parameters of the subscriber:

    * ``<initialAcknackDelay>``: :ref:`DurationType` of the initial :class:`Acknack` message.

    * ``<heartbeatResponseDelay>``: :ref:`DurationType` to set the delay of the :class:`heartbeat` message response.

- ``<unicastLocatorList>``: List of unicast input locators. It expects a :ref:`LocatorListType`.

- ``<multicastLocatorList>``: List of multicast input locators. It expects a :ref:`LocatorListType`.

- ``<outLocatorList>``:  List of output locators. It expects a :ref:`LocatorListType`.

- ``<throughputController>``: Limits the output bandwidth of the publisher.

- ``<historyMemoryPolicy>``: Memory allocation kind for subscriber's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.

- ``<userDefinedID>``: Allows setting a custom identifier.

- ``<entityID>``: Allows establishing the entityID of the subscriber.


.. _subscriberprofiles:

Subscriber profiles
-------------------

Subscriber profiles allows declaring :ref:`Subscriber configuration <pubsubconfiguration>` from XML file.
The attribute ``profile_name`` is the name that the ``Domain`` associates to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- SUBSCRIBER START -->
    :end-before: <!-- SUBSCRIBER END -->
    :dedent: 4

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

- ``<topic>``: :ref:`TopicType` configuration of the subscriber.

- ``<qos>``: Subscriber :ref:`CommonQOS` configuration.

- ``<times>``:  Allows configuring some time related parameters of the subscriber:

    * ``<initialAcknackDelay>``: :ref:`DurationType` of the initial :class:`Acknack` message.

    * ``<heartbeatResponseDelay>``: :ref:`DurationType` to set the delay of the :class:`heartbeat` message response.

- ``<unicastLocatorList>``: List of unicast locators. It expects a :ref:`LocatorListType`.

- ``<multicastLocatorList>``: List of multicast locators. It expects a :ref:`LocatorListType`.

- ``<outLocatorList>``:  List of output locators. It expects a :ref:`LocatorListType`.

- ``<expectsInlineQos>``: Boolean parameter to indicate if QOS is expected inline.

- ``<historyMemoryPolicy>``: Memory allocation kind for subscriber's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.

- ``<userDefinedID>``: Allows setting a custom identifier.

- ``<entityID>``: Allows establishing the entityID of the subscriber.


.. _commonxml:

Common
------

In the above profiles, some types are used in several different places. To avoid too many details, some of that
places have a tag like :class:`LocatorListType` that indicates that field is defined in this section.

This is the complete list of common types:

.. _LocatorListType:

LocatorListType
^^^^^^^^^^^^^^^

It represents a list of :class:`Locator_t`.
LocatorListType is normally used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that expect a list of locators and give it sense,
for example, in ``<defaultUnicastLocatorList>``:

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- LOCATOR LIST START -->
    :end-before: <!-- LOCATOR LIST END -->
    :dedent: 4

In this example, there are three different locators in ``<defaultUnicastLocatorList>``.

Let's see each Locator's fields in detail:

- ``<kind>``: Type of the Locator. It can be UDPv4, UDPv6, TCPv4, and TCPv6.

- ``<port>``: Physical port number of the locator.

- ``<port_>``: Allows managing low-level detail in ports of TCP locators, allowing set both :``<physical_port>`` and ``<logical_port>``.

- ``<address>``: IPv4 address of the locator.

- ``<addresses_>``: Allows managing low-level details in address of TCP locators (``<unique_lan_id>``, ``<wan_address>`` and ``<ip_address>``).

- ``<ipv6_address>``: IPv6 address of the locator.

.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType (XML label ``<propertiesPolicy>``) allows defining a set of generic properties.
It's useful at defining extended or custom configuration parameters.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- PROPERTIES POLICY START -->
    :end-before: <!-- PROPERTIES POLICY END -->
    :dedent: 4

- ``<name>``: Name to identify the property.

- ``<value>``: Property's value.

- ``<propagate>``: Boolean value that indicates if the property is going to be serialized along with the object it belongs to.

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time and it's commonly used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that give it sense, like ``<leaseAnnouncement>`` or ``<leaseDuration>``.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- DURATION POLICY START -->
    :end-before: <!-- DURATION POLICY END -->
    :dedent: 4

Duration time can be defined through a constant value directly (:class:`INFINITE`, :class:`ZERO`, or :class:`INVALID`),
or by ``<seconds>`` plus ``<fraction>`` labels:

- :class:`INFINITE`: Constant value, represents an infinite period of time.

- :class:`ZERO`: Constant value, represents 0.0 seconds.

- :class:`INVALID`: Constant value, represents an invalid period of time.

- ``<seconds>``: Integer seconds value.

- ``<fraction>``: Fractions of a second. A fraction is :class:`1/(2^32)` seconds.

.. _TopicType:

Topic Type
^^^^^^^^^^^^^^^^^^^^

The topic name and data type are used as meta-data to determine whether Publishers and Subscribers can exchange messages.
There is a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- TOPIC START -->
    :end-before: <!-- TOPIC END -->
    :dedent: 4

- ``<kind>``: String field that sets if the topic uses keys or not. The available values are: *NO_KEY* and *WITH_KEY*.

- ``<name>``: Name of the topic.

- ``<dataType>``: Name of the data type.

- ``<historyQos>``: The history QoS handles the number of messages that are going to be stored by publishers and subscribers in their histories.

    * ``<kind>``: History type, the available values are: *KEEP_ALL* and *KEEP_LAST*.

    * ``<depth>``: Number of packages that can be stored with the *KEEP_LAST* option.

- ``<resourceLimitsQos>``: The :ref:`history QoS <resourceLimits-qos>`

    * ``<max_samples>``: The maximum number of samples that can be stored in the history of publishers or subscribers. Its default value is 5000.

    * ``<max_instances>``: The maximum number of instances that a publisher or a subscriber can manage. Its default value is 10.

    * ``<max_samples_per_instance>``: The maximum number of samples for each instance. Its default value is 400.

    * ``<allocated_samples>``: Initial samples reserved in the history of publishers or subscribers.

.. _CommonQOS:

QOS
^^^

The quality of service (QoS) handles the restrictions applied to the application.

.. AFTER DURABILITY
    <durabilityService>
        <!-- DURABILITY_SERVICE -->
    </durabilityService>

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- QOS START -->
    :end-before: <!-- QOS END -->
    :dedent: 4

.. note::

    - :class:`DURATION` means it expects a :ref:`DurationType`.

..  - :class:`DURABILITY_SERVICE` means that the label is a :ref:`DurabilityServiceType` block.::

    - :class:`LIVELINESS` means that the label is a :ref:`LiveLinessType` block.

- ``<durability>``: is defined on :ref:`SettingDataDurability` section.

- ``<reliability>``: is defined on :ref:`reliability` section.

- ``<ownership>``: ``<kind>`` determines whether an instance of the Topic is owned by a single Publisher. If the selected ownership is :class:`EXCLUSIVE` the Publisher will use the Ownership strength value as the strength of its publication. Only the publisher with the highest strength can publish in the same Topic with the same Key.

- ``<destinationOrder>``: ``<kind>`` determines the destination timestamp. :class:`BY_RECEPTION_TIMESTAMP` for reception and :class:`BY_SOURCE_TIMESTAMP` for the source.

- ``<presentation>``:
    * ``<access_scope>`` defines the scope of presentation and can be :class:`INSTANCE`, :class:`TOPIC`, or :class:`GROUP`.

    * ``<coherent_access>`` Boolean value to set if the access must be coherent.

    * ``<ordered_access>`` Boolean value to set if the access must be ordered.

.. .. _DurabilityServiceType:

..
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

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- LIVELINESS START -->
    :end-before: <!-- LIVELINESS END -->
    :dedent: 4

- ``<kind>``: Specifies how to manage liveliness. Can take values :class:`AUTOMATIC`, :class:`MANUAL_BY_PARTICIPANT`, and :class:`MANUAL_BY_TOPIC`.

- ``<leaseDuration>``: How much time remote RTPSParticipants should consider this RTPSParticipant alive the lease is being announced. It is a :ref:`DurationType`.

- ``<announcement_period>``: The period to send its Discovery Message to all other discovered RTPSParticipants as well as to all Multicast ports. It's a :ref:`DurationType`.

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

.. literalinclude:: xmlprofiles.xml
    :language: xml
    :start-after: <!-- EXAMPLE START -->
    :end-before: <!-- EXAMPLE END -->
    :dedent: 4
