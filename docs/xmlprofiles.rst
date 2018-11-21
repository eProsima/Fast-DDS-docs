.. _xml-profiles:

XML profiles
============

In the :ref:`configuration` section you could see how to configure entity attributes using XML profiles,
but this section goes deeper into it.

XML profiles are loaded from XML files. *eProsima Fast RTPS* permits to load as much XML files as you want. An XML file
can contain several XML profiles. An XML profile is defined by a unique name (or :class:`<transport_id>` label
in the :ref:`transportdescriptors` case) that is used to reference the XML profile
when you create a Fast RTPS entity, :ref:`comm-transports-configuration`, or :ref:`dynamic-types`.
*eProsima Fast RTPS* will also try to find in current execution path and
load an XML file with the name *DEFAULT_FASTRTPS_PROFILES.xml*.
If this file exists, it is loaded at the library initialization.

Making an XML
-------------

An XML file can contain several XML profiles. They can be divided into :ref:`transportdescriptors`,
:ref:`xmldynamictypes`, :ref:`participantprofiles`, :ref:`publisherprofiles`, and :ref:`subscriberprofiles`.

.. code-block:: xml

    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>TransportProfile</transport_id>
                ....
            </transport_descriptor>
        <transport_descriptors>

        <types>
            <type>
                <struct name="struct_profile">
                    ....
                </struct>
            </type>
        </types>

        <participant profile_name="participant_profile">
            ....
        </participant>

        <publisher profile_name="publisher_profile">
            ....
        </publisher>

        <subscriber profile_name="subscriber_profile">
            ....
        </subscriber>
    </profiles>

The Fast-RTPS XML format uses some structs along several profiles types.
For commodity, these common structs have been grouped in section :ref:`commonxml`.

Finally, an XML example file with all possibilities being used can be found at :ref:`examplexml`.
This example is useful as a quick reference when you want to look for some particular property.
This `XSD file <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/fastRTPS_profiles.xsd>`_ can be used
as a quick reference too.

.. _loadingapplyingprofiles:

Loading and applying profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before creating any entity, you can load XML files using ``Domain::loadXMLProfilesFile`` function.
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

This section allows us to create transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well-defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiated it will use or create the described transport.

The complete list of configurable parameters is shown in the following XML code:

.. code-block:: xml

    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>TransportId1</transport_id> <!-- string -->
                <type>UDPv4</type> <!-- string -->
                <sendBufferSize>8192</sendBufferSize> <!-- uint32 -->
                <receiveBufferSize>8192</receiveBufferSize> <!-- uint32 -->
                <TTL>250</TTL> <!-- uint8 -->
                <maxMessageSize>16384</maxMessageSize> <!-- uint32 -->
                <maxInitialPeersRange>100</maxInitialPeersRange> <!-- uint32 -->
                <interfaceWhiteList>
                    <id>192.168.1.41</id> <!-- string -->
                    <id>127.0.0.1</id> <!-- string -->
                </interfaceWhiteList>
                <wan_addr>80.80.55.44</wan_addr> <!-- string -->
                <output_port>5101</output_port> <!-- uint16 -->
                <keep_alive_frequency_ms>5000</keep_alive_frequency_ms> <!-- uint32 -->
                <keep_alive_timeout_ms>25000</keep_alive_timeout_ms> <!-- uint32 -->
                <max_logical_port>9000</max_logical_port> <!-- uint16 -->
                <logical_port_range>100</logical_port_range> <!-- uint16 -->
                <logical_port_increment>2</logical_port_increment> <!-- uint16 -->
                <ListeningPorts>
                    <port>5100</port> <!-- uint16 -->
                    <port>5200</port> <!-- uint16 -->
                </ListeningPorts>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>TransportId2</transport_id>
                ...
            </transport_descriptor>
        </transport_descriptors>
    </profiles>

The XML label :class:`<transport_descriptors>` can hold any number of :class:`<transport_descriptor>`.

- ``<transport_id>``: Unique name to identify each transport descriptor.

- ``<type>``: Type of the transport descriptor. The supported types are UDPv4, UDPv6, TCPv4, and TCPv6.

- ``<sendBufferSize>``: Size, in bytes, of the socket send buffer.

- ``<receiveBufferSize>``: Size, in bytes, of the socket receive buffer.

- ``<TTL>``: *Time To Live*, **only** for UDP transports.

- ``<maxMessageSize>``: Maximum size in bytes of the transport message buffer.

- ``<maxInitialPeersRange>``: Establishes the maximum number of guessed initial peers to try to connect (default **4**).

- ``<interfaceWhiteList>``: Allows you to define :ref:`whitelist-interfaces`.

- ``<wan_addr>``: Allows you to declare the public WAN address when using **TCPv4 transports**.

- ``<output_port>``: Port used for output bound, instead of a random one.

- ``<keep_alive_frequency_ms>``: Frequency in milliseconds for sending RTCP keep alive requests (**only** TCP).

- ``<keep_alive_timeout_ms>``: Time in milliseconds to consider a connection is broken since the last keep alive requests were sent (**only** TCP).

- ``<max_logical_port>``: Maximum number of logical ports to try during RTCP negotiation (**only** TCP).

- ``<logical_port_range>``: Maximum number of logical ports per request to try during RTCP negotiation (**only** TCP).

- ``<logical_port_increment>``: Increment between logical ports to try during RTCP negotiation (**only** TCP).

- ``<ListeningPorts>``: Local port to work as TCP acceptor for input connections. If not set, the transport will work as TCP client only (**only** TCP).

You can see more examples in :ref:`comm-transports-configuration`.

.. _xmldynamictypes:

XML Dynamic Types
-----------------

XML Dynamic Types allows eProsima Fast RTPS to create Dynamic Types directly defining them through XML.
This allows any application to change TopicDataTypes without the need to change its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (``<types>``, types tag) in the XML file can be placed similarly to the profiles tag.
It can be a stand-alone XML Types file or be a child of the fastrtps XML root tag (``<dds>``).
Inside the types tag, must be one or more type tags (``<type>``).

Stand-Alone:

.. code-block:: xml

    <types>
        <type>
            [Type definition]
        </type>
        <type>
            [Type definition]
            [Type definition]
        </type>
    </types>

Rooted:

.. code-block:: xml

    <dds>
        <types>
            <type>
                [Type definition]
            </type>
            <type>
                [Type definition]
                [Type definition]
            </type>
        </types>
    </dds>

Finally, each type tag can contain one or more Type definition.
Define several types inside a type tag or each type in its own type tag has the same result.

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

By member types, we refer to any type that can belong to a struct or a union, or be aliased by a typedef.

When used as sequence's elements, key or value types of a map, as an aliased type, etc., its name attribute
is ignored and can be omitted.

**Basic types**

The available basic types XML tags are:

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

In the application that will make use of XML Types, we need to load the XML file that defines our types,
and then, simply instantiate DynamicPubSubTypes of our desired type.

Remember that only Structs generate usable DynamicPubSubType instances.

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

Participant profiles allow you to declare :ref:`participantconfiguration` from an XML file.
The configuration options for the participant belongs to the :class:`<rtps>` label.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile in order to load it
as shown in :ref:`loadingapplyingprofiles`.

.. code-block:: xml

    <participant profile_name="part_profile_name">
        <rtps>
            <name>Participant Name</name> <!-- String -->

            <defaultUnicastLocatorList>
                <!-- LOCATOR_LIST -->
            </defaultUnicastLocatorList>

            <defaultMulticastLocatorList>
                <!-- LOCATOR_LIST -->
            </defaultMulticastLocatorList>

            <sendSocketBufferSize>8192</sendSocketBufferSize> <!-- uint32 -->

            <listenSocketBufferSize>8192</listenSocketBufferSize>  <!-- uint32 -->

            <builtin>
                <!-- BUILTIN -->
            </builtin>

            <port>
                <portBase>7400</portBase> <!-- uint16 -->
                <domainIDGain>200</domainIDGain> <!-- uint16 -->
                <participantIDGain>10</participantIDGain> <!-- uint16 -->
                <offsetd0>0</offsetd0> <!-- uint16 -->
                <offsetd1>1</offsetd1> <!-- uint16 -->
                <offsetd2>2</offsetd2> <!-- uint16 -->
                <offsetd3>3</offsetd3> <!-- uint16 -->
            </port>

            <userData> <!-- octetVector (string) -->  </userData>

            <participantID>99</participantID>   <!-- int32 -->

            <throughputController>
                <bytesPerPeriod>8192</bytesPerPeriod> <!-- uint32 -->
                <periodMillisecs>1000</periodMillisecs> <!-- uint32 -->
            </throughputController>

            <userTransports>
                <id>TransportId1</id> <!-- string -->
                <id>TransportId2</id> <!-- string -->
            </userTransports>

            <useBuiltinTransports>FALSE</useBuiltinTransports> <!-- boolean -->

            <propertiesPolicy>
                <!-- PROPERTIES_POLICY -->
            </propertiesPolicy>
        </rtps>
    </participant>

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

Now, we are going to explain each possible configuration parameter:

- ``<name>``: Participant's name. Don't confuse it with ``profile_name``.

- ``<defaultUnicastLocatorList>``: List of default unicast locators. It expects a :ref:`LocatorListType`.

- ``<defaultMulticastLocatorList>``: List of default multicast locators. It expects a :ref:`LocatorListType`.

- ``<sendSocketBufferSize>``: Size in bytes of the output socket buffer.

- ``<listenSocketBufferSize>``: Size in bytes of the input socket buffer.

- ``<builtin>``: Built-in parameters. Explained in the :ref:`builtin` section.

- ``<port>``: Allows you to define the port parameters and gains related to the RTPS protocol. It has several subfields:

    * ``<portBase>``: Base ``port`` (*default 7400*).

    * ``<domainIDGain>``: Gain in ``domainId`` (*default 250*).

    * ``<participantIDGain>``: Gain in ``participantId`` (*default 2*).

    * ``<offsetd0>``: Multicast metadata offset (*default 0*).

    * ``<offsetd1>``: Unicast metadata offset (*default 10*).

    * ``<offsetd2>``: Multicast user data offset (*default 1*).

    * ``<offsetd3>``: Unicast user data offset (*default 11*).

- ``<userData>``: Allows you to add your own information.

- ``<participantID>``: Allows you to set the participant's identifier. Typically it will be autogenerated by the ``Domain``.

- ``<throughputController>``: Allows you to define a maximum throughput:

    * ``<bytesPerPeriod>``: Maximum bytes to send by period.
    * ``<periodMillisecs>``: Period in milliseconds.

- ``<userTransports>``: Allows you to add transport descriptors to be used by the participant, as a list of :class:`<id>`.

    .. code-block:: xml

        <id>TransportId1</id> <!-- string -->
        <id>TransportId2</id> <!-- string -->

- ``<useBuiltinTransports>``: Boolean field to indicate to the system that the participant will use the default builtin transport independently of its :class:`<userTransports>`.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.


.. _builtin:

Built-in parameters
^^^^^^^^^^^^^^^^^^^

This section of the :class:`Participant's rtps` configuration allows you to define built-in parameters.

.. code-block:: xml

    <builtin>
        <use_SIMPLE_RTPS_PDP>FALSE</use_SIMPLE_RTPS_PDP> <!-- boolean -->

        <use_WriterLivelinessProtocol>FALSE</use_WriterLivelinessProtocol>  <!-- boolean -->

        <EDP>SIMPLE</EDP> <!-- string -->

        <domainId>4</domainId> <!-- uint32 -->

        <leaseDuration>
            <!-- DURATION -->
        </leaseDuration>

        <leaseAnnouncement>
            <!-- DURATION -->
        </leaseAnnouncement>

        <simpleEDP>
            <PUBWRITER_SUBREADER>TRUE</PUBWRITER_SUBREADER> <!-- boolean -->
            <PUBREADER_SUBWRITER>TRUE</PUBREADER_SUBWRITER> <!-- boolean -->
        </simpleEDP>

        <metatrafficUnicastLocatorList>
            <!-- LOCATOR_LIST -->
        </metatrafficUnicastLocatorList>

        <metatrafficMulticastLocatorList>
            <!-- LOCATOR_LIST -->
        </metatrafficMulticastLocatorList>

        <initialPeersList>
            <!-- LOCATOR_LIST -->
        </initialPeersList>

        <staticEndpointXMLFilename>filename.xml</staticEndpointXMLFilename> <!-- string -->

        <readerHistoryMemoryPolicy>PREALLOCATED_WITH_REALLOC</readerHistoryMemoryPolicy>

        <writerHistoryMemoryPolicy>PREALLOCATED_WITH_REALLOC</writerHistoryMemoryPolicy>
    </builtin>

- ``<use_SIMPLE_RTPS_PDP>``: Boolean attribute to establish if simple RTPS participant discovery protocol must be used.

- ``<use_WriterLivelinessProtocol>``: Boolean attribute to establish the usage of the writer liveliness protocol.

- ``<EDP>``: It establishes the type of EDP protocol. It can take :class:`SIMPLE` or :class:`STATIC` values.

- ``<domainId>``: Sets the domain identifier.

- ``<leaseDuration>``: :ref:`DurationType` to set duration of lease period.

- ``<leaseAnnouncement>``: :ref:`DurationType` to set announcement of lease period.

- ``<simpleEDP>``: If :class:`EDP` is set to :class:`SIMPLE`, allows you to configure the use of :class:`PUBWRITER_SUBREADER` and :class:`PUBREADER_SUBWRITER`.

    * ``<PUBWRITER_SUBREADER>``: Boolean value to determine if :class:`PUBWRITER_SUBREADER` must be used.

    * ``<PUBREADER_SUBWRITER>``: Boolean value to determine if :class:`PUBREADER_SUBWRITER` must be used.

- ``<metatrafficUnicastLocatorList>``: List of metatraffic unicast locators. It expects a :ref:`LocatorListType`.

- ``<metatrafficMulticastLocatorList>``: List of metatraffic multicast locators. It expects a :ref:`LocatorListType`.

- ``<initialPeersList>``: List of initial peers locators. It expects a :ref:`LocatorListType`.

- ``<staticEndpointXMLFilename>``: If :class:`EDP` is set to :class:`STATIC`, allows you to set the XML file path that contains the endpoint configuration.

- ``<readerHistoryMemoryPolicy>``: Policy of memory allocation for reader's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<writerHistoryMemoryPolicy>``: Policy of memory allocation for writer's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.


.. _publisherprofiles:

Publisher profiles
------------------

Publisher profiles allows you to declare :ref:`Publisher configuration <pubsubconfiguration>` from XML file.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile in order to load it
as shown in :ref:`loadingapplyingprofiles`.

.. code-block:: xml

    <publisher profile_name="part_profile_name">
        <topic>
            <!-- TOPIC_TYPE -->
        </topic>

        <qos>
            <!-- QOS -->
        </qos>

        <times> <!-- readerTimesType -->
            <initialAcknackDelay>
                <!-- DURATION -->
            </initialAcknackDelay>
            <heartbeatResponseDelay>
                <!-- DURATION -->
            </heartbeatResponseDelay>
        </times>

        <unicastLocatorList>
            <!-- LOCATOR_LIST -->
        </unicastLocatorList>

        <multicastLocatorList>
            <!-- LOCATOR_LIST -->
        </multicastLocatorList>

        <outLocatorList>
            <!-- LOCATOR_LIST -->
        </outLocatorList>

        <throughputController>
            <bytesPerPeriod>8192</bytesPerPeriod> <!-- uint32 -->
            <periodMillisecs>1000</periodMillisecs> <!-- uint32 -->
        </throughputController>

        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>

        <propertiesPolicy>
            <!-- PROPERTIES_POLICY -->
        </propertiesPolicy>

        <userDefinedID>55</userDefinedID> <!-- Int16 -->

        <entityID>66</entityID> <!-- Int16 -->
    </publisher>

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

- ``<topic>``: :ref:`TopicType` configuration of the subscriber.

- ``<qos>``: Subscriber :ref:`CommonQOS` configuration.

- ``<times>``:  Allows you to configure some time related parameters of the subscriber:

    * ``<initialAcknackDelay>``: :ref:`DurationType` of the initial :class:`Acknack` message.

    * ``<heartbeatResponseDelay>``: :ref:`DurationType` to set the delay of the :class:`heartbeat` message response.

- ``<unicastLocatorList>``: List of unicast locators. It expects a :ref:`LocatorListType`.

- ``<multicastLocatorList>``: List of multicast locators. It expects a :ref:`LocatorListType`.

- ``<outLocatorList>``:  List of output locators. It expects a :ref:`LocatorListType`.

- ``<throughputController>``: Limits the output bandwidth of the publisher.

- ``<historyMemoryPolicy>``: Policy of memory allocation for subscriber's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.

- ``<userDefinedID>``: Allows you to set a custom identifier.

- ``<entityID>``: Allows you to establish the entityID of the subscriber.


.. _subscriberprofiles:

Subscriber profiles
-------------------

Subscriber profiles allows you to declare :ref:`Subscriber configuration <pubsubconfiguration>` from XML file.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile in order to load it
as shown in :ref:`loadingapplyingprofiles`.

.. code-block:: xml

    <subscriber profile_name="part_profile_name">
        <topic>
            <!-- TOPIC_TYPE -->
        </topic>

        <qos>
            <!-- QOS -->
        </qos>

        <times> <!-- readerTimesType -->
            <initialAcknackDelay>
                <!-- DURATION -->
            </initialAcknackDelay>
            <heartbeatResponseDelay>
                <!-- DURATION -->
            </heartbeatResponseDelay>
        </times>

        <unicastLocatorList>
            <!-- LOCATOR_LIST -->
        </unicastLocatorList>

        <multicastLocatorList>
            <!-- LOCATOR_LIST -->
        </multicastLocatorList>

        <outLocatorList>
            <!-- LOCATOR_LIST -->
        </outLocatorList>

        <expectsInlineQos>TRUE</expectsInlineQos> <!-- boolean -->

        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>

        <propertiesPolicy>
            <!-- PROPERTIES_POLICY -->
        </propertiesPolicy>

        <userDefinedID>55</userDefinedID> <!-- Int16 -->

        <entityID>66</entityID> <!-- Int16 -->
    </subscriber>

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`QOS` details, please refer to :ref:`CommonQOS`.

    - :class:`TOPIC_TYPE` is detailed in section :ref:`TopicType`.

- ``<topic>``: :ref:`TopicType` configuration of the subscriber.

- ``<qos>``: Subscriber :ref:`CommonQOS` configuration.

- ``<times>``:  Allows you to configure some time related parameters of the subscriber:

    * ``<initialAcknackDelay>``: :ref:`DurationType` of the initial :class:`Acknack` message.

    * ``<heartbeatResponseDelay>``: :ref:`DurationType` to set the delay of the :class:`heartbeat` message response.

- ``<unicastLocatorList>``: List of unicast locators. It expects a :ref:`LocatorListType`.

- ``<multicastLocatorList>``: List of multicast locators. It expects a :ref:`LocatorListType`.

- ``<outLocatorList>``:  List of output locators. It expects a :ref:`LocatorListType`.

- ``<expectsInlineQos>``: Boolean parameter to indicate if QOS is expected inline.

- ``<historyMemoryPolicy>``: Policy of memory allocation for subscriber's history. It can be :class:`PREALLOCATED`, :class:`PREALLOCATED_WITH_REALLOC` or :class:`DYNAMIC`.

- ``<propertiesPolicy>``: Additional configuration properties. It expects a :ref:`PropertiesPolicyType`.

- ``<userDefinedID>``: Allows you to set a custom identifier.

- ``<entityID>``: Allows you to establish the entityID of the subscriber.


.. _commonxml:

Common
------

In the above profiles, some types are used in several different places. To avoid too many details, in some of that
places you found a tag like :class:`<LocatorListType>` that indicates that field is defined in this section.

Now we are going to fully explain these common types:

.. _LocatorListType:

LocatorListType
^^^^^^^^^^^^^^^

It is used to represent a list of :class:`Locator_t`.
LocatorListType is normally used as an anonymous type, this is, it hasn't its own label.
Instead, it is used inside other configuration parameter labels that expect a list of locators and give it sense,
for example, in :class:`<defaultUnicastLocatorList>`:

.. code-block:: xml

    <defaultUnicastLocatorList>
        <locator>
            <kind>UDPv4</kind>
            <!-- Access as physical, typical UDP usage -->
            <port>7400</port> <!-- uint32 -->
            <address>192.168.1.41</address>
        </locator>
        <locator>
            <kind>TCPv4</kind>
            <!-- Both physical and logical, useful in TCP transports -->
            <port_>
                <physical_port>5100</physical_port> <!-- uint16 -->
                <logical_port>7400</logical_port> <!-- uint16 -->
            </port_>
            <addresses_>
                <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                <wan_address>80.80.99.45</wan_address>
                <ip_address>192.168.1.55</ip_address>
            </addresses_>
        </locator>
        <locator>
            <kind>UDPv6</kind>
            <port>8844</port>
            <ipv6_address>::1</ipv6_address>
        </locator>
    </defaultUnicastLocatorList>

In this example, we declared three different locators in :class:`<defaultUnicastLocatorList>`.

Let's see each Locator's field in detail:

- ``<kind>``: Type of the Locator can be UDPv4, UDPv6, TCPv4, and TCPv6.

- ``<port>``: Physical port number of the locator.

- ``<port_>``: Allows you to manage low-level detail in ports of TCP locators, allowing set both **physical_port** and **logical_port**.

- ``<address>``: Allows you to set the IPv4 address of the locator.

- ``<addresses_>``: Allows you to manage low-level details in address of TCP locators (**unique_lan_id**, **wan_address** and **ip_address**).

- ``<ipv6_address>``:  Allows you to set the IPv6 address of the locator.

.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType (XML label :class:`<propertiesPolicy>`) allows you to define a set of generic properties.
It can be used to set a variable number of properties,
very useful at defining extended or custom configuration parameters.

.. code-block:: xml

    <propertiesPolicy>
        <properties>
            <property>
                <name>Property1Name</name> <!-- string -->
                <value>Property1Value</value> <!-- string -->
                <propagate>FALSE</propagate> <!-- boolean -->
            </property>
            <property>
                <name>Property2Name</name> <!-- string -->
                <value>Property2Value</value> <!-- string -->
                <propagate>TRUE</propagate> <!-- boolean -->
            </property>
        </properties>
    </propertiesPolicy>

- ``<name>``: Name to identify the property.

- ``<value>``: Property's value.

- ``<propagate>``: Indicates if the property is meant to be serialized along with the object it belongs to.

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time.
DurationType is normally used as an anonymous type, this is, it hasn't its own label. Instead, it is used inside other
configuration parameter labels that give it sense, like :class:`<leaseAnnouncement>` or :class:`<leaseDuration>`.

.. code-block:: xml

    <leaseDuration>INFINITE</leaseDuration> <!-- string -->

    <leaseDuration>
        <seconds>500</seconds> <!-- int32 -->
        <fraction>0</fraction> <!-- uint32 -->
    </leaseDuration>

    <leaseAnnouncement>
        <seconds>1</seconds> <!-- int32 -->
        <fraction>856000</fraction> <!-- uint32 -->
    </leaseAnnouncement>

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
You can see a deeper explanation of the "topic" field here: :ref:`Topic_information`.

.. code-block:: xml

    <topic>
        <kind>NO_KEY</kind> <!-- string -->
        <name>TopicName</name> <!-- string -->
        <dataType>TopicDataTypeName</dataType> <!-- string -->
        <historyQos>
            <kind>KEEP_LAST</kind> <!-- string -->
            <depth>20</depth> <!-- uint32 -->
        </historyQos>
        <resourceLimitsQos>
            <max_samples>5</max_samples> <!-- unint32 -->
            <max_instances>2</max_instances> <!-- unint32 -->
            <max_samples_per_instance>1</max_samples_per_instance> <!-- unint32 -->
            <allocated_samples>20</allocated_samples> <!-- unint32 -->
        </resourceLimitsQos>
    </topic>

- ``<kind>``: String field that sets if the topic uses keys or not. The available values are: *NO_KEY* and *WITH_KEY*.

- ``<name>``: Name of the topic.

- ``<dataType>``: Indicates if the property is meant to be serialized along with the object it belongs to.

- ``<historyQos>``: The history QoS manages the number of messages that are going to be stored in publishers and subscribers in their histories.

    * ``<kind>``: History type, the available values are: *KEEP_ALL* and *KEEP_LAST*.

    * ``<depth>``: Number of packages that can be stored with the *KEEP_LAST* option.

- ``<resourceLimitsQos>``: The :ref:`history QoS <resourceLimits-qos>`

    * ``<max_samples>``: Maximum number of samples that can be stored in the history of publishers or subscribers. Its default value is 5000.

    * ``<max_instances>``: Maximum number of instances that a publisher or a subscriber can manage. Its default value is 10.

    * ``<max_samples_per_instance>``: Maximum number of samples for each instance. Its default value is 400.

    * ``<allocated_samples>``: Initial samples reserved in the history of publishers or subscribers.

.. _CommonQOS:

QOS
^^^

As a user, you can implement your own quality of service (QoS) restrictions in your application.

.. code-block:: xml

    <qos> <!-- readerQosPoliciesType -->
        <durability>
            <kind>VOLATILE</kind> <!-- string -->
        </durability>

        <durabilityService>
            <!-- DURABILITY_SERVICE -->
        </durabilityService>

        <deadline>
            <period>
                <!-- DURATION -->
            </period>
        </deadline>

        <latencyBudget>
            <duration>
                <!-- DURATION -->
            </duration>
        </latencyBudget>

        <liveliness>
            <!-- LIVELINESS -->
        </liveliness>

        <reliability>
            <kind>BEST_EFFORT</kind>
            <max_blocking_time>
                <!-- DURATION -->
            </max_blocking_time>
        </reliability>

        <lifespan>
            <!-- DURATION -->
        </lifespan>

        <userData> <!-- octetVector (string) -->  </userData>
        <timeBasedFilter>
            <minimum_separation>
                <!-- DURATION -->
            </minimum_separation>
        </timeBasedFilter>

        <ownership>
            <kind>SHARED</kind> <!-- string -->
        </ownership>

        <destinationOrder>
            <kind>BY_RECEPTION_TIMESTAMP</kind> <!-- string -->
        </destinationOrder>

        <presentation>
            <access_scope>INSTANCE</access_scope> <!-- string -->
            <coherent_access>TRUE</coherent_access> <!-- bool -->
            <ordered_access>TRUE</ordered_access> <!-- bool -->
        </presentation>

        <partition>
            <name>part1</name> <!-- string -->
            <name>part2</name> <!-- string -->
        </partition>

        <topicData>
            <value> <!-- octetVector (string) --> </value>
        </topicData>

        <groupData>
            <value> <!-- octetVector (string) --> </value>
        </groupData>
    </qos>

.. note::

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - :class:`DURABILITY_SERVICE` means that the label is a :ref:`DurabilityServiceType` block.

    - :class:`LIVELINESS` means that the label is a :ref:`LiveLinessType` block.

- ``<durability>``: is defined on :ref:`SettingDataDurability` section.

- ``<reliability>``: is defined on :ref:`reliability` section.

- ``<ownership>``: ``<kind>`` determines whether an instance of the Topic is owned by a single Publisher. If the selected ownership is EXCLUSIVE the Publisher will use the Ownership strength value as the strength of its publication. Only the publisher with the highest strength can publish in the same Topic with the same Key.

- ``<destinationOrder>``: ``<kind>`` determines the destination timestamp. BY_RECEPTION_TIMESTAMP for reception and BY_SOURCE_TIMESTAMP for the source.

- ``<presentation>``:
    * ``<access_scope>`` defines the scope of presentation and can be INSTANCE, TOPIC, GROUP.

    * ``<coherent_access>`` defines if the access must be coherent. It's a boolean value.

    * ``<ordered_access>`` establishes if the access must be ordered. It's a boolean value.

.. _DurabilityServiceType:

DurabilityServiceType
^^^^^^^^^^^^^^^^^^^^^

Durability configuration of the endpoint defines how it behaves regarding samples that existed on the topic before a
subscriber joins.

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

- ``<history_kind>``: Allows to set the History kind. It accepts :class:`KEEP_LAST` and :class:`KEEP_ALL` values.

- ``<history_depth>``: Allows to establish the depth of the history.

- ``<max_samples>``: Allows to establish the maximum number of samples to be stored.

- ``<max_instances>``: Allows to establish the maximum number of history instances.

- ``<max_samples_per_instance>``: Allows to establish the maximum number of samples per history instance.

.. _LivelinessType:

LivelinessType
^^^^^^^^^^^^^^

This parameter defines who is responsible for issues of liveliness packets.

.. code-block:: xml

    <liveliness>
        <kind>AUTOMATIC</kind> <!-- string -->
        <leaseDuration>
            <!-- DURATION -->
        </leaseDuration>
        <announcement_period>
            <!-- DURATION -->
        </announcement_period>
    </liveliness>

- ``<kind>``: Especifies how liveliness is managed. Can take values :class:`AUTOMATIC`, :class:`MANUAL_BY_PARTICIPANT`, and :class:`MANUAL_BY_TOPIC`.

- ``<leaseDuration>``: Allows to define how much time the lease is being announced. It is a :ref:`DurationType`.

- ``<announcement_period>``: Allows to define the lease time period. It's a :ref:`DurationType`.


historyQos
^^^^^^^^^^^^^^^^^^^^

The history QoS manages the amount of rtps messages that are going to be stored in publishers and subscribers.
It can be used to set a variable number of properties,
very useful at defining extended or custom configuration parameters.

.. code-block:: xml

    <propertiesPolicy>
        <properties>
            <property>
                <name>Property1Name</name> <!-- string -->
                <value>Property1Value</value> <!-- string -->
                <propagate>FALSE</propagate> <!-- boolean -->
            </property>
            <property>
                <name>Property2Name</name> <!-- string -->
                <value>Property2Value</value> <!-- string -->
                <propagate>TRUE</propagate> <!-- boolean -->
            </property>
        </properties>
    </propertiesPolicy>

- ``<name>``: Name to identify the property.

- ``<value>``: Property's value.

- ``<propagate>``: Indicates if the property is meant to be serialized along with the object it belongs to.


.. _examplexml:

Example
-------

In this section, you can see a full XML example with all possible configuration.
It can be used as a quick reference, but it may not be valid due to incompatibility or exclusive properties.
Don't take it as a working example.

.. code-block:: xml

    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>TransportId1</transport_id>
                <type>TCPv4</type>
                <sendBufferSize>8192</sendBufferSize>
                <receiveBufferSize>8192</receiveBufferSize>
                <TTL>250</TTL>
                <maxMessageSize>16384</maxMessageSize>
                <maxInitialPeersRange>100</maxInitialPeersRange>
                <interfaceWhiteList>
                    <id>192.168.1.41</id>
                    <id>127.0.0.1</id>
                </interfaceWhiteList>
                <wan_addr>80.80.55.44</wan_addr>
                <output_port>5101</output_port>
                <keep_alive_frequency_ms>5000</keep_alive_frequency_ms>
                <keep_alive_timeout_ms>25000</keep_alive_timeout_ms>
                <max_logical_port>200</max_logical_port>
                <logical_port_range>20</logical_port_range>
                <logical_port_increment>2</logical_port_increment>
                <ListeningPorts>
                    <port>5100</port>
                    <port>5200</port>
                </ListeningPorts>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>TransportId2</transport_id>
                <type>UDPv6</type>
            </transport_descriptor>
        </transport_descriptors>

        <types>
            <type> <!-- Types can be defined in its own type tag or sharing the same tag -->
                <enum name="MyAloneEnumType">
                    <literal name="A" value="0"/>
                    <literal name="B" value="1"/>
                    <literal name="C" value="2"/>
                </enum>
            </type>
            <type>
                <enum name="MyEnumType">
                    <literal name="A" value="0"/>
                    <literal name="B" value="1"/>
                    <literal name="C" value="2"/>
                </enum>

                <typedef name="MyAlias1" value="MyEnum"/>

                <typedef name="MyAlias2">
                    <long dimensions="2,2"/>
                </typedef>

                <struct name="MyStruct">
                    <long name="first"/>
                    <longlong name="second"/>
                </struct>

                <union name="MyUnion">
                    <discriminator type="octet"/>
                    <case>
                        <caseValue value="0"/>
                        <caseValue value="1"/>
                        <long name="first"/>
                    </case>
                    <case>
                        <caseValue value="2"/>
                        <MyStruct name="second"/>
                    </case>
                    <case>
                        <caseValue value="default"/>
                        <longlong name="third"/>
                    </case>
                </union>

                <!-- All possible members struct type -->
                <struct name="MyFullStruct">
                    <!-- Primitives & basic -->
                    <boolean name="my_bool"/>
                    <octet name="my_octet"/>
                    <char name="my_char"/>
                    <wchar name="my_wchar"/>
                    <short name="my_short"/>
                    <long name="my_long"/>
                    <longlong name="my_longlong"/>
                    <unsignedshort name="my_unsignedshort"/>
                    <unsignedlong name="my_unsignedlong"/>
                    <unsignedlonglong name="my_unsignedlonglong"/>
                    <float name="my_float"/>
                    <double name="my_double"/>
                    <longdouble name="my_longdouble"/>
                    <string name="my_string"/>
                    <wstring name="my_wstring"/>
                    <boundedString name="my_boundedString">
                        <maxLength value="41925"/>
                    </boundedString>
                    <boundedWString name="my_boundedWString">
                        <maxLength value="41925"/>
                    </boundedWString>

                    <!-- long long_array[2][3][4]; -->
                    <long name="long_array" dimensions="2,3,4"/>

                    <!-- sequence<sequence<long,2>,3> my_sequence_sequence; -->
                    <sequence name="my_sequence_sequence" length="3">
                        <sequence type="long" length="2"/>
                    </sequence>

                    <!-- map<long,map<long,long,2>,2> my_map_map; -->
                    <map name="my_map_map" key_type="long" length="2">
                        <value_type>
                            <map key_type="long" value_type="long" length="2"/>
                        </value_type>
                    </map>

                    <!-- Complex types -->
                    <struct name="OtherStruct">
                        <MyEnum name="my_enum"/>
                        <MyStruct name="my_struct" dimensions="5"/>
                    </struct>
                </struct>
            </type>
        </types>

        <participant profile_name="part_profile_name">
            <rtps>
                <name>Participant Name</name> <!-- String -->

                <defaultUnicastLocatorList>
                    <locator>
                        <kind>UDPv4</kind>
                        <!-- Access as physical, like UDP -->
                        <port>7400</port>
                        <address>192.168.1.41</address>
                    </locator>
                    <locator>
                        <kind>TCPv4</kind>
                        <!-- Both physical and logical, like TCP -->
                        <port_>
                            <physical_port>5100</physical_port>
                            <logical_port>7400</logical_port>
                        </port_>
                        <addresses_>
                            <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                            <wan_address>80.80.99.45</wan_address>
                            <ip_address>192.168.1.55</ip_address>
                        </addresses_>
                    </locator>
                    <locator>
                        <kind>UDPv6</kind>
                        <port>8844</port>
                        <ipv6_address>::1</ipv6_address>
                    </locator>
                </defaultUnicastLocatorList>

                <defaultMulticastLocatorList>
                    <locator>
                        <kind>UDPv4</kind>
                        <!-- Access as physical, like UDP -->
                        <port>7400</port>
                        <address>192.168.1.41</address>
                    </locator>
                    <locator>
                        <kind>TCPv4</kind>
                        <!-- Both physical and logical, like TCP -->
                        <port_>
                            <physical_port>5100</physical_port>
                            <logical_port>7400</logical_port>
                        </port_>
                        <addresses_>
                            <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                            <wan_address>80.80.99.45</wan_address>
                            <ip_address>192.168.1.55</ip_address>
                        </addresses_>
                    </locator>
                    <locator>
                        <kind>UDPv6</kind>
                        <port>8844</port>
                        <ipv6_address>::1</ipv6_address>
                    </locator>
                </defaultMulticastLocatorList>

                <sendSocketBufferSize>8192</sendSocketBufferSize>

                <listenSocketBufferSize>8192</listenSocketBufferSize>

                <builtin>
                    <use_SIMPLE_RTPS_PDP>FALSE</use_SIMPLE_RTPS_PDP>

                    <use_WriterLivelinessProtocol>FALSE</use_WriterLivelinessProtocol>

                    <EDP>SIMPLE</EDP>

                    <domainId>4</domainId>

                    <leaseDuration>INFINITE</leaseDuration>

                    <leaseAnnouncement>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </leaseAnnouncement>

                    <simpleEDP>
                        <PUBWRITER_SUBREADER>TRUE</PUBWRITER_SUBREADER>
                        <PUBREADER_SUBWRITER>TRUE</PUBREADER_SUBWRITER>
                    </simpleEDP>

                    <metatrafficUnicastLocatorList>
                        <locator>
                            <kind>UDPv4</kind>
                            <!-- Access as physical, like UDP -->
                            <port>7400</port>
                            <address>192.168.1.41</address>
                        </locator>
                        <locator>
                            <kind>TCPv4</kind>
                            <!-- Both physical and logical, like TCP -->
                            <port_>
                                <physical_port>5100</physical_port>
                                <logical_port>7400</logical_port>
                            </port_>
                            <addresses_>
                                <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                                <wan_address>80.80.99.45</wan_address>
                                <ip_address>192.168.1.55</ip_address>
                            </addresses_>
                        </locator>
                        <locator>
                            <kind>UDPv6</kind>
                            <port>8844</port>
                            <ipv6_address>::1</ipv6_address>
                        </locator>
                    </metatrafficUnicastLocatorList>

                    <metatrafficMulticastLocatorList>
                        <locator>
                            <kind>UDPv4</kind>
                            <!-- Access as physical, like UDP -->
                            <port>7400</port>
                            <address>192.168.1.41</address>
                        </locator>
                        <locator>
                            <kind>TCPv4</kind>
                            <!-- Both physical and logical, like TCP -->
                            <port_>
                                <physical_port>5100</physical_port>
                                <logical_port>7400</logical_port>
                            </port_>
                            <addresses_>
                                <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                                <wan_address>80.80.99.45</wan_address>
                                <ip_address>192.168.1.55</ip_address>
                            </addresses_>
                        </locator>
                        <locator>
                            <kind>UDPv6</kind>
                            <port>8844</port>
                            <ipv6_address>::1</ipv6_address>
                        </locator>
                    </metatrafficMulticastLocatorList>

                    <initialPeersList>
                        <locator>
                            <kind>UDPv4</kind>
                            <!-- Access as physical, like UDP -->
                            <port>7400</port>
                            <address>192.168.1.41</address>
                        </locator>
                        <locator>
                            <kind>TCPv4</kind>
                            <!-- Both physical and logical, like TCP -->
                            <port_>
                                <physical_port>5100</physical_port>
                                <logical_port>7400</logical_port>
                            </port_>
                            <addresses_>
                                <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                                <wan_address>80.80.99.45</wan_address>
                                <ip_address>192.168.1.55</ip_address>
                            </addresses_>
                        </locator>
                        <locator>
                            <kind>UDPv6</kind>
                            <port>8844</port>
                            <ipv6_address>::1</ipv6_address>
                        </locator>
                    </initialPeersList>

                    <staticEndpointXMLFilename>filename.xml</staticEndpointXMLFilename>

                    <readerHistoryMemoryPolicy>PREALLOCATED_WITH_REALLOC</readerHistoryMemoryPolicy>

                    <writerHistoryMemoryPolicy>PREALLOCATED</writerHistoryMemoryPolicy>
                </builtin>

                <port>
                    <portBase>7400</portBase>
                    <domainIDGain>200</domainIDGain>
                    <participantIDGain>10</participantIDGain>
                    <offsetd0>0</offsetd0>
                    <offsetd1>1</offsetd1>
                    <offsetd2>2</offsetd2>
                    <offsetd3>3</offsetd3>
                </port>

                <userData>111222333</userData>

                <participantID>99</participantID>

                <throughputController>
                    <bytesPerPeriod>8192</bytesPerPeriod>
                    <periodMillisecs>1000</periodMillisecs>
                </throughputController>

                <userTransports>
                    <id>TransportId1</id>
                    <id>TransportId2</id>
                </userTransports>

                <useBuiltinTransports>FALSE</useBuiltinTransports>

                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>Property1Name</name>
                            <value>Property1Value</value>
                            <propagate>FALSE</propagate>
                        </property>
                        <property>
                            <name>Property2Name</name>
                            <value>Property2Value</value>
                            <propagate>FALSE</propagate>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>

        <publisher profile_name="part_profile_name">
            <topic>
                <kind>WITH_KEY</kind>
                <name>TopicName</name>
                <dataType>TopicDataTypeName</dataType>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
                <resourceLimitsQos>
                    <max_samples>5</max_samples>
                    <max_instances>2</max_instances>
                    <max_samples_per_instance>1</max_samples_per_instance>
                    <allocated_samples>20</allocated_samples>
                </resourceLimitsQos>
            </topic>

            <qos> <!-- writerQosPoliciesType -->
                <durability>
                    <kind>VOLATILE</kind>
                </durability>
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
                <deadline>
                    <period>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </period>
                </deadline>
                <latencyBudget>
                    <duration>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </duration>
                </latencyBudget>
                <liveliness>
                    <kind>AUTOMATIC</kind>
                    <leaseDuration>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </leaseDuration>
                    <announcement_period>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </announcement_period>
                </liveliness>
                <reliability>
                    <kind>BEST_EFFORT</kind>
                    <max_blocking_time>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </max_blocking_time>
                </reliability>
                <lifespan>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </lifespan>
                <userData>155463</userData>
                <timeBasedFilter>
                    <minimum_separation>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </minimum_separation>
                </timeBasedFilter>
                <ownership>
                    <kind>SHARED</kind>
                </ownership>
                <ownershipStrength>
                    <value>5</value>
                </ownershipStrength>
                <destinationOrder>
                    <kind>BY_RECEPTION_TIMESTAMP</kind>
                </destinationOrder>
                <presentation>
                    <access_scope>TOPIC</access_scope>
                    <coherent_access>TRUE</coherent_access>
                    <ordered_access>TRUE</ordered_access>
                </presentation>
                <partition>
                    <name>part1</name>
                    <name>part2</name>
                </partition>
                <topicData>
                    <value>155645</value>
                </topicData>
                <groupData>
                    <value>6546</value>
                </groupData>
                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>

            <times>
                <initialHeartbeatDelay>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </initialHeartbeatDelay>
                <heartbeatPeriod>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </heartbeatPeriod>
                <nackResponseDelay>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </nackResponseDelay>
                <nackSupressionDuration>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </nackSupressionDuration>
            </times>

            <unicastLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </unicastLocatorList>

            <multicastLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </multicastLocatorList>

            <outLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </outLocatorList>

            <throughputController>
                <bytesPerPeriod>8192</bytesPerPeriod>
                <periodMillisecs>1000</periodMillisecs>
            </throughputController>

            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>

            <propertiesPolicy>
                <properties>
                    <property>
                        <name>Property1Name</name>
                        <value>Property1Value</value>
                        <propagate>FALSE</propagate>
                    </property>
                    <property>
                        <name>Property2Name</name>
                        <value>Property2Value</value>
                        <propagate>FALSE</propagate>
                    </property>
                </properties>
            </propertiesPolicy>

            <userDefinedID>45</userDefinedID>

            <entityID>76</entityID>
        </publisher>

        <subscriber profile_name="part_profile_name">
            <topic>
                <kind>WITH_KEY</kind>
                <name>TopicName</name>
                <dataType>TopicDataTypeName</dataType>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
                <resourceLimitsQos>
                    <max_samples>5</max_samples>
                    <max_instances>2</max_instances>
                    <max_samples_per_instance>1</max_samples_per_instance>
                    <allocated_samples>20</allocated_samples>
                </resourceLimitsQos>
            </topic>

            <qos>
                <durability>
                    <kind>PERSISTENT</kind>
                </durability>
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
                <deadline>
                    <period>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </period>
                </deadline>
                <latencyBudget>
                    <duration>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </duration>
                </latencyBudget>
                <liveliness>
                    <kind>MANUAL_BY_PARTICIPANT</kind>
                    <leaseDuration>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </leaseDuration>
                    <announcement_period>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </announcement_period>
                </liveliness>
                <reliability>
                    <kind>BEST_EFFORT | RELIABLE</kind>
                    <max_blocking_time>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </max_blocking_time>
                </reliability>
                <lifespan>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </lifespan>
                <userData>554688</userData>
                <timeBasedFilter>
                    <minimum_separation>
                        <seconds>1</seconds>
                        <fraction>856000</fraction>
                    </minimum_separation>
                </timeBasedFilter>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
                <destinationOrder>
                    <kind>BY_RECEPTION_TIMESTAMP</kind>
                </destinationOrder>
                <presentation>
                    <access_scope>INSTANCE</access_scope>
                    <coherent_access>TRUE</coherent_access>
                    <ordered_access>TRUE</ordered_access>
                </presentation>
                <partition>
                    <name>part1</name>
                    <name>part2</name>
                </partition>
                <topicData>
                    <value>165733</value>
                </topicData>
                <groupData>
                    <value>165433</value>
                </groupData>
            </qos>

            <times>
                <initialAcknackDelay>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </initialAcknackDelay>
                <heartbeatResponseDelay>
                    <seconds>1</seconds>
                    <fraction>856000</fraction>
                </heartbeatResponseDelay>
            </times>

            <unicastLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </unicastLocatorList>

            <multicastLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </multicastLocatorList>

            <outLocatorList>
                <locator>
                    <kind>UDPv4</kind>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </locator>
                <locator>
                    <kind>TCPv4</kind>
                    <!-- Both physical and logical, like TCP -->
                    <port_>
                        <physical_port>5100</physical_port>
                        <logical_port>7400</logical_port>
                    </port_>
                    <addresses_>
                        <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
                        <wan_address>80.80.99.45</wan_address>
                        <ip_address>192.168.1.55</ip_address>
                    </addresses_>
                </locator>
                <locator>
                    <kind>UDPv6</kind>
                    <port>8844</port>
                    <ipv6_address>::1</ipv6_address>
                </locator>
            </outLocatorList>

            <expectsInlineQos>TRUE</expectsInlineQos>

            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>

            <propertiesPolicy>
                <properties>
                    <property>
                        <name>Property1Name</name>
                        <value>Property1Value</value>
                        <propagate>FALSE</propagate>
                    </property>
                    <property>
                        <name>Property2Name</name>
                        <value>Property2Value</value>
                        <propagate>FALSE</propagate>
                    </property>
                </properties>
            </propertiesPolicy>

            <userDefinedID>55</userDefinedID>

            <entityID>66</entityID>
        </subscriber>
    </profiles>
