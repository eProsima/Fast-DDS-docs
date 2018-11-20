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
as quick reference too.

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

This section allow us to create transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiate it will use or create the described transport.

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

- **transport_id**: Unique name to identify each transport descriptor.

- **type**: Type of the transport descriptor. Current supported types are UDPv4, UDPv6, TCPv4, and TCPv6.

- **sendBufferSize**: Size, in bytes, of the socket send buffer.

- **receiveBufferSize**: Size, in bytes, of the socket receive buffer.

- **TTL**: *Time To Live*, **only** for UDP transports.

- **maxMessageSize**: Maximum size in bytes of the transport message buffer.

- **maxInitialPeersRange**: Stablishes the maximum number of guessed inital peers to try to connect (default **4**).

- **interfaceWhiteList**: Allows you to define :ref:`whitelist-interfaces`.

- **wan_addr**: Allows you to declare the public WAN address when using **TCPv4 transports**.

- **output_port**: Port used for output bound, instead a random one.

- **keep_alive_frequency_ms**: Frequency in milliseconds for sending RTCP keep alive requests (**only** TCP).

- **keep_alive_timeout_ms**: Time in milliseconds to consider a connection is broken since the last keep alive requests was sent (**only** TCP).

- **max_logical_port**: Maximum number of logical ports to try during RTCP negotiation (**only** TCP).

- **logical_port_range**: Maximum number of logical ports per request to try during RTCP negotiation (**only** TCP).

- **logical_port_increment**: Increment between logical ports to try during RTCP negotiation (**only** TCP).

- **ListeningPorts**: Local port to work as TCP acceptor for input connections. If not set, the transport will work as TCP client only (**only** TCP).

You can see more examples in :ref:`comm-transports-configuration`.

.. _xmldynamictypes:

XML Dynamic Types
-----------------

XML Dynamic Types allows eProsima Fast RTPS to create Dynamic Types directly defining them through XML.
This allows any application to change TopicDataTypes without the need to change its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (`<types>`, types tag) in the XML file can be placed similarly to the profiles tag.
It can be a stand-alone XML Types file or be a child of the fastrtps XML root tag (`<dds>`).
Inside the types tag, must be one or more type tags (`<type>`).

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
    |       <literal name="A" value="0"/>           |     enum_builder->AddEmptyMember(0, "A");                                                                 |
    |       <literal name="B" value="1"/>           |     enum_builder->AddEmptyMember(1, "B");                                                                 |
    |       <literal name="C" value="2"/>           |     enum_builder->AddEmptyMember(2, "C");                                                                 |
    |   </enum>                                     |     DynamicType_ptr enum_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(enum_builder.get()); |
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
    |       </case>                          |     union_builder->AddMember(0, "first", long_builder, "", { 0, 1 }, false);                                                   |
    |       <case>                           |     union_builder->AddMember(1, "second", struct_builder, "", { 2 }, false);                                                   |
    |           <caseValue value="2"/>       |     union_builder->AddMember(2, "third", long_long_builder, "", { }, true);                                                    |
    |           <MyStruct name="second"/>    |     DynamicType_ptr union_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(union_builder.get());                    |
    |       </case>                          |                                                                                                                                |
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

When used as sequences elements, key or value types of a map, as an aliased type, etc., its name attribute
is ignored and can be omitted.

**Basic types**

The available basic types XML tags are:

- boolean
- octet
- char
- wchar
- short
- long
- longlong
- unsignedshort
- unsignedlong
- unsignedlonglong
- float
- double
- longdouble
- string
- wstring
- boundedString
- boundedWString

All of them are defined simply:

    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                    | C++                                                                                                                            |
    +========================================+================================================================================================================================+
    | .. code-block:: xml                    | .. code-block:: c++                                                                                                            |
    |                                        |                                                                                                                                |
    |   <longlong name="my_long"/>           |     DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();                 |
    |                                        |     DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(long_long_builder.get());            |
    |                                        |                                                                                                                                |
    +----------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+

Except for boundedString and boundedWString that should include an inner element *maxLength* whose value indicates
the maximum length of the string.

    +--------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                        | C++                                                                                                                            |
    +============================================+================================================================================================================================+
    | .. code-block:: xml                        | .. code-block:: c++                                                                                                            |
    |                                            |                                                                                                                                |
    |   <boundedString name="my_large_string">   |     DynamicTypeBuilder_ptr string_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(41925);              |
    |       <maxLength value="41925"/>           |     DynamicType_ptr string_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(string_builder.get());                  |
    |   </boundedString>                         |                                                                                                                                |
    |                                            |                                                                                                                                |
    |   <boundedWString name="my_large_wstring"> |     DynamicTypeBuilder_ptr wstring_builder = DynamicTypeBuilderFactory::GetInstance()->CreateWstringBuilder(20925);            |
    |       <maxLength value="20925"/>           |     DynamicType_ptr wstring_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(wstring_builder.get());                |
    |   </boundedWString>                        |                                                                                                                                |
    |                                            |                                                                                                                                |
    +--------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+

**Arrays**

Arrays are defined exactly the same way as any other member type, but adds the attribute *dimensions*.
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
    |                                                  |     DynamicType_ptr array_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(array_builder.get());                            |
    |                                                  |                                                                                                                                        |
    +--------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------+


It's IDL analogue would be:

.. code-block:: c++

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its name, its content type and its (optional) length.
The type of its content can be defined by its type attribute or by a member type.

Example:

    +-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------+
    | XML                                                   | C++                                                                                                                                    |
    +=======================================================+========================================================================================================================================+
    | .. code-block:: xml                                   | .. code-block:: c++                                                                                                                    |
    |                                                       |                                                                                                                                        |
    |   <sequence name="my_sequence_sequence" length="3">   |     uint32_t length = 2;                                                                                                               |
    |       <sequence type="long" length="2"/>              |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                              |
    |   </sequence>                                         |     DynamicTypeBuilder_ptr seq_builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(long_builder.get(), length);  |
    |                                                       |     DynamicType_ptr seq_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(seq_builder.get());                                |
    |                                                       |                                                                                                                                        |
    +-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------+

The example shows a sequence with length 3 of sequences with length 2 with long contents.
As IDL would be:

.. code-block:: c++

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner (or content) sequence has no name, as it would be ignored by the parser.

**Maps**

Maps are similar to sequences but they need to define two types instead one. One for its key and another
for its value.
Again, both types can be defined as attributes or as members, but in this cases, when defined
as members, they are content in another XML element key_type and value_type respectively.

The definition kind of each type can be mixed, this is, one type can be defined as an attribute and the
other as a member.

Example:

    +---------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+
    | XML                                                           | C++                                                                                                                            |
    +===============================================================+================================================================================================================================+
    | .. code-block:: xml                                           | .. code-block:: c++                                                                                                            |
    |                                                               |                                                                                                                                |
    |   <map name="my_map_map" key_type="long" length="2">          |     uint32_t length = 2;                                                                                                       |
    |       <value_type>                                            |     DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();                      |
    |           <map key_type="long" value_type="long" length="2"/> |     DynamicTypeBuilder_ptr map_builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(long_builder.get(),        |
    |       </value_type>                                           |     long_builder.get(), length);                                                                                               |
    |   </map>                                                      |                                                                                                                                |
    |                                                               |     DynamicType_ptr map_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(map_builder.get());                        |
    |                                                               |                                                                                                                                |
    +---------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------+

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

Participant profiles allows you to declare participant configurations from XML file.

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
                <use_SIMPLE_RTPS_PDP>FALSE</use_SIMPLE_RTPS_PDP> <!-- boolean -->

                <use_WriterLivelinessProtocol>FALSE</use_WriterLivelinessProtocol>  <!-- boolean -->

                <EDP> SIMPLE | STATIC </EDP> <!-- string -->

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

                <readerHistoryMemoryPolicy> PREALLOCATED | PREALLOCATED_WITH_REALLOC | DYNAMIC  </readerHistoryMemoryPolicy> <!-- historyMemoryPolicyType -->

                <writerHistoryMemoryPolicy> <!-- historyMemoryPolicyType --> </writerHistoryMemoryPolicy>
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

**NOTES**:

- :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

- :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

- :class:`DURATION` means it expects a :ref:`DurationType`.

.. _publisherprofiles:

Publisher profiles
------------------

fghfghfg

.. _subscriberprofiles:

Subscriber profiles
-------------------

fghfghfgh

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

- **kind**: Type of the Locator can be UDPv4, UDPv6, TCPv4, and TCPv6.

- **port**: Physical port number of the locator.

- **port_**: Allows you to manage low-level detail in ports of TCP locators, allowing set both **physical_port** and **logical_port**.

- **address**: Allows you to set the IPv4 address of the locator.

- **addresses_**: Allows you to manage low-level details in address of TCP locators (**unique_lan_id**, **wan_address** and **ip_address**).

- **ipv6_address**:  Allows you to set the IPv6 address of the locator.

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

- **name**: Name to identify the property.

- **value**: Property's value.

- **propagate**: Indicates if the property is meant to be serialized along with the object it belongs to.

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
or by :class:`<seconds>` plus :class:`<fraction>` labels:

- **INFINITE**: Constant value, represents an infinite period of time.

- **ZERO**: Constant value, represents 0.0 seconds.

- **INVALID**: Constant value, represents an invalid period of time.

- **seconds**: Integer seconds value.

- **fraction**: Fractions of a second. A fraction is :class:`1/(2^32)` seconds.

.. _CommonQOS:

QOS
^^^

QOS blah blah blah

.. _examplexml:

Example
-------

klsahdklhasd