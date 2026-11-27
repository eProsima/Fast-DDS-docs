<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="discovery-static"></a>

<a id="static-discovery"></a>

# STATIC Discovery

*Fast DDS* allows for the substitution of the SEDP protocol for the EDP phase with a static version that completely
eliminates EDP meta traffic.
This can become useful when dealing with limited network bandwidth and a well-known schema of [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and
[DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
If all DataWriters and DataReaders, and their [Topics](../dds_layer/topic/topic.md#dds-layer-topic) and data types, are known beforehand, the EDP phase can be
replaced with a static configuration of peers.
It is important to note that by doing this, no EDP discovery meta traffic will be generated, and only those peers
defined in the configuration will be able to communicate.
The STATIC discovery related settings are:

| Name                                                      | Description                                                                                |
|-----------------------------------------------------------|--------------------------------------------------------------------------------------------|
| [STATIC EDP](#static-edp)                                 | It activates the STATIC discovery protocol.                                                |
| [STATIC EDP XML Configuration Specification](#static-xml) | Specifies an XML content with a description of the remote DataWriters and<br/>DataReaders. |
| [Initial Announcements](simple.md#initial-announcements)  | It defines the behavior of the DomainParticipant initial announcements (PDP phase).        |

<a id="static-edp"></a>

<a id="id1"></a>

## STATIC EDP

To activate the STATIC EDP, the SEDP must be disabled on the [`WireProtocolConfigQos`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE).
This can be done either by code or using an XML configuration file:

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
pqos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
```

### XML

```xml
<participant profile_name="participant_profile_static_edp">
    <rtps>
        <builtin>
            <discovery_config>
                <EDP>STATIC</EDP>
            </discovery_config>
        </builtin>
    </rtps>
</participant>
```

Currently two different formats of exchanging information in the Participant Discovery Phase (PDP) are supported:
the default one and another that reduces the network bandwidth used.
[Static Discovery’s Exchange Format](../property_policies/non_consolidated_qos.md#property-policies-edp-exchange-format) explains how to change this.

<a id="static-xml"></a>

<a id="static-edp-xml-configuration-specification"></a>

## STATIC EDP XML Configuration Specification

Since activating STATIC EDP suppresses all EDP meta traffic, the information about the remote entities (DataWriters
and DataReaders) must be statically specified, which is done using dedicated XML files.
A [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) may load several of such configuration files so that the information about different entities can
be contained in one file, or split into different files to keep it more organized.
*Fast DDS*  provides a
[Static Discovery example](https://github.com/eProsima/Fast-DDS/blob/master/examples/cpp/dds/StaticHelloWorldExample)
that implements this EDP discovery protocol.

The following table describes all the possible elements of a STATIC EDP XML configuration file.
A full example of such file can be found in [STATIC EDP XML Example](#static-xml-example).

<!-- Some large words outside of table. Then table fit maximum line length -->

| Name                    | Description                                                                                                                                       | Values                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Default                                                                                                                                                                                |
|-------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<userId>`              | Mandatory. <br/><br/>Uniquely identifies the DataReader/DataWriter.                                                                               | `uint16_t`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | 0                                                                                                                                                                                      |
| `<entityID>`            | EntityId of the DataReader/DataWriter.                                                                                                            | `uint16_t`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | 0                                                                                                                                                                                      |
| `<expects_inline_qos>`  | It indicates if QOS is expected inline<br/>(DataReader **only**).                                                                                 | `bool`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | `false`                                                                                                                                                                                |
| `<topicName>`           | Mandatory. <br/><br/>The topic of the remote DataReader/DataWriter.<br/>Should match with one of the topics of the local DataReaders/DataWriters. | `string_255`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |                                                                                                                                                                                        |
| `<topicDataType>`       | Mandatory. <br/><br/>The data type of the topic.                                                                                                  | `string_255`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |                                                                                                                                                                                        |
| `<topicKind>`           | The kind of topic.                                                                                                                                | `NO_KEY` <br/><br/>`WITH_KEY` <br/>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | `NO_KEY` <br/>                                                                                                                                                                         |
| `<partitionQos>`        | The name of a partition of the remote peer.<br/>Repeat to configure several partitions.                                                           | `string`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                                                                                        |
| `<unicastLocator>`      | Unicast locator of the DomainParticipant.<br/>See [Locators definition](#staticlocators).                                                         |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                                                                                                                                                                                        |
| `<multicastLocator>`    | Multicast locator of the DomainParticipant.<br/>See [Locators definition](#staticlocators).                                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                                                                                                                                                                                        |
| `<reliabilityQos>`      | See the [ReliabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy) section.                                     | [`BEST_EFFORT_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) <br/><br/>[`RELIABLE_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)                                                                                                                                                                                          | [`BEST_EFFORT_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) |
| `<durabilityQos>`       | See the [DurabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#durabilityqospolicy) section.                                       | [`VOLATILE_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE) <br/><br/>[`TRANSIENT_LOCAL_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) <br/><br/>[`TRANSIENT_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE) | [`VOLATILE_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)           |
| `<ownershipQos>`        | See [Ownership QoS](#ownershipqos).                                                                                                               |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                                                                                                                                                                                        |
| `<livelinessQos>`       | Defines the liveliness of the remote peer.<br/>See [Liveliness QoS](#livelinessqos).                                                              |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                                                                                                                                                                                        |
| `<disablePositiveAcks>` | See [DisablePositiveACKsQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy).                                  | See [DisablePositiveAcks](../xml_configuration/common.md#xml-disablepositiveacks)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |                                                                                                                                                                                        |

<a id="staticlocators"></a>

<a id="locators-definition"></a>

### Locators definition

Locators for remote peers are configured using `<unicastLocator>` and `<multicastLocator>` tags.
These take no value, and the locators are defined using tag elements.
Locators defined with `<unicastLocator>` and `<multicastLocator>` are accumulative, so they can be repeated to
assign several remote endpoints locators to the same peer.

* `address`: a mandatory `string` representing the locator address.
* `port`: an optional `uint16_t` representing a port on that address.

<a id="ownershipqos"></a>

<a id="ownership-qos"></a>

### Ownership QoS

The ownership of the topic can be configured using `<ownershipQos>` tag.
It takes no value, and the configuration is done using tag elements:

* `kind`: can be one of [`SHARED_OWNERSHIP_QOS`](../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE) or [`EXCLUSIVE_OWNERSHIP_QOS`](../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE).
  This element is mandatory withing the tag.
* `strength`: an optional `uint32_t` specifying how strongly the remote DomainParticipant owns the [Topic](../dds_layer/topic/topic.md#dds-layer-topic).
  This QoS can be set on DataWriters **only**.
  If not specified, default value is zero.

<a id="livelinessqos"></a>

<a id="liveliness-qos"></a>

### Liveliness QoS

The [LivelinessQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy) of the remote peer is configured using `<livelinessQos>` tag.
It takes no value, and the configuration is done using tag elements:

* `kind`: can be any of [`AUTOMATIC_LIVELINESS_QOS`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE), [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) or
  [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE). This element is mandatory withing the tag.
* `leaseDuration_ms`: an optional `uint32` specifying the lease duration for the remote peer.
  The special value `INF` can be used to indicate infinite lease duration.
  If not specified, default value is `INF`

<a id="check-static-xml-file"></a>

<a id="checking-static-edp-xml-files"></a>

## Checking STATIC EDP XML Files

Before loading a static EDP XML file, it would be useful to check its validity and make sure the file will be
successfully loaded.
This verification can be performed on [DomainParticipantFactory](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) using
[`DomainParticipantFactory::check_xml_static_discovery()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory26check_xml_static_discoveryERNSt6stringE), using either XML files or the configuration directly,
as in the examples below.

### File

```c++
// The (file://) flag is optional.
std::string file = "file://static_Discovery.xml";
DomainParticipantFactory* factory = DomainParticipantFactory::get_instance();
if (RETCODE_OK != factory->check_xml_static_discovery(file))
{
    std::cout << "Error parsing xml file " << file << std::endl;
}
```

### Data

```c++
// The (data://) flag is required to load the configuration directly.
std::string fileData = "data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
        "<staticdiscovery>" \
        "<participant>" \
        "<name>HelloWorldPublisher</name>" \
        "<writer>" \
        "<userId>1</userId>" \
        "<entityID>2</entityID>" \
        "<topicName>HelloWorldTopic</topicName>" \
        "<topicDataType>HelloWorld</topicDataType>" \
        "</writer>" \
        "</participant>" \
        "</staticdiscovery>";
if (RETCODE_OK != factory->check_xml_static_discovery(fileData))
{
    std::cout << "Error parsing xml file data:" << std::endl << fileData << std::endl;
}
```

<a id="static-xml-example"></a>

<a id="static-edp-xml-example"></a>

### STATIC EDP XML Example

The following is a complete example of a configuration XML file for two remote DomainParticipant, a DataWriter and
a DataReader.
This configuration **must** agree with the configuration used to create the remote DataReader/DataWriter.
Otherwise, communication between DataReaders and DataWriters may be affected.
If any non-mandatory element is missing, it will take the default value.
As a rule of thumb, all the elements that were specified on the remote DataReader/DataWriter creation should be
configured.

| **XML**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```xml<br/><br/><staticdiscovery><br/>    <participant><br/>        <name>HelloWorldSubscriber</name><br/>        <reader><br/>            <userId>3</userId><br/>            <entityID>4</entityID><br/>            <expects_inline_qos>true</expects_inline_qos><br/>            <topicName>HelloWorldTopic</topicName><br/>            <topicDataType>HelloWorld</topicDataType><br/>            <topicKind>WITH_KEY</topicKind><br/>            <partitionQos>HelloPartition</partitionQos><br/>            <partitionQos>WorldPartition</partitionQos><br/>            <unicastLocator address="192.168.0.128" port="5000"/><br/>            <unicastLocator address="10.47.8.30" port="6000"/><br/>            <multicastLocator address="239.255.1.1" port="7000"/><br/>            <reliabilityQos>BEST_EFFORT_RELIABILITY_QOS</reliabilityQos><br/>            <durabilityQos>VOLATILE_DURABILITY_QOS</durabilityQos><br/>            <ownershipQos kind="SHARED_OWNERSHIP_QOS"/><br/>            <livelinessQos kind="AUTOMATIC_LIVELINESS_QOS" leaseDuration_ms="1000"/><br/>            <disablePositiveAcks><br/>                <enabled>true</enabled><br/>            </disablePositiveAcks><br/>        </reader><br/>    </participant><br/>    <participant><br/>        <name>HelloWorldPublisher</name><br/>        <writer><br/>            <unicastLocator address="192.168.0.120" port="9000"/><br/>            <unicastLocator address="10.47.8.31" port="8000"/><br/>            <multicastLocator address="239.255.1.1" port="7000"/><br/>            <userId>5</userId><br/>            <entityID>6</entityID><br/>            <topicName>HelloWorldTopic</topicName><br/>            <topicDataType>HelloWorld</topicDataType><br/>            <topicKind>WITH_KEY</topicKind><br/>            <partitionQos>HelloPartition</partitionQos><br/>            <partitionQos>WorldPartition</partitionQos><br/>            <reliabilityQos>BEST_EFFORT_RELIABILITY_QOS</reliabilityQos><br/>            <durabilityQos>VOLATILE_DURABILITY_QOS</durabilityQos><br/>            <ownershipQos kind="SHARED_OWNERSHIP_QOS" strength="50"/><br/>            <livelinessQos kind="AUTOMATIC_LIVELINESS_QOS" leaseDuration_ms="1000"/><br/>            <disablePositiveAcks><br/>                <enabled>true</enabled><br/>                <duration><br/>                    <sec>300</sec><br/>                </duration><br/>            </disablePositiveAcks><br/>        </writer><br/>    </participant><br/></staticdiscovery><br/><br/>``` |

<a id="static-xml-load"></a>

<a id="loading-static-edp-xml-files"></a>

## Loading STATIC EDP XML Files

Statically discovered remote DataReaders/DataWriters **must** define a unique *userID* on their profile, whose value
**must** agree with the one specified in the discovery configuration XML.
This is done by setting the user ID on the [DataReaderQos](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos)/[DataWriterQos](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos):

### C++

```c++
// Configure the DataWriter
DataWriterQos wqos;
wqos.endpoint().user_defined_id = 1;

// Configure the DataReader
DataReaderQos rqos;
rqos.endpoint().user_defined_id = 3;
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_static_discovery">
    <userDefinedID>3</userDefinedID>
</data_writer>

<data_reader profile_name="reader_xml_conf_static_discovery">
    <userDefinedID>5</userDefinedID>
</data_reader>
```

On the local DomainParticipant, you can load STATIC EDP configuration content specifying the file containing it.

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config("file://RemotePublisher.xml");
pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config("file://RemoteSubscriber.xml");
```

### XML

```xml
<participant profile_name="participant_profile_static_load_xml">
    <rtps>
        <builtin>
            <discovery_config>
                <static_edp_xml_config>file://RemotePublisher.xml</static_edp_xml_config>
                <static_edp_xml_config>file://RemoteSubscriber.xml</static_edp_xml_config>
            </discovery_config>
        </builtin>
    </rtps>
</participant>
```

Or you can specify the STATIC EDP configuration content directly.

| **C++**                                                                                                                                                                                                                                                                                          |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```c++<br/>DomainParticipantQos pqos;<br/><br/>pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config(<br/>    "data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \<br/>    "<staticdiscovery><participant><name>RTPSParticipant</name></participant></staticdiscovery>");<br/>``` |
