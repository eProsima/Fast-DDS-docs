<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="subscriberprofiles"></a>

<a id="datareader-profiles"></a>

# DataReader profiles

The DataReader profiles allow declaring [DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) from an XML file.
These profiles are defined within the `<data_reader>` XML tags.

<a id="datareader-xml-attributes"></a>

## DataReader XML attributes

The `<data_reader>` element has two attributes defined: `profile_name` and `is_default_profile`.

| Name                 | Description                                                                                                                                                                                                                                                                                                                                    | Use       |
|----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `profile_name`       | Sets the name under which the `<data_reader>` profile is registered in the DDS Domain,<br/>so that it can be loaded later by the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), as shown in<br/>[Loading and applying profiles](making_xml_profiles.md#loadingapplyingprofiles). | Mandatory |
| `is_default_profile` | Sets the `<data_reader>` profile as the default profile. Thus, if a default profile<br/>exists, it will be used when no other DataReader profile is specified at the<br/>DataReader’s creation.                                                                                                                                                | Optional  |

<a id="datareader-configuration"></a>

## DataReader configuration

The DataReader configuration is performed through the XML elements listed in the following table.

| Name                             | Description                                                                                                                                                                                                                              | Values                                                       | Default                                                                                                                                                      |
|----------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<topic>`                        | [TopicType](common.md#topictype) configuration of the DataReader.                                                                                                                                                                        | [TopicType](common.md#topictype)                             |                                                                                                                                                              |
| `<qos>`                          | Subscriber [QoS](common.md#commonqos) configuration.                                                                                                                                                                                     | [QoS](common.md#commonqos)                                   |                                                                                                                                                              |
| `<times>`                        | It allows configuring some time related<br/>parameters of the DataReader.                                                                                                                                                                | [ReaderTimes](#subtimes)                                     |                                                                                                                                                              |
| `<unicastLocatorList>`           | List of input unicast locators.<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                           | List of [LocatorListType](common.md#locatorlisttype)         |                                                                                                                                                              |
| `<multicastLocatorList>`         | List of input multicast locators.<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                         | List of [LocatorListType](common.md#locatorlisttype)         |                                                                                                                                                              |
| `<external_unicast_locators>`    | List of [External Locators](../transport/announced_locators.md#external-locators)<br/>to announce for the communication<br/>with this DataReader.                                                                                        | [ExternalLocatorListType](common.md#externallocatorlisttype) |                                                                                                                                                              |
| `<ignore_non_matching_locators>` | Whether to ignore locators received on<br/>announcements from other entities when<br/>they don’t match with any of the locators<br/>announced by this DataReader.                                                                        | `bool`                                                       | false                                                                                                                                                        |
| `<expects_inline_qos>`           | It indicates if QoS is expected inline.                                                                                                                                                                                                  | `bool`                                                       | `false`                                                                                                                                                      |
| `<historyMemoryPolicy>`          | Memory allocation kind for DataReaders’s  history.                                                                                                                                                                                       | [HistoryMemoryPolicy](common.md#historymemorypoliciesxml)    | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) |
| `<propertiesPolicy>`             | Additional configuration properties.                                                                                                                                                                                                     | [PropertiesPolicyType](common.md#propertiespolicytype)       |                                                                                                                                                              |
| `<userDefinedID>`                | Used for StaticEndpointDiscovery.                                                                                                                                                                                                        | `int16_t`                                                    | -1                                                                                                                                                           |
| `<entityID>`                     | Set the [`entity_id`](../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos9entity_idE) of the [RTPSEndpointQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos)  class. | `int16_t`                                                    | -1                                                                                                                                                           |
| `<matchedPublishersAllocation>`  | Sets the limits of the collection of matched<br/>DataWriters. See<br/>[ParticipantResourceLimitsQos](../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos).                                                       | [Allocation Configuration](common.md#commonalloc)            |                                                                                                                                                              |

**Example**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <data_reader profile_name="data_reader_profile_name">
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                </historyQos>
            </topic>

            <qos>
                <!-- QOS -->
            </qos>

            <times> <!-- readerTimesType -->
                <initial_acknack_delay>
                    <nanosec>70</nanosec>
                </initial_acknack_delay>

                <heartbeat_response_delay>
                    <nanosec>5</nanosec>
                </heartbeat_response_delay>
            </times>

            <unicastLocatorList>
                <!-- LOCATOR_LIST -->
                <locator>
                    <udpv4>
                        <address>192.168.0.1</address>
                    </udpv4>
                </locator>
            </unicastLocatorList>

            <multicastLocatorList>
                <!-- LOCATOR_LIST -->
                <locator>
                    <udpv4>
                        <address>192.168.0.1</address>
                    </udpv4>
                </locator>
            </multicastLocatorList>

            <external_unicast_locators>
                <!-- EXTERNAL_LOCATOR_LIST -->
                <udpv4 externality="1" cost="0" mask="24">
                    <address>100.100.100.10</address>
                    <port>12345</port>
                </udpv4>
            </external_unicast_locators>

            <ignore_non_matching_locators>true</ignore_non_matching_locators>

            <expects_inline_qos>true</expects_inline_qos>

            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>

            <!-- PROPERTIES_POLICY -->
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>PropertyName</name>
                        <value>PropertyValue</value>
                    </property>
                </properties>
            </propertiesPolicy>

            <userDefinedID>55</userDefinedID>

            <entityID>66</entityID>

            <matchedPublishersAllocation>
                <initial>0</initial>
                <maximum>0</maximum>
                <increment>1</increment>
            </matchedPublishersAllocation>
        </data_reader>
    </profiles>
</dds>
```

#### NOTE
- `LOCATOR_LIST` means it expects a [LocatorListType](common.md#locatorlisttype).
- `EXTERNAL_LOCATOR_LIST` means a [ExternalLocatorListType](common.md#externallocatorlisttype) is expected.
- `PROPERTIES_POLICY` means that the label is a [PropertiesPolicyType](common.md#propertiespolicytype) block.
- For `QOS` details, please refer to [QoS](common.md#commonqos).
- `TOPIC_TYPE` is detailed in section [TopicType](common.md#topictype).

<a id="subtimes"></a>

<a id="readertimes"></a>

### ReaderTimes

These parameters are included within [RTPSReliableReaderQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsreliablereaderqos) in the [ReaderTimes](../dds_layer/core/policy/eprosimaExtensions.md#readertimes) structure.

| Name                         | Description                                     | Values                                 | Default   |
|------------------------------|-------------------------------------------------|----------------------------------------|-----------|
| `<initial_acknack_delay>`    | Initial ACKNACK delay.                          | [DurationType](common.md#durationtype) | 70 ms     |
| `<heartbeat_response_delay>` | Response time delay when receiving a Heartbeat. | [DurationType](common.md#durationtype) | 5 ms      |
