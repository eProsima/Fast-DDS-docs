<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="publisherprofiles"></a>

<a id="datawriter-profiles"></a>

# DataWriter profiles

The DataWriter profiles allow for configuring [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) from an XML file.
These profiles are defined within the `<data_writer>` XML tags.

<a id="datawriter-xml-attributes"></a>

## DataWriter XML attributes

The `<data_writer>` element has two attributes defined: `profile_name` and `is_default_profile`.

| Name                 | Description                                                                                                                                                                                                                                                                                                                                    | Use       |
|----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `profile_name`       | Sets the name under which the `<data_writer>` profile is registered in the DDS Domain,<br/>so that it can be loaded later by the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), as shown in<br/>[Loading and applying profiles](making_xml_profiles.md#loadingapplyingprofiles). | Mandatory |
| `is_default_profile` | Sets the `<data_writer>` profile as the default profile. Thus, if a default profile<br/>exists, it will be used when no other DataWriter profile is specified at the<br/>DataWriter’s creation.                                                                                                                                                | Optional  |

<a id="datawriter-configuration"></a>

## DataWriter configuration

The DataWriter configuration is performed through the XML elements listed in the following table.

| Name                             | Description                                                                                                                                                                                                                               | Values                                                       | Default                                                                                                                                                      |
|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<topic>`                        | [TopicType](common.md#topictype) configuration of the DataWriter.                                                                                                                                                                         | [TopicType](common.md#topictype)                             |                                                                                                                                                              |
| `<qos>`                          | DataWriter [QoS](common.md#commonqos) configuration.                                                                                                                                                                                      | [QoS](common.md#commonqos)                                   |                                                                                                                                                              |
| `<times>`                        | It configures some time related parameters of the DataWriter.                                                                                                                                                                             | [WriterTimes](#pubtimes)                                     |                                                                                                                                                              |
| `<unicastLocatorList>`           | List of input unicast locators.<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                            | `<locator>`                                                  |                                                                                                                                                              |
| `<multicastLocatorList>`         | List of input multicast locators.<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                          | `<locator>`                                                  |                                                                                                                                                              |
| `<external_unicast_locators>`    | List of [External Locators](../transport/announced_locators.md#external-locators)<br/>to announce for the communication<br/>with this DataWriter.                                                                                         | [ExternalLocatorListType](common.md#externallocatorlisttype) |                                                                                                                                                              |
| `<ignore_non_matching_locators>` | Whether to ignore locators received on<br/>announcements from other entities when<br/>they don’t match with any of the locators<br/>announced by this DataWriter.                                                                         | `bool`                                                       | false                                                                                                                                                        |
| `<historyMemoryPolicy>`          | Memory allocation kind for DataWriter’s<br/>history. See [HistoryQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind).                                                                                    | [HistoryMemoryPolicy](common.md#historymemorypoliciesxml)    | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) |
| `<propertiesPolicy>`             | Additional configuration properties.                                                                                                                                                                                                      | [PropertiesPolicyType](common.md#propertiespolicytype)       |                                                                                                                                                              |
| `<userDefinedID>`                | Used for `EDPStatic`.                                                                                                                                                                                                                     | `int16_t`                                                    | -1                                                                                                                                                           |
| `<entityID>`                     | Sets the [`entity_id`](../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos9entity_idE) of the [RTPSEndpointQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos)  class. | `int16_t`                                                    | -1                                                                                                                                                           |
| `<matchedSubscribersAllocation>` | Sets the limits of the collection of matched<br/>DataReaders. See<br/>[ParticipantResourceLimitsQos](../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos).                                                        | [Allocation Configuration](common.md#commonalloc)            |                                                                                                                                                              |

**Example**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <data_writer profile_name="datawriter_profile_name">
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                </historyQos>
            </topic>

            <qos>
                <!-- QOS -->
            </qos>

            <times> <!-- writerTimesType -->
                <initial_heartbeat_delay>
                    <nanosec>12</nanosec>
                </initial_heartbeat_delay>

                <heartbeat_period>
                    <sec>3</sec>
                </heartbeat_period>

                <nack_response_delay>
                    <nanosec>5</nanosec>
                </nack_response_delay>

                <nack_supression_duration>
                    <sec>0</sec>
                </nack_supression_duration>
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

            <matchedSubscribersAllocation>
                <initial>0</initial>
                <maximum>0</maximum>
                <increment>1</increment>
            </matchedSubscribersAllocation>

            <!-- reader_filters_allocation cannot be configured using XML (yet) -->
        </data_writer>
    </profiles>
</dds>
```

#### NOTE
- `LOCATOR_LIST` means a [LocatorListType](common.md#locatorlisttype) is expected.
- `EXTERNAL_LOCATOR_LIST` means a [ExternalLocatorListType](common.md#externallocatorlisttype) is expected.
- `PROPERTIES_POLICY` means that the label is a [PropertiesPolicyType](common.md#propertiespolicytype) block.
- For `QOS` details, please refer to [QoS](common.md#commonqos).
- `TOPIC_TYPE` is detailed in section [TopicType](common.md#topictype).

<a id="pubtimes"></a>

<a id="writertimes"></a>

### WriterTimes

These parameters are included within [RTPSReliableWriterQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsreliablewriterqos) in the [WriterTimes](../dds_layer/core/policy/eprosimaExtensions.md#writertimes) structure.

| Name                         | Description                                                                                                                    | Values                                 | Default   |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------------|----------------------------------------|-----------|
| `<initial_heartbeat_delay>`  | Initial heartbeat delay.                                                                                                       | [DurationType](common.md#durationtype) | 12 ms     |
| `<heartbeat_period>`         | Periodic heartbeat period.                                                                                                     | [DurationType](common.md#durationtype) | 3 s       |
| `<nack_response_delay>`      | Delay to apply to the response of an ACKNACK message.                                                                          | [DurationType](common.md#durationtype) | 5 ms      |
| `<nack_supression_duration>` | This time allows the DataWriter to ignore NACK<br/>messages for a given period of time right after<br/>the data has been sent. | [DurationType](common.md#durationtype) | 0 ms      |
