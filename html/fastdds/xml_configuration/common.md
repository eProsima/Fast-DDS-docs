<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="commonxml"></a>

<a id="common"></a>

# Common

The preceding XML profiles define some XML elements that are common to several profiles.
This section aims to explain these common elements.

* [LocatorListType](#locatorlisttype)
* [ExternalLocatorListType](#externallocatorlisttype)
* [PropertiesPolicyType](#propertiespolicytype)
* [DurationType](#durationtype)
* [TopicType](#topictype)
  - [HistoryQoS](#hqos)
  - [ResourceLimitsQos](#rlsqos)
* [ThreadSettings](#threadsettingstype)
* [BuiltinTransports](#builtintransporttype)
* [QoS](#commonqos)
  - [Data-Sharing](#xml-datasharing)
  - [Deadline](#xml-deadline)
  - [DisableHeartbeatPiggyback](#xml-disableheartbeatpiggyback)
  - [DisablePositiveAcks](#xml-disablepositiveacks)
  - [Durability](#xml-durability)
  - [Entity Factory](#xml-entity-factory)
  - [GroupData](#xml-groupdata)
  - [LatencyBudget](#xml-latencybudget)
  - [Lifespan](#xml-lifespan)
  - [Liveliness](#xml-liveliness)
  - [Ownership](#xml-ownership)
  - [Ownership Strength](#xml-ownershipstrength)
  - [Partition](#xml-partition)
  - [PublishMode](#xml-publishmode)
  - [ReliabilityQosPolicy](#xml-reliability)
  - [TopicData](#xml-topicdata)
  - [UserData](#xml-userdata)
* [HistoryMemoryPolicy](#historymemorypoliciesxml)
* [Allocation Configuration](#commonalloc)

<a id="locatorlisttype"></a>

<a id="id1"></a>

## LocatorListType

It represents a list of [`Locator_t`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE).
LocatorListType is used inside other configuration parameter labels that expect a list of locators,
for example, in `<defaultUnicastLocatorList>`.
Therefore, LocatorListType is defined as a set of `<locator>` elements.
The `<locator>` element has a single child element that defines the transport protocol for which the locator is
defined. These are: `<udpv4>`, `<tcpv4>`, `<udpv6>`, and `<tcpv6>`.
The table presented below outlines each possible Locator’s field.

#### NOTE
[SHM transport](../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) locators cannot be configured as they are
automatically handled by SHM.

| Name              | Description                                                                         | Values                                      | Default   |
|-------------------|-------------------------------------------------------------------------------------|---------------------------------------------|-----------|
| `<port>`          | RTPS port number of the locator.<br/>*Physical port* in UDP, *logical port* in TCP. | `uint16_t`                                  | 0         |
| `<physical_port>` | TCP’s *physical port*.                                                              | `uint16_t`                                  | 0         |
| `<address>`       | IP address of the locator.                                                          | `string` (IPv4/IPv6 format<br/>or DNS name) | Empty     |
| `<unique_lan_id>` | The LAN ID uniquely identifies the LAN the<br/>locator belongs to (**TCPv4 only**). | `string` (16 bytes)                         | Empty     |
| `<wan_address>`   | WAN IPv4 address (**TCPv4 only**).                                                  | `string` (IPv4 format)                      | `0.0.0.0` |

**Example**

The following example shows the implementation of one locator of each transport protocol in
`<defaultUnicastLocatorList>`.

```xml
<defaultUnicastLocatorList>
    <locator>
        <udpv4>
            <!-- Access as physical, typical UDP usage -->
            <port>7400</port>
            <address>192.168.1.41</address>
        </udpv4>
        <udpv4>
            <!-- Access as physical, typical UDP usage -->
            <port>7600</port>
            <address>localhost</address>
        </udpv4>
    </locator>
    <locator>
        <tcpv4>
            <!-- Both physical and logical (port), useful in TCP transports -->
            <physical_port>5100</physical_port>
            <port>7400</port>
            <unique_lan_id>192.168.1.1.1.1.2.55</unique_lan_id>
            <wan_address>80.80.99.45</wan_address>
            <address>192.168.1.55</address>
        </tcpv4>
    </locator>
    <locator>
        <udpv6>
            <port>8844</port>
            <address>::1</address>
        </udpv6>
        <udpv6>
            <port>8888</port>
            <address>localhost</address>
        </udpv6>
    </locator>
    <locator>
        <tcpv6>
            <!-- Both physical and logical (port), useful in TCP transports -->
            <physical_port>5100</physical_port>
            <port>7400</port>
            <address>fe80::55e3:290:165:5af8</address>
        </tcpv6>
    </locator>
</defaultUnicastLocatorList>
```

#### NOTE
**[Pro]** For TCPv6 addresses, the zone index suffix (e.g. `fe80::1%eth0`) is accepted but ignored.
For more information, see [TCPv6 Addresses and Interfaces](../transport/tcp/tcp.md#transport-tcp-ipv6-addresses).

<a id="externallocatorlisttype"></a>

<a id="id2"></a>

## ExternalLocatorListType

It represents a list of external locator entries.
Each entry can be a `<udpv4>` or a `<udpv6>` tag.
These tags can be configured with the following attributes:

| Name          | Description                                                                                                                          | Values    |   Default |
|---------------|--------------------------------------------------------------------------------------------------------------------------------------|-----------|-----------|
| `externality` | Number of hops from the participant’s host to the<br/>LAN represented by the external locator.<br/>Valid values: from 1 to 255.      | `uint8_t` |         1 |
| `cost`        | Communication cost relative to other locators on<br/>the same externality level.<br/>Valid values: from 0 to 255.                    | `uint8_t` |         0 |
| `mask`        | Number of significant bits on the LAN represented<br/>by the external locator.<br/>Valid values: from 1 to 31 (UDPv4) or 127 (UDPv6) | `uint8_t` |        24 |

They should contain the following tags:

| Name        | Description                                                              | Values                                      |
|-------------|--------------------------------------------------------------------------|---------------------------------------------|
| `<port>`    | UDP port number of the locator.<br/>The UDP port number should be valid. | `uint16_t`                                  |
| `<address>` | IP address of the locator.                                               | `string` (IPv4/IPv6 format<br/>or DNS name) |

**Example**

The following example shows the implementation of one locator of each transport protocol in
`<default_external_unicast_locators>`.

```xml
<default_external_unicast_locators>
    <udpv4 externality="1" cost="0" mask="24">
        <address>100.100.100.10</address>
        <port>23456</port>
    </udpv4>
    <udpv6 externality="2" cost="0" mask="48">
        <address>::1</address>
        <port>1234</port>
    </udpv6>
</default_external_unicast_locators>
```

<a id="propertiespolicytype"></a>

<a id="id3"></a>

## PropertiesPolicyType

PropertiesPolicyType defines the `<propertiesPolicy>` element.
It allows the user to define a set of generic properties inside a `<properties>` element.
It is useful at defining extended or custom configuration parameters.

| Name          | Description                                                                        | Values   | Default   |
|---------------|------------------------------------------------------------------------------------|----------|-----------|
| `<name>`      | Name to identify the property.                                                     | `string` |           |
| `<value>`     | Property’s value.                                                                  | `string` |           |
| `<propagate>` | Indicates if it is going to be serialized along with the<br/>object it belongs to. | `bool`   | `false`   |

**Example**

```xml
<propertiesPolicy>
    <properties>
        <property>
            <name>Property1Name</name>
            <value>Property1Value</value>
            <propagate>false</propagate>
        </property>
        <property>
            <name>Property2Name</name>
            <value>Property2Value</value>
            <propagate>true</propagate>
        </property>
    </properties>
</propertiesPolicy>
```

<a id="durationtype"></a>

<a id="id4"></a>

## DurationType

DurationType expresses a period of time and it is commonly used inside other XML elements, such as in
`<leaseAnnouncement>` or `<leaseDuration>`.
A DurationType is defined by at least one mandatory element of two possible ones: `<sec>` plus `<nanosec>`.
An infinite value can be specified by using the values `DURATION_INFINITY`,
`DURATION_INFINITE_SEC` and `DURATION_INFINITE_NSEC`.

| Name        | Description            | Values     |   Default |
|-------------|------------------------|------------|-----------|
| `<sec>`     | Number of seconds.     | `int32_t`  |         0 |
| `<nanosec>` | Number of nanoseconds. | `uint32_t` |         0 |

#### NOTE
It is inconsistent to specify a negative value for the `<sec>` element, or
a `<nanosec>` value greater than or equal to 1 second (i.e., 1,000,000,000 nanoseconds).

**Example**

```xml
<discovery_config>
    <leaseDuration>
        <sec>DURATION_INFINITY</sec>
    </leaseDuration>
    <leaseAnnouncement>
        <sec>1</sec>
        <nanosec>856000</nanosec>
    </leaseAnnouncement>
</discovery_config>
```

<a id="topictype"></a>

<a id="id5"></a>

## TopicType

This XML element allows the configuration of the specific [HistoryQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#historyqospolicy) and [ResourceLimitsQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy)
QoS of the Datawriters and DataReaders in which this element is defined inside of.
Also, it sets the [TopicQos](../dds_layer/topic/topic/topic.md#dds-layer-topic-topicqos) configuration with the policies detailed.

| Name                  | Description                                                                                                                                                     | Values                       |
|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------|
| `<historyQos>`        | It controls the behavior of *Fast DDS*<br/>when the value of an instance changes<br/>before it is finally communicated to<br/>some of its existing DataReaders. | [HistoryQoS](#hqos)          |
| `<resourceLimitsQos>` | It controls the resources that *Fast DDS*<br/>can use in order to meet the<br/>requirements imposed by the application<br/>and other QoS settings.              | [ResourceLimitsQos](#rlsqos) |

**Example**

```xml
<data_writer profile_name="dataWriter_topic_example">
    <topic>
        <historyQos>
            <kind>KEEP_LAST</kind>
            <depth>20</depth>
        </historyQos>
        <resourceLimitsQos>
            <max_samples>5</max_samples>
            <max_instances>2</max_instances>
            <max_samples_per_instance>1</max_samples_per_instance>
            <allocated_samples>20</allocated_samples>
            <extra_samples>10</extra_samples>
        </resourceLimitsQos>
    </topic>
</data_writer>
```

<a id="hqos"></a>

<a id="historyqos"></a>

### HistoryQoS

It controls the behavior of *Fast DDS* when the value of an instance changes before it is finally
communicated to some of its existing DataReaders.
Please refer to [HistoryQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind) for further information on HistoryQoS.

| Name                                                                                                         | Description                                                                                                                                                                              | Values                                                                                                                                                 | Default                                                                                                                                                |
|--------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<kind>`                                                                                                     | *Fast DDS* will only attempt to keep the latest values<br/>of the instance  and discard the older ones.                                                                                  | [`KEEP_LAST`](../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) | [`KEEP_LAST`](../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) |
| *Fast DDS* will attempt to maintain and deliver all the<br/>values of the instance  to existing DataReaders. | [`KEEP_ALL`](../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE)                                     |                                                                                                                                                        |                                                                                                                                                        |
| `<depth>`                                                                                                    | It must be consistent with the [ResourceLimitsQos](#rlsqos)<br/>`<max_samples_per_instance>`<br/>element value. It must be verified that:<br/>`<depth>` <= `<max_samples_per_instance>`. | `uint32_t`                                                                                                                                             | 1                                                                                                                                                      |

<a id="rlsqos"></a>

<a id="resourcelimitsqos"></a>

### ResourceLimitsQos

It controls the resources that *Fast DDS* can use in order to meet the requirements imposed by the
application and other QoS settings.
Please refer to [ResourceLimitsQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy) for further information on ResourceLimitsQos.

| Name                         | Description                                                                             | Values    | Default            |
|------------------------------|-----------------------------------------------------------------------------------------|-----------|--------------------|
| `<max_samples>`              | It must verify that:<br/>`<max_samples>` >= `<max_samples_per_instance>`.               | `int32_t` | -1 (no<br/>limits) |
| `<max_instances>`            | It defines the maximum number of instances.                                             | `int32_t` | -1 (no<br/>limits) |
| `<max_samples_per_instance>` | It must verify that: [HistoryQos](#hqos)<br/>`<depth>` <= `<max_samples_per_instance>`. | `int32_t` | -1 (no<br/>limits) |
| `<allocated_samples>`        | It controls the maximum number of samples to be stored.                                 | `int32_t` | 100                |
| `<extra_samples>`            | The number of extra samples to allocate on the pool.                                    | `int32_t` | 1                  |

<a id="threadsettingstype"></a>

<a id="threadsettings"></a>

## ThreadSettings

It controls some OS settings for the *Fast DDS* created threads.
Please refer to [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for further information on ResourceLimitsQos.

| Name                                                                                                                                        | Description                                                                                                                                                                                                                                                      | Values     | Default   |
|---------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|-----------|
| [`scheduling_policy`](../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings17scheduling_policyE) | Configures the scheduling policy used for the thread.                                                                                                                                                                                                            | `int32_t`  | -1        |
| [`priority`](../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8priorityE)                    | Configures the thread’s priority.                                                                                                                                                                                                                                | `int32_t`  | -2^31     |
| [`affinity`](../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8affinityE)                    | On some systems (Windows, Linux), this is a bit mask for setting<br/>the threads affinity to each core individually. On MacOS, this<br/>sets the affinity tag for the thread, and the OS tries to share<br/>the L2 cache between threads with the same affinity. | `uint32_t` | 0         |
| [`stack_size`](../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings10stack_sizeE)               | Configures the thread’s stack size in bytes.                                                                                                                                                                                                                     | `int32_t`  | -1        |

<a id="builtintransporttype"></a>

<a id="builtintransports"></a>

## BuiltinTransports

It controls the builtin transports that will be used during the initialization of the DomainParticipant.
Please refer to [Managing the Builtin Transports](../rtps_layer/rtps_layer.md#rtps-layer-builtin-transports) for further information on builtin transports.

This type must follow this configuration:

| Name                 | Description                         | Values                                                                                                  | Default   |
|----------------------|-------------------------------------|---------------------------------------------------------------------------------------------------------|-----------|
| `<builtinTransport>` | Defines the builtin transport mode. | `NONE` <br/> `DEFAULT` <br/> `DEFAULTv6` <br/> `SHM` <br/> `UDPv4` <br/> `UDPv6`<br/><br/> `LARGE_DATA` | `DEFAULT` |

The `<builtinTransport>` tag can be configured with the following attributes:

| Name                      | Description                                                                                                                                               | Values     | Default    |
|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|------------|------------|
| `max_msg_size`            | Maximum message size that will be specified in<br/>the transport layer.<br/>Valid values: from 1 to (2^32)-1.                                             | `uint32_t` | 65500      |
| `sockets_size`            | Size of the send and receive socket buffers.<br/>Valid values: from 0 to (2^32)-1. <br/><br/>**[Pro]** If 0, TCP socket buffers are auto-tuned by the OS. | `uint32_t` | OS default |
| `non_blocking`            | Whether to use non-blocking send calls or not.<br/>Valid values: `true` or `false`.                                                                       | `bool`     | `false`    |
| `tcp_negotiation_timeout` | Timeout duration for logical port negotiation.<br/>Valid values: from 1 to (2^32)-1. <br/><br/>Only valid for `LARGE_DATA` mode.                          | `uint32_t` | 0          |

<a id="commonqos"></a>

<a id="qos"></a>

## QoS

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to define how each
[Entity](../dds_layer/core/entity/entity.md#dds-layer-core-entity) will behave.
Please refer to the [Policy](../dds_layer/core/policy/policy.md#dds-layer-core-policy) section for more information on QoS.

| Name                              | Description                                                                                                      | Values                                                      |
|-----------------------------------|------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------|
| `<data_representation>` **[Pro]** | See [DataRepresentationQosPolicy](../dds_layer/core/policy/xtypesExtensions.md#datarepresentationqospolicy).     | [Data Representation](#xml-datarepresentation)              |
| `<data_sharing>`                  | See [DataSharingQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy)                  | [Data-Sharing](#xml-datasharing)                            |
| `<deadline>`                      | See [DeadlineQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#deadlineqospolicy).                      | [Deadline](#xml-deadline)                                   |
| `<disable_heartbeat_piggyback>`   | See [DisableHeartbeatPiggyback](../dds_layer/core/policy/eprosimaExtensions.md#disableheartbeatpiggyback).       | [DisableHeartbeatPiggyback](#xml-disableheartbeatpiggyback) |
| `<disablePositiveAcks>`           | See [DisablePositiveACKsQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy). | [DisablePositiveAcks](#xml-disablepositiveacks)             |
| `<durability>`                    | See [DurabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#durabilityqospolicy).                  | [Durability](#xml-durability)                               |
| `<entity_factory>`                | See [EntityFactoryQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#entityfactoryqospolicy).            | [Entity Factory](#xml-entity-factory)                       |
| `<groupData>`                     | See [GroupDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#groupqospolicy).                        | [GroupData](#xml-groupdata)                                 |
| `<latencyBudget>`                 | See [LatencyBudgetQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#latencybudgetqospolicy).            | [LatencyBudget](#xml-latencybudget)                         |
| `<lifespan>`                      | See [LifespanQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#lifespanqospolicy).                      | [Lifespan](#xml-lifespan)                                   |
| `<liveliness>`                    | See [LivelinessQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy).                  | [Liveliness](#xml-liveliness)                               |
| `<ownership>`                     | See [OwnershipQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#ownershipqospolicy).                    | [Ownership](#xml-ownership)                                 |
| `<ownershipStrength>`             | See [OwnershipStrengthQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#ownershipstrengthqospolicy).    | [Ownership Strength](#xml-ownershipstrength)                |
| `<transport_priority>`            | See [TransportPriorityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#transportpriorityqospolicy).    | `int32_t` (default: 0)                                      |
| `<partition>`                     | See [PartitionQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy).                    | [Partition](#xml-partition)                                 |
| `<publishMode>`                   | See [PublishModeQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#publishmodeqospolicy).                 | [PublishMode](#xml-publishmode)                             |
| `<reliability>`                   | See [ReliabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy).                | [ReliabilityQosPolicy](#xml-reliability)                    |
| `<topicData>`                     | See [TopicDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#topicdataqospolicy).                    | [TopicData](#xml-topicdata)                                 |
| `<userData>`                      | See [UserDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy).                      | [UserData](#xml-userdata)                                   |

**Example**

```xml
<data_writer profile_name="pub_topic_qos">
<qos> <!-- writerQosPoliciesType -->
    <data_representation>
        <value>XCDR2</value>
    </data_representation>

    <data_sharing>
        <kind>AUTOMATIC</kind>
        <shared_dir>/home</shared_dir>
        <max_domains>10</max_domains>
        <domain_ids>
            <domainId>0</domainId>
            <domainId>11</domainId>
        </domain_ids>
    </data_sharing>

    <deadline>
        <period>
            <sec>1</sec>
        </period>
    </deadline>

    <!-- DataWriter specific QoS -->
    <disable_heartbeat_piggyback>true</disable_heartbeat_piggyback>

    <disablePositiveAcks>
        <enabled>true</enabled>
        <duration>
            <sec>1</sec>
        </duration>
    </disablePositiveAcks>

    <durability>
        <kind>VOLATILE</kind>
    </durability>

    <groupData>
        <value>1.a.2</value>
    </groupData>

    <!-- QoS policy pending implementation -->
    <latencyBudget>
        <duration>
            <sec>1</sec>
        </duration>
    </latencyBudget>

    <lifespan>
        <duration>
            <sec>1</sec>
        </duration>
    </lifespan>

    <liveliness>
        <kind>AUTOMATIC</kind>

        <lease_duration>
            <sec>1</sec>
        </lease_duration>

        <announcement_period>
            <sec>1</sec>
        </announcement_period>
    </liveliness>

    <ownership>
        <kind>EXCLUSIVE</kind>
    </ownership>

    <!-- DataWriter specific QoS -->
    <ownershipStrength>
        <value>50</value>
    </ownershipStrength>

    <transport_priority>12</transport_priority>

    <partition>
        <names>
            <name>part1</name>
            <name>part2</name>
        </names>
    </partition>

    <!-- DataWriter specific QoS -->
    <publishMode>
        <kind>ASYNCHRONOUS</kind>
    </publishMode>

    <reliability>
        <kind>BEST_EFFORT</kind>
        <max_blocking_time>
            <sec>1</sec>
        </max_blocking_time>
    </reliability>

    <topicData>
        <value>2.b.1</value>
    </topicData>

    <userData>
        <value>3.c.0</value>
    </userData>
</qos>
```

<a id="xml-datarepresentation"></a>

<a id="data-representation-pro"></a>

### Data Representation **[Pro]**

| Name      | Description                                                                                                                                                                                                                                                                                   | Values                   | Default    |
|-----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------|------------|
| `<value>` | Data representation offered by the DataWriter or requested by the DataReader.<br/>This element can be repeated to define a list of data representations, which is kept in the order given.<br/>See [DataRepresentationId](../dds_layer/core/policy/xtypesExtensions.md#datarepresentationid). | `XCDR` <br/><br/>`XCDR2` | Empty list |

`XCDR` corresponds to [`XCDR_DATA_REPRESENTATION`](../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE) and `XCDR2` to [`XCDR2_DATA_REPRESENTATION`](../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId25XCDR2_DATA_REPRESENTATIONE).
At least one `<value>` is required, and a value cannot be repeated within the list.
See [DataRepresentationQosPolicy](../dds_layer/core/policy/xtypesExtensions.md#datarepresentationqospolicy) for how an empty list is interpreted.

#### NOTE
A [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) offers a single data representation, so only the first `<value>` of the list is taken into
account. The remaining ones are ignored both when encoding the data and when matching with a [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
Defining more than one `<value>` is therefore only meaningful for [DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), which request every data
representation they accept.

**Example**

```xml
<data_writer profile_name="writer_xml_conf_data_representation_profile">
    <qos>
        <!-- The DataWriter offers XCDR version 2 -->
        <data_representation>
            <value>XCDR2</value>
        </data_representation>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_data_representation_profile">
    <qos>
        <!-- The DataReader accepts XCDR version 2 and XCDR version 1 -->
        <data_representation>
            <value>XCDR2</value>
            <value>XCDR</value>
        </data_representation>
    </qos>
</data_reader>
```

<a id="xml-datasharing"></a>

<a id="data-sharing"></a>

### Data-Sharing

| Name            | Description                                                                           | Values                                                                                                                                                                                                                                                                                                                                                                                     | Default                                                                                                                    |
|-----------------|---------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| `<kind>`        | See [DataSharingKind](../dds_layer/core/policy/eprosimaExtensions.md#datasharingkind) | [`AUTOMATIC`](../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE) <br/><br/>[`ON`](../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind2ONE) <br/><br/>[`OFF`](../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind3OFFE) <br/> | [`AUTOMATIC`](../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE) |
| `<shared_dir>`  | Directory used for the memory-mapped files.                                           | `string`                                                                                                                                                                                                                                                                                                                                                                                   | Empty                                                                                                                      |
| `<max_domains>` | Maximum number of Data-Sharing domain IDs<br/>in the local or remote endpoints.       | `uint32_t`                                                                                                                                                                                                                                                                                                                                                                                 | 0 (unlimited)                                                                                                              |
| `<domain_ids>`  | List of Data-Sharing domain IDs configured<br/>for the current endpoint.              | `<domainId>`                                                                                                                                                                                                                                                                                                                                                                               | Empty list                                                                                                                 |

| Name         | Description                                            | Values     |
|--------------|--------------------------------------------------------|------------|
| `<domainId>` | Domain ID to be used by the endpoint for Data-Sharing. | `uint32_t` |

<a id="xml-deadline"></a>

<a id="deadline"></a>

### Deadline

| Name       | Description                                                                                 | Values                        | Default                                                                                                                |
|------------|---------------------------------------------------------------------------------------------|-------------------------------|------------------------------------------------------------------------------------------------------------------------|
| `<period>` | See [DeadlineQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#deadlineqospolicy). | [DurationType](#durationtype) | [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |

<a id="xml-disableheartbeatpiggyback"></a>

<a id="disableheartbeatpiggyback"></a>

### DisableHeartbeatPiggyback

| Name                            | Description                                                                                                | Values   | Default   |
|---------------------------------|------------------------------------------------------------------------------------------------------------|----------|-----------|
| `<disable_heartbeat_piggyback>` | See [DisableHeartbeatPiggyback](../dds_layer/core/policy/eprosimaExtensions.md#disableheartbeatpiggyback). | `bool`   | `false`   |

#### IMPORTANT
This configuration is only available for [DataWriter QoS profile configuration](datawriter.md#publisherprofiles).

<a id="xml-disablepositiveacks"></a>

<a id="disablepositiveacks"></a>

### DisablePositiveAcks

| Name         | Description                                                                                                      | Values                        | Default                                                                                                                |
|--------------|------------------------------------------------------------------------------------------------------------------|-------------------------------|------------------------------------------------------------------------------------------------------------------------|
| `<enabled>`  | See [DisablePositiveACKsQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy). | `bool`                        | `false`                                                                                                                |
| `<duration>` | See [DisablePositiveACKsQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy). | [DurationType](#durationtype) | [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |

<a id="xml-durability"></a>

<a id="durability"></a>

### Durability

| Name     | Description                                                                                    | Values                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Default                                                                                                                                                                                                                                                                                                                                                                             |
|----------|------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<kind>` | See [DurabilityQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#durabilitykind). | [`VOLATILE`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE) <br/><br/>[`TRANSIENT_LOCAL`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) <br/><br/>[`TRANSIENT`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE) <br/><br/>[`PERSISTENT`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind25PERSISTENT_DURABILITY_QOSE) | DataReaders: [`VOLATILE`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE) <br/><br/>DataWriters: [`TRANSIENT_LOCAL`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) <br/> |

<a id="xml-entity-factory"></a>

<a id="entity-factory"></a>

### Entity Factory

| Name                            | Description                                                                                           | Values   | Default   |
|---------------------------------|-------------------------------------------------------------------------------------------------------|----------|-----------|
| `<autoenable_created_entities>` | See [EntityFactoryQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#entityfactoryqospolicy). | `bool`   | `true`    |

<a id="xml-groupdata"></a>

<a id="groupdata"></a>

### GroupData

| Name      | Description                                                                               | Values                                                                                                              | Default   |
|-----------|-------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------|-----------|
| `<value>` | See [GroupDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#groupqospolicy). | `string` (std::vector<[`octet`](../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)>) | Empty     |

<a id="xml-latencybudget"></a>

<a id="latencybudget"></a>

### LatencyBudget

| Name         | Description                                                                                           | Values                        |   Default |
|--------------|-------------------------------------------------------------------------------------------------------|-------------------------------|-----------|
| `<duration>` | See [LatencyBudgetQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#latencybudgetqospolicy). | [DurationType](#durationtype) |         0 |

<a id="xml-lifespan"></a>

<a id="lifespan"></a>

### Lifespan

| Name         | Description                                                                                 | Values                        | Default                                                                                                                |
|--------------|---------------------------------------------------------------------------------------------|-------------------------------|------------------------------------------------------------------------------------------------------------------------|
| `<duration>` | See [LifespanQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#lifespanqospolicy). | [DurationType](#durationtype) | [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |

<a id="xml-liveliness"></a>

<a id="liveliness"></a>

### Liveliness

| Name                                                                                                                                                                                    | Description                                                                                                 | Values                                                                                                                                                          | Default                                                                                                                                                         |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<kind>`                                                                                                                                                                                | See<br/>[LivelinessQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicykind). | [`AUTOMATIC`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE) | [`AUTOMATIC`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE) |
| [`MANUAL_BY_PARTICIPANT`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) |                                                                                                             |                                                                                                                                                                 |                                                                                                                                                                 |
| [`MANUAL_BY_TOPIC`](../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             |                                                                                                             |                                                                                                                                                                 |                                                                                                                                                                 |
| `<lease_duration>`                                                                                                                                                                      | See [LivelinessQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy).             | [DurationType](#durationtype)                                                                                                                                   | [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE)                                          |
| `<announcement_period>`                                                                                                                                                                 | See [LivelinessQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy).             | [DurationType](#durationtype)                                                                                                                                   | [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE)                                          |

<a id="xml-ownership"></a>

<a id="ownership"></a>

### Ownership

| Name     | Description                                                                                           | Values                                                                                                                                                                                                                                                                                                                        | Default                                                                                                                                                |
|----------|-------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<kind>` | See [OwnershipQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#ownershipqospolicykind). | [`SHARED`](../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE) <br/><br/>[`EXCLUSIVE`](../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE) | [`SHARED`](../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE) |

<a id="xml-ownershipstrength"></a>

<a id="ownership-strength"></a>

### Ownership Strength

| Name      | Description                                                                                                       | Values     |   Default |
|-----------|-------------------------------------------------------------------------------------------------------------------|------------|-----------|
| `<value>` | See<br/>[OwnershipStrengthQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#ownershipstrengthqospolicy). | `uint32_t` |         0 |

#### IMPORTANT
This configuration is only available for [DataWriter QoS profile configuration](datawriter.md#publisherprofiles).

<a id="xml-partition"></a>

<a id="partition"></a>

### Partition

| Name      | Description                                                                                                                                                                       | Values   |
|-----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| `<names>` | It comprises a set of `<name>` elements containing the name of each<br/>partition.  See [PartitionQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy). | `<name>` |

<a id="xml-publishmode"></a>

<a id="publishmode"></a>

### PublishMode

| Name                     | Description                                                                                      | Values         | Default        |
|--------------------------|--------------------------------------------------------------------------------------------------|----------------|----------------|
| `<kind>`                 | See [PublishModeQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#publishmodeqospolicy). | `ASYNCHRONOUS` | `ASYNCHRONOUS` |
| `SYNCHRONOUS`            |                                                                                                  |                |                |
| `<flow_controller_name>` | [FlowControllersQos](../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos) name.    | `<string>`     | Empty          |

#### IMPORTANT
This configuration is only available for [DataWriter QoS profile configuration](datawriter.md#publisherprofiles).

<a id="xml-reliability"></a>

<a id="reliabilityqospolicy"></a>

### ReliabilityQosPolicy

| Name                                                                                                                                                             | Description                                                                                                   | Values                                                                                                                                                                 | Default                                                                                                                                                                                                                                                                                                                                                                         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<kind>`                                                                                                                                                         | See<br/>[ReliabilityQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicykind). | [`BEST_EFFORT`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) | DataReaders: [`BEST_EFFORT`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE)<br/><br/><br/>DataWriters: [`RELIABLE`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) |
| [`RELIABLE`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) |                                                                                                               |                                                                                                                                                                        |                                                                                                                                                                                                                                                                                                                                                                                 |
| `<max_blocking_time>`                                                                                                                                            | See [ReliabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy).             | [DurationType](#durationtype)                                                                                                                                          | 100 ms                                                                                                                                                                                                                                                                                                                                                                          |

<a id="xml-topicdata"></a>

<a id="topicdata"></a>

### TopicData

| Name      | Description                                                                                   | Values                                                                                                              | Default   |
|-----------|-----------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------|-----------|
| `<value>` | See [TopicDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#topicdataqospolicy). | `string` (std::vector<[`octet`](../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)>) | Empty     |

<a id="xml-userdata"></a>

<a id="userdata"></a>

### UserData

| Name      | Description                                                                                 | Values                                                                                                              | Default   |
|-----------|---------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------|-----------|
| `<value>` | See [UserDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy). | `string` (std::vector<[`octet`](../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)>) | Empty     |

<a id="historymemorypoliciesxml"></a>

<a id="historymemorypolicy"></a>

## HistoryMemoryPolicy

Indicates the way the memory is managed in terms of dealing with the CacheChanges of the [RTPSEndpointQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos).

| Name                    | Description                                                                                                                                 | Values                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Default                                                                                                                                                      |
|-------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<historyMemoryPolicy>` | Four different options as described<br/>in [MemoryManagementPolicy](../dds_layer/core/policy/eprosimaExtensions.md#memorymanagementpolicy). | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) <br/><br/>[`PREALLOCATED_WITH_REALLOC`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE) <br/><br/>[`DYNAMIC`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy27DYNAMIC_RESERVE_MEMORY_MODEE) <br/><br/>[`DYNAMIC_REUSABLE`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy28DYNAMIC_REUSABLE_MEMORY_MODEE) | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) |

**Example**

```xml
<data_writer profile_name="data_writer_historyMemoryPolicy">
    <!-- ...  -->
    <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
</data_writer>

<data_reader profile_name="data_reader_historyMemoryPolicy">
    <!-- ...  -->
    <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
</data_reader>
```

<a id="commonalloc"></a>

<a id="allocation-configuration"></a>

## Allocation Configuration

The `<allocation>` element allows to control the allocation behavior of internal collections for which the number
of elements depends on the number of entities in the system.
For instance, there are collections inside a DataWriter which depend on the number of DataReaders matching with it.
Please refer to [ParticipantResourceLimitsQos](../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos) for a detailed documentation on DomainParticipant allocation,
and to [Tuning allocations](../use_cases/realtime/allocations.md#realtime-allocations) for detailed information on how to tune allocation related parameters.

| Name          | Description                                                                 | Values     | Default            |
|---------------|-----------------------------------------------------------------------------|------------|--------------------|
| `<initial>`   | Number of elements for which space is initially allocated.                  | `uint32_t` | 0                  |
| `<maximum>`   | Maximum number of elements for which space will be allocated.               | `uint32_t` | 0 (Means no limit) |
| `<increment>` | Number of new elements that will be allocated when more space is necessary. | `uint32_t` | 1                  |

<a id="flowcontrollers-xml"></a>

<a id="flow-controller-descriptors"></a>

## Flow Controller Descriptors

This `<flow_controller_descriptor_list>` element configures the list of flow controllers of a participant,
so they can later be used on its DataWriters.
Please refer to [FlowControllersQos](../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos) for a detailed documentation.

| Data Member Name         | Type                                                                                                                                                                | Default Value                                                                                                                                   |
|--------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| `<name>`                 | `string`                                                                                                                                                            | Empty                                                                                                                                           |
| `<scheduler>`            | [`FlowControllerSchedulerPolicy`](../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicyE) | [`FIFO`](../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy4FIFOE) |
| `<max_bytes_per_period>` | `int32_t`                                                                                                                                                           | 0 (i.e. infinite)                                                                                                                               |
| `<period_ms>`            | `uint64_t`                                                                                                                                                          | 100                                                                                                                                             |
| `<sender_thread>`        | [ThreadSettings](#threadsettingstype)                                                                                                                               |                                                                                                                                                 |
