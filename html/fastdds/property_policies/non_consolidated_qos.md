<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="property-policies-non-consolidated-qos"></a>

<a id="non-consolidated-qos"></a>

# Non consolidated QoS

The [PropertyPolicyQos Options](property_policies.md#property-policies) are used to develop new [eProsima Extensions](../dds_layer/core/policy/eprosimaExtensions.md#eprosima-extensions) QoS.
Before consolidating a new QoS Policy, it is usually set using this generic QoS Policy.
Consequently, this section is prone to frequent updates so the user is advised to check latest changes after upgrading
to a different release version.

<a id="push-mode-qos-policy"></a>

<a id="datawriter-operating-mode-qos-policy"></a>

## `DataWriter operating mode` QoS Policy

By default, *Fast DDS* [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) are enabled using `push mode`.
This implies that they will add new samples into their queue, and then immediately deliver them to matched readers.
For writers that produce non periodic bursts of data, this may imply saturating the network with a lot of packets,
increasing the possibility of losing them on unreliable (i.e. UDP) transports.
Depending on their QoS, DataReaders may also have to ignore some received samples, so they will have to be resent.

Configuring the DataWriters on `pull mode` offers an alternative by letting each reader pace its own data stream.
It works by the writer notifying the reader what it is available, and waiting for it to request only as much as it
can handle.
At the cost of greater latency, this model can deliver reliability while using far fewer packets than `push mode`.

DataWriters periodically announce the state of their queue by means of a heartbeat.
Upon reception of the heartbeat, DataReaders will request the DataWriter to send the samples they want to process.
Consequently, the publishing rate can be tuned setting the heartbeat period accordingly.
See [Tuning Heartbeat Period](../use_cases/large_data/large_data.md#tuning-heartbeat-period) for more details.

| PropertyPolicyQos name   | PropertyPolicyQos value   | Default value   |
|--------------------------|---------------------------|-----------------|
| `"fastdds.push_mode"`    | `"true"`/`"false"`        | `"true"`        |

### C++

```c++
DataWriterQos wqos;

// Enable pull mode
wqos.properties().properties().emplace_back(
    "fastdds.push_mode",
    "false");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
<data_writer profile_name="pull_mode_datawriter_xml_profile">
    <propertiesPolicy>
        <properties>
            <!-- Enable pull mode -->
            <property>
                <name>fastdds.push_mode</name>
                <value>false</value>
            </property>
        </properties>
    </propertiesPolicy>
</data_writer>
</profiles>
```

#### NOTE
* Communication to readers running on the same process ([Intra-process delivery](../transport/intraprocess.md#intraprocess-delivery)) will always use `push mode`.
* Communication to [`BEST_EFFORT_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) readers will always use `push mode`.

#### WARNING
* It is inconsistent to enable the `pull mode` and also set the [`ReliabilityQosPolicyKind`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE) to
  [`BEST_EFFORT_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE).
* It is inconsistent to enable the `pull mode` and also set the [`heartbeat_period`](../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes16heartbeat_periodE) to
  [`c_TimeInfinite`](../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE).

<a id="unique-network-flows-qos-policy"></a>

<a id="id1"></a>

## Unique network flows QoS Policy

This QoS Policy allows a [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) to be created with a unique network flow, i.e. listening on a
port that is not shared with any other endpoint of the same [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
This is useful when the application needs to apply specific Network QoS parameters (for example, through 3GPP/5QI
configuration on the underlying networking equipment) to the traffic of a particular topic.

The feature is enabled by adding the property `"fastdds.unique_network_flows"` to the
[PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) of the [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).

| PropertyPolicyQos name           | PropertyPolicyQos value   | Default value   |
|----------------------------------|---------------------------|-----------------|
| `"fastdds.unique_network_flows"` | `""`                      | Not set         |

For a complete description of network flows in *Fast DDS*, the APIs used to inspect them, and a usage example,
see [Unique network flows](../use_cases/unique_network_flows/unique_network_flows.md#use-case-unique-flows).

<a id="property-policies-statistics"></a>

<a id="statistics-module-settings"></a>

## Statistics Module Settings

Fast DDS Statistics Module uses the [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) to indicate the statistics DataWriters that are enabled
automatically (see [Automatically enabling statistics DataWriters](../statistics/dds_layer/domainParticipant.md#auto-enabling-statistics-datawriters)).
In this case, the property value is a semicolon separated list containing the
[statistics topic name aliases](../statistics/dds_layer/topic_names.md#statistics-topic-names) of those DataWriters that the user wants to enable.

| PropertyPolicyQos name   | PropertyPolicyQos value                                                                                                    | Default value   |
|--------------------------|----------------------------------------------------------------------------------------------------------------------------|-----------------|
| `"fastdds.statistics"`   | Semicolon separated list of [statistics topic name aliases](../statistics/dds_layer/topic_names.md#statistics-topic-names) | `""`            |

### C++

```c++
DomainParticipantQos pqos;

// Activate Fast DDS Statistics module
pqos.properties().properties().emplace_back("fastdds.statistics",
        "HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC");
```

### XML

```xml
<participant profile_name="statistics_domainparticipant_conf_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Activate Fast DDS Statistics Module -->
                <property>
                    <name>fastdds.statistics</name>
                    <value>HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="property-policies-physical-data"></a>

<a id="physical-data-in-discovery-information"></a>

### Physical Data in Discovery Information

It is possible to include the information conveyed in the [PHYSICAL_DATA_TOPIC](../statistics/dds_layer/topic_names.md#statistics-topic-names-physical) into the participant
discovery message, a.k.a *DATA[p]* (see [Discovery phases](../discovery/discovery.md#disc-phases)).
This is done by setting the following properties within the [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos):

| PropertyPolicyQos name            | PropertyPolicyQos value                                  | Default value without  `FASTDDS_STATISTICS`   | Default value with  `FASTDDS_STATISTICS`   |
|-----------------------------------|----------------------------------------------------------|-----------------------------------------------|--------------------------------------------|
| `"fastdds.physical_data.host"`    | Name of the host computer in which  the application runs | Not set                                       | `""`                                       |
| `"fastdds.physical_data.user"`    | Name of the user running the application                 | Not set                                       | `""`                                       |
| `"fastdds.physical_data.process"` | Name of the process running the application              | Not set                                       | `""`                                       |

Whenever any of these properties is defined within the [`DomainParticipantQos`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE), the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) *DATA[p]*
will contained the set value.
Furthermore, if any of these properties is set to a value of `""`, which is the default when `FASTDDS_STATISTICS` is
defined (see [CMake options](../../installation/configuration/cmake_options.md#cmake-options)), *Fast DDS* will automatically populate the value using the following convention:

* `"fastdds.physical_data.host"`: Host name as returned by [asio::ip::host_name()](https://think-async.com/Asio/asio-1.22.0/doc/asio/reference/ip__host_name.html),
  followed by `":<default data sharing domain id>"`
* `"fastdds.physical_data.user"`: Name of the user running the application, or `"unknown"` if it could not be
  retrieved.
* `"fastdds.physical_data.process"`: The process ID of the process in which the application is running.

All the previous entails that adding physical information to the *DATA[p]* can be done regardless of whether
`FASTDDS_STATISTICS` is defined, and that it is possible to let *Fast DDS* set some default values into the
reported `host`, `user`, and `process`:

1. If `FASTDDS_STATISTICS` is defined, and the user does not specify otherwise, *Fast DDS* will set default values to
   the physical properties of the *DATA[p]*.
2. If `FASTDDS_STATISTICS` is defined, and the user sets values to the properties, the user settings are honored.
3. If `FASTDDS_STATISTICS` is defined, and the user removes the physical properties from the
   [`DomainParticipantQos`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE), then no physical information is transmitted in the *DATA[p]*.
4. If `FASTDDS_STATISTICS` is not defined, it is still possible to transmit physical information in the *DATA[p]* by
   setting the aforementioned properties:
   1. If set to `""`, then *Fast DDS* will populate their value according to the described rules.
   2. If set to something other than `""`, then the set value will be transmitted in the *DATA[p]* as-is.

In case `FASTDDS_STATISTICS` is defined, and the reporting of statistics over the `DISCOVERY_TOPIC` is enabled (see
[Statistics Module Settings](#property-policies-statistics)), then the physical information included in the *DATA[p]* is also transmitted over
the `DISCOVERY_TOPIC` (see [PHYSICAL_DATA_TOPIC](../statistics/dds_layer/topic_names.md#statistics-topic-names-physical)) whenever one [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) discovers
another one.

#### NOTE
**[Pro]** In *Fast DDS* **[Pro]**, the statistics module is always compiled in, so `FASTDDS_STATISTICS` is always
considered defined in this section.
Besides, the `DISCOVERY_TOPIC` and the [PHYSICAL_DATA_TOPIC](../statistics/dds_layer/topic_names.md#statistics-topic-names-physical) are not available in
*Fast DDS* **[Pro]** (see [Statistics topics not available in Fast DDS](../statistics/dds_layer/topic_names.md#statistics-topic-names-removed) ), so the physical information is only transmitted in the
*DATA[p]*.

### C++

```c++
/* Create participant which announces default physical properties */
DomainParticipantQos pqos_default_physical;
// NOTE: Setting the properties to "" is not necessary, as that is already their default value
pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.host", "");
pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.user", "");
pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.process", "");
DomainParticipant* participant_with_physical = DomainParticipantFactory::get_instance()->create_participant(0,
                pqos_default_physical);

/* Create participant which announces custom physical properties */
DomainParticipantQos pqos_custom_physical;
// NOTE: Clear the properties before setting them, as they are already present with their default value
// pqos_custom_physical.properties().properties().clear()
pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.host", "custom_hostname");
pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.user", "custom_username");
pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.process", "custom_process");
DomainParticipant* participant_custom_physical = DomainParticipantFactory::get_instance()->create_participant(0,
                pqos_custom_physical);

/* Create participant which does not announce physical properties */
DomainParticipantQos pqos_no_physical;
pqos_no_physical.properties().properties().clear();
DomainParticipant* participant_without_physical = DomainParticipantFactory::get_instance()->create_participant(
    0, pqos_no_physical);

/* Load physical properties from default XML file */
DomainParticipantFactory::get_instance()->load_profiles();
DomainParticipantQos pqos_default_xml_physical =
        DomainParticipantFactory::get_instance()->get_default_participant_qos();
DomainParticipant* participant_default_xml_physical =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos_default_xml_physical);

/* Load physical properties from specific XML file */
DomainParticipantFactory::get_instance()->load_XML_profiles_file("somefile.xml");
DomainParticipantFactory::get_instance()->load_profiles();
DomainParticipantQos pqos_custom_xml_physical =
        DomainParticipantFactory::get_instance()->get_default_participant_qos();
DomainParticipant* participant_custom_xml_physical =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos_custom_xml_physical);
```

### XML

```xml
<?xml version="1.0" encoding="utf-8"?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="statistics_participant" is_default_profile="true">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.physical_data.host</name>
                            <value>custom_hostname</value>
                        </property>
                        <property>
                            <name>fastdds.physical_data.user</name>
                            <value>custom_username</value>
                        </property>
                        <property>
                            <name>fastdds.physical_data.process</name>
                            <value>custom_process</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### IMPORTANT
The properties set using XML override those in the default QoS, which means that it is possible to set the physical
properties using XML regardless of whether `FASTDDS_STATISTICS` is defined.
However, it is not possible to remove the properties using XML, meaning that an application using *Fast DDS* with
`FASTDDS_STATISTICS` enabled which does not want for the physical information to be transmitted in the
[`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) *DATA[p]* must remove the properties using the aforementioned C++ API.

<a id="property-policies-partitions"></a>

<a id="endpoint-partitions"></a>

## Endpoint Partitions

Fast DDS uses this [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) to define which partitions does an endpoint belong to. This property
follows the same logic regarding matching as the [PartitionQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy) that can be defined for Publishers and
Subscribers.

This property’s value is a semicolon separated list containing the partition names the user wants this endpoint
to belong to.

#### IMPORTANT
If both a Publisher and one of its DataWriters have conflicting partition configuration, this is, a DataWriter
has this property defined while the Publisher has the [PartitionQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy) defined, the DataWriter
configuration takes precedence and the Publisher [PartitionQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy) is ignored for this endpoint. This applies
to Subscribers and their DataReaders as well.

This property will be automatically set when creating DataReaders and DataWriters using the create_with_profile
functions. It cannot be changed after the entity has been created.

| PropertyPolicyQos name   | PropertyPolicyQos value                     | Default value   |
|--------------------------|---------------------------------------------|-----------------|
| `"partitions"`           | Semicolon separated list of partition names | `""`            |

### C++

```c++
DataWriterQos wqos;

// Add partitions
wqos.properties().properties().emplace_back(
    "partitions",
    "part1;part2");

DataReaderQos rqos;

// Add partitions
rqos.properties().properties().emplace_back(
    "partitions",
    "part1;part2");
```

### XML

```xml
<data_writer profile_name="pub_partition_example">
    <qos>
        <partition>
            <names>
                <name>part1</name>
                <name>part2</name>
            </names>
        </partition>
    </qos>
</data_writer>

<data_reader profile_name="sub_partition_example">
    <qos>
        <partition>
            <names>
                <name>part1</name>
                <name>part2</name>
            </names>
        </partition>
    </qos>
</data_reader>
```

<a id="property-policies-edp-exchange-format"></a>

<a id="static-discovery-s-exchange-format"></a>

## Static Discovery’s Exchange Format

Static Discovery exchanges data in the Participant Discovery Phase (PDP).
Currently there are two different exchange formats which can be selected using the property
`dds.discovery.static_edp.exchange_format`.

| PropertyPolicyQos value   | Description                                                                                                                                   | Default   |
|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `"v1"`                    | Standard exchange format for Static Discovery.                                                                                                | ✅         |
| `"v1_Reduced"`            | Format which reduces the necessary network bandwidth to transmit Static<br/>Discovery’s information in the Participant Discovery Phase (PDP). |           |

### C++

```c++
DomainParticipantQos participant_qos;
participant_qos.properties().properties().emplace_back(
    "dds.discovery.static_edp.exchange_format",
    "v1_Reduced"
    );
```

### XML

```xml
<participant profile_name="participant_xml_conf_static_discovery_format_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <property>
                    <name>dds.discovery.static_edp.exchange_format</name>
                    <value>v1_Reduced</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="property-policies-shm-enforce-metatraffic"></a>

<a id="shm-transport-meta-traffic-enforcement"></a>

## SHM transport meta-traffic enforcement

A DomainParticipant will by default configure both a [UDP Transport](../transport/udp/udp.md#transport-udp-udp) and a
[Shared Memory Transport](../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory).
When a participant on another process in the same host is discovered, the endpoint discovery
might be done using either transport.

Avoiding Shared Memory communication for discovery traffic can save valuable resources.
The behavior regarding this can be configured using the property `fastdds.shm.enforce_metatraffic`.

| PropertyPolicyQos value   | Description                                                                                                                                                             | Default   |
|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `"none"`                  | Use other transports for meta-traffic.                                                                                                                                  | ✅         |
| `"unicast"`               | Enable SHM transport unicast communications.                                                                                                                            |           |
| `"all"`                   | Enable SHM transport unicast and multicast communications.<br/>This will enable discovery between SHM only participants<br/>and participants having several transports. |           |

#### NOTE
When SHM is the only transport configured for a participant, the setting of this property is ignored,
and considered to be `"all"`.

### C++

```c++
DomainParticipantQos participant_qos;

// SHM transport will listen for unicast meta-traffic
participant_qos.properties().properties().emplace_back(
    "fastdds.shm.enforce_metatraffic",
    "unicast");
```

### XML

```xml
<participant profile_name="participant_xml_conf_shm_enforce_metatraffic_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <property>
                    <name>fastdds.shm.enforce_metatraffic</name>
                    <value>unicast</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="property-policies-shm-host-id-only"></a>

<a id="shm-transport-locators-format-pro"></a>

## SHM transport locators format **[Pro]**

In *Fast DDS*, and in *Fast DDS* **[Pro]** prior to version 3.7.0, the locators format for SHM transport included a
two byte ID based on the IP addresses of the host, which was used to identify the host in which the participant
was running.
Starting from *Fast DDS* **[Pro]** version 3.7.0, the locators format for SHM transport has been changed to also include
a 13-byte ID based on the machine ID of the host.

When the machine ID part of the locator is set, it is used to identify the host in which a
participant is running, and the IP-based host ID part of the locator is ignored.
When the machine ID part of the locator is not set, the IP-based host ID part of the locator is
used to identify the host in which a participant is running.

In certain scenarios, where virtual machines are cloned or containers are created from the same
image, the machine ID of the host may be the same for different hosts, which may lead to
participants running in different hosts being considered to be running in the same host, and
therefore not being able to communicate.
In order to avoid this situation, the property `fastdds.shm.only_host_id_on_locators` can be set
to `true` to force the SHM transport to use only the IP-based host ID part of the locator, both
in the announced locators and when processing the locators of remote participants.

### C++

```c++
DomainParticipantQos participant_qos;

// SHM transport will use only the IP-based host ID part of the locator
participant_qos.properties().properties().emplace_back(
    "fastdds.shm.only_host_id_on_locators",
    "true");
```

### XML

```xml
<participant profile_name="participant_xml_conf_shm_only_host_id_on_locators_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <property>
                    <name>fastdds.shm.only_host_id_on_locators</name>
                    <value>true</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="property-max-message-size"></a>

<a id="maximum-message-size"></a>

## Maximum Message Size

One common requirement is the differentiation between the maximum size of received and sent datagrams.
This capability is especially important in scenarios where a system might need to handle large incoming
data sizes but should restrict the size of the data it sends to prevent overwhelming network resources
or complying with network traffic policies.
The primary attribute for controlling datagram size is maxMessageSize, which sets the upper limit
for both the size of datagrams that can be received and those that can be sent.
Property `fastdds.max_message_size` allows restricting the size of outgoing datagrams without
changing the size of incoming ones.
This property allows for the specific configuration of the maximum number of bytes for datagrams that
are sent.
By configuring this property to a value lower than the smallest maxMessageSize across all transports,
applications can achieve a lower sending limit while maintaining the ability to receive larger datagrams.

| PropertyPolicyQos name       | PropertyPolicyQos value   | Default value   |
|------------------------------|---------------------------|-----------------|
| `"fastdds.max_message_size"` | `uint32_t`                | `"4294967295"`  |

#### NOTE
An invalid value of `fastdds.max_message_size` would log an error,
and the default value will be used.

<a id="setting-max-message-size-participant"></a>

<a id="setting-fastdds-max-message-size-at-participant-level"></a>

### Setting `fastdds.max_message_size` At Participant Level

### C++

```c++
DomainParticipantQos pqos;

// Set maximum number of bytes of the datagram to be sent
pqos.properties().properties().emplace_back(
    "fastdds.max_message_size",
    "1200");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<participant profile_name="max_message_size_participant_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Set the maximum size in bytes for all RTPS datagrams sent by the participant -->
                <property>
                    <name>fastdds.max_message_size</name>
                    <value>1200</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="setting-max-message-size-writer"></a>

<a id="setting-fastdds-max-message-size-at-writer-level"></a>

### Setting `fastdds.max_message_size` At Writer Level

### C++

```c++
DataWriterQos wqos;

// Set maximum number of bytes of the datagram to be sent
wqos.properties().properties().emplace_back(
    "fastdds.max_message_size",
    "1200");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<data_writer profile_name="max_msg_size_datawriter_xml_profile">
    <propertiesPolicy>
        <properties>
            <!-- Set the maximum size in bytes for all RTPS datagrams sent by the writer -->
            <property>
                <name>fastdds.max_message_size</name>
                <value>1200</value>
            </property>
        </properties>
    </propertiesPolicy>
</data_writer>
```

<a id="property-type-propagation"></a>

<a id="type-propagation"></a>

## Type Propagation

By default, *Fast DDS* leverages [Remote Data Types Discovery](../xtypes/discovery_matching.md#xtypes-discovery-matching) both to discover remote types and to propagate local
ones.
Type propagation entails both adding meta-information to the EDP messages (see [Discovery phases](../discovery/discovery.md#disc-phases)) and using a
dedicated set of builtin endpoints to transmit type definitions between the different [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) in the
network.

Depending on the application design and deployment, it might be desirable to change the default type propagation
behavior to save bandwidth and/or resources.
For this reason, *Fast DDS* [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) can be configured with a property `fastdds.type_propagation`
that controls how type information is propagated.

| PropertyPolicyQos name       | PropertyPolicyQos value                                                                            | Default value   |
|------------------------------|----------------------------------------------------------------------------------------------------|-----------------|
| `"fastdds.type_propagation"` | `"disabled"` <br/><br/>`"enabled"` <br/><br/>`"minimal_bandwidth"` <br/><br/>`"registration_only"` | `"enabled"`     |

The different property values have the following effects on the local [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant):

| Value                 | `TypeObject` registration   | Send type information on EDP   | Receive type information on EDP   | Type lookup service replies   |
|-----------------------|-----------------------------|--------------------------------|-----------------------------------|-------------------------------|
| `"disabled"`          | NO                          | NO                             | IGNORED                           | DISABLED                      |
| `"enabled"`           | COMPLETE and MINIMAL        | COMPLETE and MINIMAL           | PROCESSED                         | ENABLED                       |
| `"minimal_bandwidth"` | MINIMAL only                | MINIMAL only                   | Only MINIMAL PROCESSED            | ENABLED                       |
| `"registration_only"` | COMPLETE and MINIMAL        | COMPLETE and MINIMAL           | IGNORED                           | DISABLED                      |

### C++

```c++
DomainParticipantQos pqos;

pqos.properties().properties().emplace_back(
    "fastdds.type_propagation",
    "disabled");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="type_propagation_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.type_propagation</name>
                            <value>disabled</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="property-type-lookup-request-timeout"></a>

<a id="typelookup-service-request-timeout"></a>

## TypeLookup Service Request Timeout

When a remote type cannot be resolved with the information received on the EDP messages (see [Discovery phases](../discovery/discovery.md#disc-phases)),
the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) requests it through the TypeLookup Service, and the matching of the endpoints using that
type is deferred until the reply arrives.
If the reply never arrives, because it was lost or because the remote [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) stopped answering
without being removed from the network, those endpoints would remain unmatched forever.

Property `fastdds.type_lookup_request_timeout_ms` configures the minimum time a TypeLookup request is kept waiting
for an answer before it is failed.
The endpoints announced by the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) that did not answer are failed, so their matching relies on
type name and topic name (see [Remote Data Types Discovery](../xtypes/discovery_matching.md#xtypes-discovery-matching)), while
the ones announced by other [`DomainParticipants`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) are requested again.

| PropertyPolicyQos name                     | PropertyPolicyQos value                        | Default value   |
|--------------------------------------------|------------------------------------------------|-----------------|
| `"fastdds.type_lookup_request_timeout_ms"` | `uint32_t` <br/><br/>`"0"` disables the check. | `"20000"`       |

#### NOTE
An invalid value of `fastdds.type_lookup_request_timeout_ms` would log an error,
and the default value will be used. Units are in milliseconds.

### C++

```c++
DomainParticipantQos pqos;

pqos.properties().properties().emplace_back(
    "fastdds.type_lookup_request_timeout_ms",
    "5000");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="type_lookup_request_timeout_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.type_lookup_request_timeout_ms</name>
                            <value>5000</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="property-serialize-optional-qos"></a>

<a id="adding-optional-qos-to-discovery-data"></a>

## Adding optional QoS to Discovery data

During [Discovery](../discovery/discovery.md#discovery), [`DomainParticipants`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), [`DataWriters`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) and [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) acknowledge each other.
This is performed in two phases, the Participant Discovery Phase (PDP) and the Endpoint Discovery Phase (EDP).
To do that, the DomainParticipants share information about their DataWriters and DataReaders with each other,
using the communication channels established during the PDP phase.
This information contains all data required to match the endpoints, such as the [`Topic`](../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE), data type, and
certain [QoS Policies](../dds_layer/core/policy/policy.md#dds-layer-core-policy) that might affect matching.
Specific compatibility rules can be found in each QoS section of [QoS Policies](../dds_layer/core/policy/policy.md#dds-layer-core-policy).

However, there are some QoS that are not mandatory for matching, but can be useful to have upon discovery.
Property `fastdds.serialize_optional_qos` allows the user to include these optional QoS during discovery.
This property is configured at the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) level through the policy [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos).
Hence, the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) and all its associated endpoints will send their optional QoS
in the discovery messages.

Optional QoS, like any other QoS, will only be serialized if they have non-default values.
Not receiving information about a QoS in the EDP message means that it has a default value.
Optional QoS will be serialized if the value of the property is set to `TRUE`, `True`, `true` or `1`, any
other value will be considered as not set or `FALSE`.

The following table lists all the optional QoS that can be serialized in the discovery messages:

| PropertyPolicyQos                                                                                                                                                  | Applies to:                                                                                                                                                                                                                  |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`WireProtocolConfigQos`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE)                      | [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).                                                                                                |
| [`ResourceLimitsQosPolicy`](../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE)                | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) and [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE). |
| [`TransportPriorityQosPolicy`](../api_reference/dds_pim/core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE)       | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).                                                                                                                  |
| [`WriterDataLifecycleQosPolicy`](../api_reference/dds_pim/core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicyE) | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).                                                                                                                  |
| [`ReaderDataLifecycleQosPolicy`](../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicyE) | [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).                                                                                                                |
| [`PublishModeQosPolicy`](../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE)                         | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).                                                                                                                  |
| [`RTPSReliableWriterQos`](../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE)                        | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).                                                                                                                  |
| [`RTPSReliableReaderQos`](../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQosE)                       | [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).                                                                                                                |
| [`RTPSEndpointQos`](../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE)                                        | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) and [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE). |
| [`WriterResourceLimitsQos`](../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE)                | [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).                                                                                                                  |
| [`ReaderResourceLimitsQos`](../api_reference/dds_pim/subscriber/readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQosE)                 | [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).                                                                                                                |

### C++

```c++
DomainParticipantQos pqos;

pqos.properties().properties().emplace_back(
    "fastdds.serialize_optional_qos",
    "true"); // true or True or TRUE or 1
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="serialize_optional_qos_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.serialize_optional_qos</name>
                            <value>true</value> <!-- true, True, TRUE, 1 -->
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="properties-sql-filter-expression-limits"></a>

<a id="sql-filter-expression-limits"></a>

## SQL Filter Expression limits

When creating content-filtered topics using the
[default SQL-like filter](../dds_layer/topic/contentFilteredTopic/defaultFilter.md#dds-layer-topic-contentfilteredtopic-default-filter), some restrictions apply to the
SQL expressions that can be used for content-based filtering.
The default limits are set to values that should be enough for most use cases, but they can be customized using the
following properties when a participant is created:

| PropertyPolicyQos name                       | PropertyPolicyQos value                                  | Default value   |
|----------------------------------------------|----------------------------------------------------------|-----------------|
| `"dds.sql.expression.max_expression_length"` | Maximum length of the SQL expression, in characters.     | `16384`         |
| `"dds.sql.expression.max_subexpressions"`    | Maximum number of sub-expressions in the SQL expression. | `256`           |

Creating content-filtered topics with expressions exceeding any of these limits will fail.
Expressions received in discovery messages exceeding any of these limits will be ignored, and filtering will happen
in the reader side.

### C++

```c++
DomainParticipantQos pqos;

pqos.properties().properties().emplace_back(
    "dds.sql.expression.max_expression_length",
    "20000");
pqos.properties().properties().emplace_back(
    "dds.sql.expression.max_subexpressions",
    "10");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="custom_sql_limits_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>dds.sql.expression.max_expression_length</name>
                            <value>20000</value>
                        </property>
                        <property>
                            <name>dds.sql.expression.max_subexpressions</name>
                            <value>10</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```
