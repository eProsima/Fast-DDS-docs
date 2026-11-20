<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="eprosima-extensions"></a>

<a id="id1"></a>

# eProsima Extensions

The eProsima QoS Policies extensions are those that allow changing the values of the RTPS layer configurable settings.

<a id="datasharingqospolicy"></a>

<a id="id2"></a>

## DataSharingQosPolicy

This additional QoS allows configuring the data-sharing delivery communication between a writer and a reader.
Please, see [Data-sharing delivery](../../../transport/datasharing.md#datasharing-delivery) for a description of the data-sharing delivery functionality.

List of QoS Policy data members:

| Data Member                           | Type                                 | Accessor                                                                                                                                                                                  | Default Value   |
|---------------------------------------|--------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| Data-sharing kind                     | [DataSharingKind](#datasharingkind)  | [`kind()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy4kindEv)                                                 | `AUTO`          |
| Shared memory directory               | `string`                             | [`shm_directory()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy13shm_directoryEv)                              | Empty string    |
| Maximum domain number                 | `uint32_t`                           | [`max_domains()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy11max_domainsEv)                                  | 0 (unlimited)   |
| Data-sharing domain IDs               | `vector<uint64_t>`                   | [`domain_ids()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy10domain_idsEv)                                    | Empty           |
| Data-sharing listener thread settings | [ThreadSettings](#threadsettingsqos) | [`data_sharing_listener_thread()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy28data_sharing_listener_threadEv) |                 |
* Data-sharing kind:
  Specifies the behavior of data-sharing delivery.
  See [DataSharingKind](#datasharingkind) for a description of possible values and their effect.
* Shared memory directory:
  The directory that will be used for the memory-mapped files.
  If none is configured, then the system default directory will be used.
* Maximum domain number:
  Establishes the maximum number of data-sharing domain IDs in the local or remote endpoints.
  Domain IDs are exchanged between data-sharing delivery compatible endpoints.
  If this value is lower that the size of the list for any remote endpoint, the matching may fail.
  A value of zero represents unlimited number of IDs.
* Data sharing domain IDs:
  The list of data-sharing domain IDs configured for the current [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
  If no ID is provided, the system will create a unique one for the current machine.
* Data-sharing listener thread settings:
  The [ThreadSettings](#threadsettingsqos) for the data-sharing thread dedicated to listening for incoming traffic.

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="datasharingkind"></a>

<a id="id3"></a>

### DataSharingKind

There are three possible values (see [`DataSharingKind`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKindE)):

* [`OFF`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind3OFFE):
  The data-sharing delivery is disabled.
  No communication will be performed using data-sharing delivery functionality.
* [`ON`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind2ONE):
  The data-sharing delivery is manually enabled.
  An error will occur if the current topic is not [compatible](../../../transport/datasharing.md#datasharing-delivery-constraints)
  with data-sharing delivery.
  Communication with remote entities that share at least one data-sharing domain ID
  will be done using data-sharing delivery functionality.
* [`AUTO`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE):
  data-sharing delivery will be activated if the current topic is
  [compatible](../../../transport/datasharing.md#datasharing-delivery-constraints) with data-sharing,
  and deactivated if not.

<!-- datasharingconfiguration: -->

<a id="data-sharing-configuration-helper-functions"></a>

### Data-sharing configuration helper functions

In order to set the data-sharing delivery configuration, one of the following helper member functions must be used.
There is one for each [DataSharingKind](#datasharingkind) flavor:

| Function                                                                                                                                           | Resulting DataSharingKind                                                                                                   | Shared memory directory   | Data sharing domain IDs   |
|----------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|---------------------------|---------------------------|
| [`automatic()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy9automaticEv) | [`AUTO`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE) | Optional                  | Optional                  |
| [`on()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy2onERKNSt6stringE)   | [`ON`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind2ONE)     | Mandatory                 | Optional                  |
| [`off()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy3offEv)             | [`OFF`](../../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind3OFFE)   | N/A                       | N/A                       |

Instead of defining the data-sharing domain IDs on these helper functions,
you can add them later with the [`add_domain_id()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy13add_domain_idENSt8uint16_tE) function.
Beware that adding a new domain ID counts as modifying the QosPolicy,
so it must be done before the entity is enabled.

<a id="example"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader entities
DataWriterQos writer_qos;

// DataSharing is set to AUTO by default, which means that DataSharing will be used if the topic
// is compatible. If no Shared Memory directory is specified, the default one will be used.

// Configure DataSharing to ON to enable DataSharing and specify the shared memory directory (mandatory)
writer_qos.data_sharing().on("/path/to/shared_memory/directory");
// Alternatively, configure DataSharing to OFF to disable it
writer_qos.data_sharing().off();

// Configure the DataSharing as AUTO with two user-defined IDs (can also be used with ON)
std::vector<uint16_t> ids;
ids.push_back(0x1234);
ids.push_back(0xABCD);
writer_qos.data_sharing().automatic(ids);

// Alternatively, add them individually
writer_qos.data_sharing().add_domain_id(uint16_t(0x1234));
writer_qos.data_sharing().add_domain_id(uint16_t(0xABCD));
// Or you can leave the IDs empty and the system will automatically create a
// unique ID for the current machine

// Set the maximum number of domains to 5. Setting 0 means 'unlimited'
writer_qos.data_sharing().set_max_domains(5);

// [OPTIONAL] ThreadSettings for listening thread
writer_qos.data_sharing().data_sharing_listener_thread(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});

// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <data_writer profile_name="writer_profile_qos_datasharing">
            <qos>
                <data_sharing>
                    <kind>AUTOMATIC</kind>
                    <domain_ids>
                        <domainId>123</domainId>
                        <domainId>098</domainId>
                    </domain_ids>
                </data_sharing>
            </qos>
        </data_writer>

        <data_reader profile_name="reader_profile_qos_datasharing">
            <qos>
                <data_sharing>
                    <kind>AUTOMATIC</kind>
                    <domain_ids>
                        <domainId>123</domainId>
                        <domainId>098</domainId>
                    </domain_ids>
                    <data_sharing_listener_thread>
                        <scheduling_policy>-1</scheduling_policy>
                        <priority>0</priority>
                        <affinity>0</affinity>
                        <stack_size>-1</stack_size>
                    </data_sharing_listener_thread>
                </data_sharing>
            </qos>
        </data_reader>
    </profiles>
<dds>
```

<a id="disablepositiveacksqospolicy"></a>

<a id="id4"></a>

## DisablePositiveACKsQosPolicy

This additional QoS allows reducing network traffic when strict reliable communication is not required and bandwidth is
limited.
It consists in changing the default behavior by which positive acks are sent from readers to writers.
Instead, only negative acks will be sent when a reader is missing a sample, but writers will keep data for an adjustable
time before considering it as acknowledged.
See [`DisablePositiveACKsQosPolicy`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                              | Type         | Default Value                                                                                                                |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|------------------------------------------------------------------------------------------------------------------------------|
| [`enabled`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy7enabledE)   | `bool`       | `false`                                                                                                                      |
| [`duration`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy8durationE) | `Duration_t` | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |
* [`enabled`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy7enabledE):
  Specifies if the QoS is enabled or not. If it is true means that the positive acks are disabled and the
  DataReader only sends negative acks. Otherwise, both positive and negative acks are sent.
* [`duration`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy8durationE):
  State the duration that the DataWriters keep the data before considering it as acknowledged.
  This value does not apply to DataReaders.

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
The [`enabled`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy7enabledE) Data Member cannot be modified on enabled entities.
Thus, this feature must be set up during initialization.
Only the [`duration`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy8durationE) Data Member can be modified at runtime.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#disableacks-compatibilityrule) for further details.

<a id="disableacks-compatibilityrule"></a>

<a id="compatibility-rule"></a>

### Compatibility Rule

To maintain the compatibility between DisablePositiveACKsQosPolicy in DataReaders and DataWriters, the DataReader
cannot have this QoS enabled if the DataWriter has it disabled.

Table with the possible combinations:

| DataWriter enabled value   | DataReader enabled value   | Compatibility   |
|----------------------------|----------------------------|-----------------|
| `true`                     | `true`                     | Yes             |
| `true`                     | `false`                    | Yes             |
| `false`                    | `true`                     | No              |
| `false`                    | `false`                    | Yes             |

<a id="id5"></a>

### Example

### C++

```c++
// This Qos has different API for DataWriter and DataReader entities, because it is accesed
// through the ReliableWriterQos and ReliableReaderQos respectively.
// For further details see RTPSReliableWriterQos & RTPSReliableReaderQos sections.
DataWriterQos writer_qos;
DataReaderQos reader_qos;
// The DisablePositiveACKsQosPolicy is constructed with enabled = false by default
// Change enabled to true
writer_qos.reliable_writer_qos().disable_positive_acks.enabled = true;
reader_qos.reliable_reader_qos().disable_positive_acks.enabled = true;
// The DisablePositiveACKsQosPolicy is constructed with infinite duration by default
// Change the duration to 1 second
writer_qos.reliable_writer_qos().disable_positive_acks.duration = {1, 0};
reader_qos.reliable_reader_qos().disable_positive_acks.duration = {1, 0};

// Exclusively, DataWriters can additionally disable the heartbeat piggyback
writer_qos.reliable_writer_qos().disable_heartbeat_piggyback = true;

// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
reader_ = subscriber_->create_datareader(topic_, reader_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_disable_positive_acks_profile">
    <qos>
        <disablePositiveAcks>
            <enabled>true</enabled>
            <duration>
                <sec>1</sec>
            </duration>
        </disablePositiveAcks>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_disable_positive_acks_profile">
    <qos>
        <disablePositiveAcks>
            <enabled>true</enabled>
        </disablePositiveAcks>
    </qos>
</data_reader>
```

<a id="rtpsreliablereaderqos"></a>

<a id="id6"></a>

## RTPSReliableReaderQos

This RTPS QoS Policy allows the configuration of several RTPS reliable reader’s aspects.
See [`RTPSReliableReaderQos`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                          | Type                                                          |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------|
| [`times`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQos5timesE)                                  | [ReaderTimes](#readertimes)                                   |
| [`disable_positive_acks`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQos21disable_positive_acksE) | [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) |
* [`times`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQos5timesE):
  Defines the duration of the RTPSReader events. See [ReaderTimes](#readertimes) for further details.
* [`disable_positive_acks`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQos21disable_positive_acksE):
  Configures the settings to disable the positive acks.
  See [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) for further details.

#### NOTE
This QoS Policy applies to [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
Only the [`duration`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy8durationE) Data Member of the [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) and the [`times`](../../../api_reference/dds_pim/subscriber/rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQos5timesE)
Data Member can be modified on enabled entities.

<a id="readertimes"></a>

<a id="id7"></a>

### ReaderTimes

This structure defines the times associated with the Reliable Readers’ events.
See [`ReaderTimes`](../../../api_reference/rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimesE).

List of structure members:

| Member Name                                                                                                                                               | Type         | Default Value   |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|-----------------|
| [`initial_acknack_delay`](../../../api_reference/rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimes21initial_acknack_delayE)       | `Duration_t` | 70 ms           |
| [`heartbeat_response_delay`](../../../api_reference/rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimes24heartbeat_response_delayE) | `Duration_t` | 5 ms            |
* [`initial_acknack_delay`](../../../api_reference/rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimes21initial_acknack_delayE):
  Defines the duration of the initial acknack delay.
* [`heartbeat_response_delay`](../../../api_reference/rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimes24heartbeat_response_delayE):
  Establishes the duration of the delay applied when a heartbeat message is received.

<a id="id8"></a>

### Example

### C++

```c++
// This example only applies to DataReader entities
DataReaderQos reader_qos;
// The RTPSReliableReaderQos is constructed with initial_acknack_delay = 70 ms by default
// Change the initialAcknackDelay to 70 nanoseconds
reader_qos.reliable_reader_qos().times.initial_acknack_delay = {0, 70};
// The RTPSReliableReaderQos is constructed with heartbeat_response_delay = 5 ms by default
// Change the heartbeatResponseDelay to 5 nanoseconds
reader_qos.reliable_reader_qos().times.heartbeat_response_delay = {0, 5};
// You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
reader_qos.reliable_reader_qos().disable_positive_acks.enabled = true;
// Use modified QoS in the creation of the DataReader entity
reader_ = subscriber_->create_datareader(topic_, reader_qos);
```

### XML

```xml
<data_reader profile_name="sub_profile_name">
    <times> <!-- readerTimesType -->
        <initial_acknack_delay> <!-- DURATION -->
            <nanosec>70</nanosec>
        </initial_acknack_delay>
        <heartbeat_response_delay> <!-- DURATION -->
            <nanosec>5</nanosec>
        </heartbeat_response_delay>
    </times>
    <!--You can also change the values of DisablePositiveACKsQosPolicy.-->
    <!--See DisablePositiveACKsQosPolicy section for further details-->
</data_reader>
```

<a id="rtpsreliablewriterqos"></a>

<a id="id9"></a>

## RTPSReliableWriterQos

This RTPS QoS Policy allows the configuration of several RTPS reliable writer’s aspects.
See [`RTPSReliableWriterQos`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                     | Type                                                          |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------|
| [`times`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos5timesE)                                              | [WriterTimes](#writertimes)                                   |
| [`disable_positive_acks`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos21disable_positive_acksE)             | [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) |
| [`disable_heartbeat_piggyback`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos27disable_heartbeat_piggybackE) | [DisableHeartbeatPiggyback](#disableheartbeatpiggyback)       |
* [`times`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos5timesE):
  Defines the duration of the RTPSWriter events.
  See [WriterTimes](#writertimes) for further details.
* [`disable_positive_acks`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos21disable_positive_acksE):
  Configures the settings to disable the positive acks.
  See [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) for further details.
* [`disable_heartbeat_piggyback`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos27disable_heartbeat_piggybackE):
  Configures the settings to disable the heartbeat piggyback mechanism.
  See [DisableHeartbeatPiggyback](#disableheartbeatpiggyback) for further details.

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
Only the [`duration`](../../../api_reference/dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicy8durationE) Data Member of the [DisablePositiveACKsQosPolicy](#disablepositiveacksqospolicy) and the [`times`](../../../api_reference/dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQos5timesE)
Data Member can be modified on enabled entities.

<a id="writertimes"></a>

<a id="id10"></a>

### WriterTimes

This structure defines the times associated with the Reliable Writers’ events.

List of structure members:

| Member Name                                                                                                                                               | Type         | Default Value   |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|-----------------|
| [`initial_heartbeat_delay`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes23initial_heartbeat_delayE)   | `Duration_t` | 12ms            |
| [`heartbeat_period`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes16heartbeat_periodE)                 | `Duration_t` | 3s              |
| [`nack_response_delay`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes19nack_response_delayE)           | `Duration_t` | 5ms             |
| [`nack_supression_duration`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes24nack_supression_durationE) | `Duration_t` | 0s              |
* [`initial_heartbeat_delay`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes23initial_heartbeat_delayE):
  Defines duration of the initial heartbeat delay.
* [`heartbeat_period`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes16heartbeat_periodE):
  Specifies the interval between periodic heartbeats.
* [`nack_response_delay`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes19nack_response_delayE):
  Establishes the duration of the delay applied to the response of an ACKNACK message.
* [`nack_supression_duration`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes24nack_supression_durationE):
  The RTPSWriter ignores the nack messages received after sending the data until the
  duration time elapses.

<a id="disableheartbeatpiggyback"></a>

<a id="id11"></a>

### DisableHeartbeatPiggyback

Besides sending heartbeats periodically using the [`heartbeat_period`](../../../api_reference/rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimes16heartbeat_periodE) (see [WriterTimes](#writertimes)), reliable
DataWriters also use a mechanism to append a heartbeat submessage in the same message where data is being delivered to
the DataReaders.
This mechanism acts in specific situations where the reliable communication state must be up to date to maintain
optimal communication:

- When the DataWriter sends as many bytes to the *socket* as the length of the *socket* buffer, a heartbeat
  submessage is appended after the last data.
- When the DataWriter’s history is full, the DataWriter starts to append heartbeat submessages after each data.

This mechanism can be disabled using this policy.

<a id="id12"></a>

### Example

### C++

```c++
// This example only applies to DataWriter entities
DataWriterQos writer_qos;
// The RTPSReliableWriterQos is constructed with initial_heartbeat_delay = 12 ms by default
// Change the initial_heartbeat_delay to 20 nanoseconds
writer_qos.reliable_writer_qos().times.initial_heartbeat_delay = {0, 20};
// The RTPSReliableWriterQos is constructed with heartbeat_period = 3 s by default
// Change the heartbeat_period to 5 seconds
writer_qos.reliable_writer_qos().times.heartbeat_period = {5, 0};
// The RTPSReliableWriterQos is constructed with nack_response_delay = 5 ms by default
// Change the nack_response_delay to 10 nanoseconds
writer_qos.reliable_writer_qos().times.nack_response_delay = {0, 10};
// The RTPSReliableWriterQos is constructed with nack_supression_duration = 0 s by default
// Change the nack_supression_duration to 20 nanoseconds
writer_qos.reliable_writer_qos().times.nack_supression_duration = {0, 20};
// You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
writer_qos.reliable_writer_qos().disable_positive_acks.enabled = true;
// The RTPSReliableWriterQos is constructed with disable_heartbeat_piggyback = false by default
// Disable the heartbeat piggyback mechanism.
writer_qos.reliable_writer_qos().disable_heartbeat_piggyback = true;
// Use modified QoS in the creation of the DataWriter entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="pub_profile_name">
    <times> <!-- writerTimesType -->
        <initial_heartbeat_delay> <!-- DURATION -->
            <nanosec>20</nanosec>
        </initial_heartbeat_delay>
        <heartbeat_period> <!-- DURATION -->
            <sec>5</sec>
        </heartbeat_period>
        <nack_response_delay> <!-- DURATION -->
            <nanosec>10</nanosec>
        </nack_response_delay>
        <nack_supression_duration> <!-- DURATION -->
            <nanosec>20</nanosec>
        </nack_supression_duration>
    </times>

    <!--You can also change the values of DisablePositiveACKsQosPolicy.-->
    <!--See DisablePositiveACKsQosPolicy section for further details-->
    <qos>
        <!--Disable heartbeat piggyback mechanism.-->
        <disable_heartbeat_piggyback>true</disable_heartbeat_piggyback>
    </qos>
</data_writer>
```

<a id="flowcontrollersqos"></a>

<a id="id13"></a>

## FlowControllersQos

This QoS configures the list of flow controllers of a participant, so they can later be used on
its DataWriters.
It is a vector of shared pointers to [`FlowControllerDescriptor`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptorE), which has the following fields:

| Data Member Name                                                                                                                                                             | Type                                                                                                                                                                      | Default Value          |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------|
| [`name`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor4nameE)                                  | `string`                                                                                                                                                                  |                        |
| [`scheduler`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor9schedulerE)                        | [`FlowControllerSchedulerPolicy`](../../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicyE) | FIFO_SCHED_POLICY-api| |
| [`max_bytes_per_period`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor20max_bytes_per_periodE) | `int32_t`                                                                                                                                                                 | 0 (i.e. infinite)      |
| [`period_ms`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor9period_msE)                        | `uint64_t`                                                                                                                                                                | 100                    |
| [`sender_thread`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor13sender_threadE)               | [ThreadSettings](#threadsettingsqos)                                                                                                                                      |                        |

Please refer to [Flow Controllers](../../../use_cases/large_data/large_data.md#flow-controllers) section for more information.

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) entities.
<br />
It cannot be changed on enabled entities.

<a id="participantresourcelimitsqos"></a>

<a id="id14"></a>

## ParticipantResourceLimitsQos

This QoS configures allocation limits and the use of physical memory for internal resources.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                      | Type                                                                      |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| [`locators`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes8locatorsE)              | [RemoteLocatorsAllocationAttributes](#remotelocatorsallocationattributes) |
| [`participants`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes12participantsE)     | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig)         |
| [`readers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7readersE)                | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig)         |
| [`writers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7writersE)                | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig)         |
| [`send_buffers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes12send_buffersE)     | [SendBuffersAllocationAttributes](#sendbuffersallocationattributes)       |
| [`data_limits`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes11data_limitsE)       | [VariableLengthDataLimits](#variablelengthdatalimits)                     |
| [`content_filter`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes14content_filterE) | [ContentFilterProperty::AllocationConfiguration](#contentfilterlimits)    |
* [`locators`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes8locatorsE):
  Defines the limits for collections of remote locators.
* [`participants`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes12participantsE):
  Specifies the allocation behavior and limits for collections dependent on the total number of
  participants.
* [`readers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7readersE):
  Specifies the allocation behavior and limits for collections dependent on the total number of
  readers per participant.
* [`writers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7writersE):
  Specifies the allocation behavior and limits for collections dependent on the total number of
  writers per participant.
* [`send_buffers`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes12send_buffersE):
  Defines the allocation behavior and limits for the send buffer manager.
* [`data_limits`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes11data_limitsE):
  States the limits for variable-length data.
* [`content_filter`](../../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes14content_filterE):
  States the limits for content-filter discovery information.

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) entities.
<br />
It cannot be changed on enabled entities.

<a id="remotelocatorsallocationattributes"></a>

<a id="id15"></a>

### RemoteLocatorsAllocationAttributes

This structure holds the limits for the remote locators’ collections.
See [`RemoteLocatorsAllocationAttributes`](../../../api_reference/rtps/attributes/RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributesE).

List of structure members:

| Member Name                                                                                                                                                                                         | Type     |   Default Value |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|-----------------|
| [`max_unicast_locators`](../../../api_reference/rtps/attributes/RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributes20max_unicast_locatorsE)     | `size_t` |               4 |
| [`max_multicast_locators`](../../../api_reference/rtps/attributes/RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributes22max_multicast_locatorsE) | `size_t` |               1 |
* [`max_unicast_locators`](../../../api_reference/rtps/attributes/RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributes20max_unicast_locatorsE):
  This member controls the maximum number of unicast locators to keep for each discovered
  remote entity.
  It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.
* [`max_multicast_locators`](../../../api_reference/rtps/attributes/RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributes22max_multicast_locatorsE):
  This member controls the maximum number of multicast locators to keep for each discovered
  remote entity.
  The default value is usually enough, as it does not make sense to add more than one multicast locator per entity.

<a id="resourcelimitedcontainerconfig"></a>

<a id="id16"></a>

### ResourceLimitedContainerConfig

This structure holds the limits of a resource limited collection, as well as the allocation configuration, which can be
fixed size or dynamic size.

List of structure members:

| Member Name   | Type     | Default Value                        |
|---------------|----------|--------------------------------------|
| `initial`     | `size_t` | 0                                    |
| `maximum`     | `size_t` | `std::numeric_limits<size_t>::max()` |
| `increment`   | `size_t` | 1 (dynamic size), 0 (fixed size)     |
* `initial`:
  Indicates the number of elements to preallocate in the collection.
* `maximum`:
  Specifies the maximum number of elements allowed in the collection.
* `increment`:
  States the number of items to add when the reserved capacity limit is reached. This member has a
  different default value depending on the allocation configuration chosen.

<a id="sendbuffersallocationattributes"></a>

<a id="id17"></a>

### SendBuffersAllocationAttributes

This structure holds the limits for the allocations of the send buffers.
See [`SendBuffersAllocationAttributes`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributesE).

List of structure members:

| Member Name                                                                                                                                                                                   | Type                                                              | Default Value   |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|-----------------|
| [`preallocated_number`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes19preallocated_numberE)       | `size_t`                                                          | 0               |
| [`dynamic`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes7dynamicE)                                | `bool`                                                            | `false`         |
| [`network_buffers_config`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes22network_buffers_configE) | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig) | (16, inf, 16)   |
* [`preallocated_number`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes19preallocated_numberE):
  This member controls the initial number of send buffers to be allocated.
  The default value will perform an initial guess of the number of buffers required, based on the number of threads
  from which a send operation could be started.
* [`dynamic`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes7dynamicE):
  This member controls how the buffer manager behaves when a send buffer is not available.
  When true, a new buffer will be created. Otherwise, it will wait for a buffer to be returned.
* [`network_buffers_config`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes22network_buffers_configE):
  This attribute defines the allocation behavior and limits of the network buffers contained in each send buffer.
  The default value will preallocate 16 network buffers and dynamically allocate 16 network buffers every time
  that growing the vector is needed.

#### NOTE
[`network_buffers_config`](../../../api_reference/rtps/attributes/SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributes22network_buffers_configE) will also be used to instantiate a vector of
[`SerializedPayload_t`](../../../api_reference/rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) that contains the metadata necessary to avoid payload copies during the creation of the
RTPS message.

<a id="variablelengthdatalimits"></a>

<a id="id18"></a>

### VariableLengthDataLimits

This structure holds the limits for variable-length data.
See [`VariableLengthDataLimits`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimitsE).

List of structure members:

| Member Name                                                                                                                                                     | Type     |   Default Value |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|-----------------|
| [`max_properties`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_propertiesE) | `size_t` |               0 |
| [`max_user_data`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits13max_user_dataE)   | `size_t` |               0 |
| [`max_partitions`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_partitionsE) | `size_t` |               0 |
* [`max_properties`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_propertiesE):
  Defines the maximum size, in octets, of the properties data in the local or remote participant.
* [`max_user_data`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits13max_user_dataE):
  Establishes the maximum size, in octets, of the user data in the local or remote participant.
* [`max_partitions`](../../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_partitionsE):
  States the maximum size, in octets, of the partitions data in the local or remote participant.

<a id="contentfilterlimits"></a>

<a id="contentfilterproperty-allocationconfiguration"></a>

### ContentFilterProperty::AllocationConfiguration

This structure holds the limits for content-filter related discovery information.
See [`ContentFilterProperty::AllocationConfiguration`](../../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfigurationE).

List of structure members:

| Member Name                                                                                                                                                                                            | Type                                                              | Default Value   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|-----------------|
| [`expression_initial_size`](../../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration23expression_initial_sizeE) | `size_t`                                                          | 0               |
| [`expression_parameters`](../../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration21expression_parametersE)     | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig) | `{0, 100, 1}`   |
* [`expression_initial_size`](../../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration23expression_initial_sizeE):
  Preallocated size of the filter expression.
* [`expression_parameters`](../../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration21expression_parametersE):
  Allocation configuration for the list of expression parameters.

<a id="id19"></a>

### Example

### C++

```c++
// This example only applies to DomainParticipant entities
DomainParticipantQos participant_qos;
// Set the maximum size of participant resource limits collection to 3 and it allocation configuration to fixed size
participant_qos.allocation().participants =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(
    3u);
// Set the maximum size of reader's resource limits collection to 2 and its allocation configuration to fixed size
participant_qos.allocation().readers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
// Set the maximum size of writer's resource limits collection to 1 and its allocation configuration to fixed size
participant_qos.allocation().writers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
// Set the maximum size of the partition data to 256
participant_qos.allocation().data_limits.max_partitions = 256u;
// Set the maximum size of the user data to 256
participant_qos.allocation().data_limits.max_user_data = 256u;
// Set the maximum size of the properties data to 512
participant_qos.allocation().data_limits.max_properties = 512u;
// Set the preallocated filter expression size to 512
participant_qos.allocation().content_filter.expression_initial_size = 512u;
// Set the maximum number of expression parameters to 4 and its allocation configuration to fixed size
participant_qos.allocation().content_filter.expression_parameters =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
// Use modified QoS in the creation of the corresponding DomainParticipant
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

```xml
<participant profile_name="participant_alloc_qos_example">
    <rtps>
        <allocation>
            <!-- We know we have 3 participants on the domain -->
            <total_participants>
                <initial>3</initial>
                <maximum>3</maximum>
                <increment>0</increment>
            </total_participants>
            <!-- We know we have at most 2 readers on each participant -->
            <total_readers>
                <initial>2</initial>
                <maximum>2</maximum>
                <increment>0</increment>
            </total_readers>
            <!-- We know we have at most 1 writer on each participant -->
            <total_writers>
                <initial>1</initial>
                <maximum>1</maximum>
                <increment>0</increment>
            </total_writers>
            <max_partitions>256</max_partitions>
            <max_user_data>256</max_user_data>
            <max_properties>512</max_properties>

            <!-- content_filter cannot be configured using XML (yet) -->
        </allocation>
    </rtps>
</participant>

```

<a id="propertypolicyqos"></a>

<a id="id20"></a>

## PropertyPolicyQos

This additional QoS Policy ([`PropertyPolicyQos`](../../../api_reference/dds_pim/core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE)) stores name/value pairs that can be used to configure certain
DDS settings that cannot be configured directly using an standard QoS Policy.
For the complete list of settings that can be configured with this QoS Policy, please refer to [PropertyPolicyQos Options](../../../property_policies/property_policies.md#property-policies).

This QoS also allows to add custom user properties that could be sent to the external entities.
This could be done by setting as `true` the `propagate` value of the Property.

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="id21"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DomainParticipant and DataReader entities
DataWriterQos writer_qos;
// Add new property for the Auth:PKI-DH plugin
writer_qos.properties().properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");
// Add new property for the Access:Permissions plugin
writer_qos.properties().properties().emplace_back(eprosima::fastdds::rtps::Property("dds.sec.access.plugin",
        "builtin.Access-Permissions"));

// Add new user custom property to send to external Participants
writer_qos.properties().properties().emplace_back("Custom Property Name", "Custom value", true);
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<participant profile_name="secure_participant_conf_all_plugin_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Activate Auth:PKI-DH plugin -->
                <property>
                    <name>dds.sec.auth.plugin</name>
                    <value>builtin.PKI-DH</value>
                </property>

                <!-- Activate Access:Permissions plugin -->
                <property>
                    <name>dds.sec.access.plugin</name>
                    <value>builtin.Access-Permissions</value>
                </property>

                <!-- User Custom Property to send externally -->
                <property>
                    <name>Custom Property Name</name>
                    <value>Custom value</value>
                    <propagate>true</propagate>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="publishmodeqospolicy"></a>

<a id="id22"></a>

## PublishModeQosPolicy

This QoS Policy configures how the [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) sends the data.
See [`PublishModeQosPolicy`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE).

It also configures the name of the flow controller to use when asynchronous publishing is used.
It should be the name of a flow controller registered on the creation of the DomainParticipant.
See [FlowControllersQos](#flowcontrollersqos).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                       | Type                                                  | Default Value                                                                                                                                                                          |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicy4kindE)                                  | [PublishModeQosPolicyKind](#publishmodeqospolicykind) | [`SYNCHRONOUS_PUBLISH_MODE`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKind24SYNCHRONOUS_PUBLISH_MODEE) |
| [`flow_controller_name`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicy20flow_controller_nameE) | `string`                                              | [`FASTDDS_FLOW_CONTROLLER_DEFAULT`](../../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps31FASTDDS_FLOW_CONTROLLER_DEFAULTE)               |

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

<a id="publishmodeqospolicykind"></a>

<a id="id23"></a>

### PublishModeQosPolicyKind

There are two possible values (see [`PublishModeQosPolicyKind`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKindE)):

* [`SYNCHRONOUS_PUBLISH_MODE`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKind24SYNCHRONOUS_PUBLISH_MODEE): The data is sent in the context of the user thread that calls the write operation.
* [`ASYNCHRONOUS_PUBLISH_MODE`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKind25ASYNCHRONOUS_PUBLISH_MODEE): An internal thread takes the responsibility of sending the data asynchronously.
  The write operation returns before the data is actually sent.

Also, the [`flow_controller_name`](../../../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicy20flow_controller_nameE) has to be set to the name of a valid [Flow Controllers](../../../use_cases/large_data/large_data.md#flow-controllers)
descriptor name.

<a id="id24"></a>

### Example

### C++

```c++
// This example only applies to DataWriter entities
DataWriterQos writer_qos;
// The PublishModeQosPolicy is constructed with kind = SYNCHRONOUS by default
// Change the kind to ASYNCHRONOUS
writer_qos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
// Use modified QoS in the creation of the DataWriter entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_profile_qos_publishmode">
    <qos>
        <publishMode>
            <kind>ASYNCHRONOUS</kind>
            <flow_controller_name>example_flow_controller</flow_controller_name>
        </publishMode>
    </qos>
</data_writer>
```

<a id="readerresourcelimitsqos"></a>

<a id="id25"></a>

## ReaderResourceLimitsQos

This QoS Policy states the limits for the matched [DataWriters](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)’ resource limited collections based on the maximum
number of DataWriters that are going to match with the [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
See [`ReaderResourceLimitsQos`](../../../api_reference/dds_pim/subscriber/readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                            | Type                                                              |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| [`matched_publisher_allocation`](../../../api_reference/dds_pim/subscriber/readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQos28matched_publisher_allocationE) | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig) |

#### NOTE
This QoS Policy applies to [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="id26"></a>

### Example

### C++

```c++
// This example only applies to DataReader entities
DataReaderQos reader_qos;
// Set the maximum size for writer matched resource limits collection to 1 and its allocation configuration to fixed size
reader_qos.reader_resource_limits().matched_publisher_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
// Set the maximum size for sample info resource limits to 22 and its allocation configuration to fixed size
reader_qos.reader_resource_limits().sample_infos_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(22u);
// Set the maximum size for writer matched resource limits collection to 3 and its allocation configuration to fixed size
reader_qos.reader_resource_limits().outstanding_reads_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
reader_qos.reader_resource_limits().max_samples_per_read = 42;
// Use modified QoS in the creation of the DataReader entity
reader_ = subscriber_->create_datareader(topic_, reader_qos);
```

### XML

```xml
<data_reader profile_name="alloc_qos_example_sub">
    <!-- we know we will only have one matching publisher -->
    <matchedPublishersAllocation>
        <initial>1</initial>
        <maximum>1</maximum>
        <increment>0</increment>
    </matchedPublishersAllocation>
</data_reader>
```

<a id="writerresourcelimitsqos"></a>

<a id="id27"></a>

## WriterResourceLimitsQos

This QoS Policy states the limits for the matched [DataReaders](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader)’ resource limited collections based on the maximum
number of DataReaders that are going to match with the [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter).
See [`WriterResourceLimitsQos`](../../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                               | Type                                                              |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| [`matched_subscriber_allocation`](../../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos29matched_subscriber_allocationE) | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig) |
| [`reader_filters_allocation`](../../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos25reader_filters_allocationE)         | [ResourceLimitedContainerConfig](#resourcelimitedcontainerconfig) |

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

<a id="id28"></a>

### Example

### C++

```c++
// This example only applies to DataWriter entities
DataWriterQos writer_qos;
// Set the maximum size for reader matched resource limits collection to 3 and its allocation configuration to fixed size
writer_qos.writer_resource_limits().matched_subscriber_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
// Set the maximum number of writer side content filters to 1 and its allocation configuration to fixed size
writer_qos.writer_resource_limits().reader_filters_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
// Use modified QoS in the creation of the DataWriter entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="alloc_qos_example_pub_for_topic_1">
    <!-- we know we will have three matching subscribers -->
    <matchedSubscribersAllocation>
        <initial>3</initial>
        <maximum>3</maximum>
        <increment>0</increment>
    </matchedSubscribersAllocation>

    <!-- reader_filters_allocation cannot be configured using XML (yet) -->
</data_writer>
```

<a id="rtpsendpointqos"></a>

<a id="id29"></a>

## RTPSEndpointQos

This QoS Policy configures the aspects of an RTPS endpoint, such as the list of locators, the identifiers, and the
history memory policy.
See [`RTPSEndpointQos`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                             | Type                                                                                                                              | Default Value                                                                                                                                                                                            |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`unicast_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos20unicast_locator_listE)                 | [`LocatorList`](../../../api_reference/rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE)            | Empty List                                                                                                                                                                                               |
| [`multicast_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos22multicast_locator_listE)             | [`LocatorList`](../../../api_reference/rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE)            | Empty List                                                                                                                                                                                               |
| [`remote_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos19remote_locator_listE)                   | [`LocatorList`](../../../api_reference/rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE)            | Empty List                                                                                                                                                                                               |
| [`external_unicast_locators`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos25external_unicast_locatorsE)       | [`ExternalLocators`](../../../api_reference/rtps/attributes/ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) | Empty                                                                                                                                                                                                    |
| [`ignore_non_matching_locators`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos28ignore_non_matching_locatorsE) | `bool`                                                                                                                            | false                                                                                                                                                                                                    |
| [`user_defined_id`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos15user_defined_idE)                           | `int16_t`                                                                                                                         | -1                                                                                                                                                                                                       |
| [`entity_id`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos9entity_idE)                                        | `int16_t`                                                                                                                         | -1                                                                                                                                                                                                       |
| [`history_memory_policy`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos21history_memory_policyE)               | [MemoryManagementPolicy](#memorymanagementpolicy)                                                                                 | [`PREALLOCATED_WITH_REALLOC_MEMORY_MODE`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE) |
* [`unicast_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos20unicast_locator_listE):
  Defines the list of unicast locators associated to the DDS Entity.
  DataReaders and DataWriters inherit the list of unicast locators set in the DomainParticipant, but it can be
  changed by means of this QoS.
* [`multicast_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos22multicast_locator_listE):
  Stores the list of multicast locators associated to the DDS Entity.
  By default, DataReaders and DataWriters do not use any multicast locator, but it can be changed by means of this QoS.
* [`remote_locator_list`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos19remote_locator_listE):
  States the list of remote locators associated to the DDS Entity.
* [`external_unicast_locators`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos25external_unicast_locatorsE):
  Defines the [External Locators](../../../transport/announced_locators.md#external-locators) to announce for the communication with this DDS Entity.
* [`ignore_non_matching_locators`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos28ignore_non_matching_locatorsE):
  Defines whether to ignore locators received on announcements from other DDS entities when they don’t match with any
  of the locators announced by this DDS Entity.
* [`user_defined_id`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos15user_defined_idE):
  Establishes the unique identifier used for StaticEndpointDiscovery.
* [`entity_id`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos9entity_idE):
  The user can specify the identifier for the endpoint.
* [`history_memory_policy`](../../../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos21history_memory_policyE):
  Indicates the way the memory is managed in terms of dealing with the CacheChanges.

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="memorymanagementpolicy"></a>

<a id="id30"></a>

### MemoryManagementPolicy

There are four possible values (see [`MemoryManagementPolicy`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicyE)):

* [`PREALLOCATED_MEMORY_MODE`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE):
  This option sets the size to the maximum of each data type. It produces the largest
  memory footprint but the smallest allocation count.
* [`PREALLOCATED_WITH_REALLOC_MEMORY_MODE`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE):
  This option set the size to the default for each data type and it requires
  reallocation when a bigger message arrives. It produces a lower memory footprint at the expense of increasing the
  allocation count.
* [`DYNAMIC_RESERVE_MEMORY_MODE`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy27DYNAMIC_RESERVE_MEMORY_MODEE):
  This option allocates the size dynamically at the time of message arrival. It
  produces the least memory footprint but the highest allocation count.
* [`DYNAMIC_REUSABLE_MEMORY_MODE`](../../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy28DYNAMIC_REUSABLE_MEMORY_MODEE):
  This option is similar to `DYNAMIC_RESERVE_MEMORY_MODE`, but the allocated memory
  is reused for future messages.

<a id="id31"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader entities
DataWriterQos writer_qos;
// Add new unicast locator with port 7800
eprosima::fastdds::rtps::Locator_t new_unicast_locator;
new_unicast_locator.port = 7800;
writer_qos.endpoint().unicast_locator_list.push_back(new_unicast_locator);
// Add new multicast locator with IP 239.255.0.4 and port 7900
eprosima::fastdds::rtps::Locator_t new_multicast_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
new_multicast_locator.port = 7900;
writer_qos.endpoint().multicast_locator_list.push_back(new_multicast_locator);
// Add an external locator with IP 100.100.100.10, port 12345, mask 24, externality 1, and cost 0
eprosima::fastdds::rtps::LocatorWithMask external_locator;
external_locator.kind = LOCATOR_KIND_UDPv4;
external_locator.port = 12345;
external_locator.mask(24);
writer_qos.endpoint().external_unicast_locators[1][0].push_back(external_locator);
// Drop non matching locators
writer_qos.endpoint().ignore_non_matching_locators = true;
// Set 3 as user defined id
writer_qos.endpoint().user_defined_id = 3;
// Set 4 as entity id
writer_qos.endpoint().entity_id = 4;
// The RTPSWriterQos is constructed with history_memory_policy = PREALLOCATED by default
// Change the history_memory_policy to DYNAMIC_RESERVE
writer_qos.endpoint().history_memory_policy = eprosima::fastdds::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_unicast_locators_profile">
    <userDefinedID>3</userDefinedID>
    <entityID>2</entityID> <!-- Int16 -->
    <unicastLocatorList>
        <locator>
            <udpv4>
                <port>7800</port>
            </udpv4>
        </locator>
    </unicastLocatorList>
    <multicastLocatorList>
        <locator>
            <udpv4>
                <address>239.255.0.4</address>
                <port>7900</port>
            </udpv4>
        </locator>
    </multicastLocatorList>
    <external_unicast_locators>
        <udpv4 externality="1" cost="0" mask="24">
            <address>100.100.100.10</address>
            <port>12345</port>
        </udpv4>
    </external_unicast_locators>
    <ignore_non_matching_locators>true</ignore_non_matching_locators>
    <!-- The history memory policy is changed to DYNAMIC_RESERVE -->
    <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
</data_writer>

<data_reader profile_name="reader_xml_conf_unicast_locators_profile">
    <userDefinedID>5</userDefinedID>
    <entityID>4</entityID> <!-- Int16 -->
    <unicastLocatorList>
        <locator>
            <udpv4>
                <port>7800</port>
            </udpv4>
        </locator>
    </unicastLocatorList>
    <multicastLocatorList>
        <locator>
            <udpv4>
                <address>239.255.0.4</address>
                <port>7900</port>
            </udpv4>
        </locator>
    </multicastLocatorList>
    <external_unicast_locators>
        <udpv4 externality="1" cost="0" mask="24">
            <address>100.100.100.10</address>
            <port>12345</port>
        </udpv4>
    </external_unicast_locators>
    <ignore_non_matching_locators>true</ignore_non_matching_locators>
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
</data_reader>
```

<a id="threadsettingsqos"></a>

<a id="threadsettings"></a>

## ThreadSettings

This structure is part of other QoS policies, and allows controlling some OS settings for the threads created.
The default values will leave the default OS settings on the created threads.
Changing these values may require special permissions.

| Data Member Name                                                                                                                                  | Type       | Default Value   |
|---------------------------------------------------------------------------------------------------------------------------------------------------|------------|-----------------|
| [`scheduling_policy`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings17scheduling_policyE) | `int32_t`  | -1              |
| [`priority`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8priorityE)                    | `int32_t`  | -2^31           |
| [`affinity`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8affinityE)                    | `uint32_t` | 0               |
| [`stack_size`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings10stack_sizeE)               | `int32_t`  | -1              |
* [`scheduling_policy`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings17scheduling_policyE): Configures the scheduling policy used for the thread.
  This value is platform specific and it is used as-is to configure the specific platform thread.
  It is ignored on Windows platforms.
* [`priority`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8priorityE): Configures the thread’s priority.
  This value is platform specific and it is used as-is to configure the specific platform thread.
* [`affinity`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings8affinityE): On some systems (Windows, Linux), this is a bit mask for setting
  the threads affinity to each core individually.
  On MacOS, this sets the affinity tag for the thread, and the OS tries to share the L2 cache
  between threads with the same affinity.
  This value is platform specific and it is used as-is to configure the specific platform thread.
* [`stack_size`](../../../api_reference/rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettings10stack_sizeE): Configures the thread’s stack size in bytes.
  This value is platform specific and it is used as-is to configure the specific platform thread.

The [ThreadSettings](#threadsettingsqos) for reception threads in port-based transport descriptors can also be configured
by associating each thread-specific configuration with its corresponding port number.

<a id="id32"></a>

### Example

The following example illustrate a thread settings configuration:

### C++

```c++
// This example uses a specific thread of the DomainParticipantQos, but it can also be applied to most of
// the threads used by FastDDS in DomainParticipantFactoryQos, DomainParticipantQos or DataSharingQosPolicy.
// Each thread has its own getter and setter function. See Fast DDS documentation for further details.
DomainParticipantQos participant_qos;
participant_qos.timed_events_thread().scheduling_policy = 2;
participant_qos.timed_events_thread().priority = 10;
participant_qos.timed_events_thread().affinity = 4;
participant_qos.timed_events_thread().stack_size = 2000;

// Use modified QoS in the creation of the corresponding entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

```xml
<thread_settings>
    <scheduling_policy>2</scheduling_policy>
    <priority>10</priority>
    <affinity>4</affinity>
    <stack_size>2000</stack_size>
</thread_settings>
```

The subsequent example depicts a reception threads settings configuration:

### C++

```c++
// Implement a Custom class, derived from PortBasedTransportDescriptor, to set the reception threads configuration
CustomPortBasedTransportDescriptor descriptor;
PortBasedTransportDescriptor::ReceptionThreadsConfigMap reception_threads_config;
reception_threads_config[20000].scheduling_policy = 1;
reception_threads_config[20000].priority = 30;
reception_threads_config[20000].affinity = 2;
reception_threads_config[20000].stack_size = 1024;
reception_threads_config[20001].scheduling_policy = 2;
reception_threads_config[20001].priority = 10;
reception_threads_config[20001].affinity = 6;
reception_threads_config[20001].stack_size = 4096;

// Use setter to set the reception threads configuration
descriptor.reception_threads(reception_threads_config);
```

### XML

```xml
<transport_descriptors>
    <transport_descriptor>
        <transport_id>test_transport</transport_id>
        <type>UDPv4</type>
        <reception_threads>
            <reception_thread port="20000">
                <scheduling_policy>1</scheduling_policy>
                <priority>30</priority>
                <affinity>2</affinity>
                <stack_size>1024</stack_size>
            </reception_thread>
            <reception_thread port="20001">
                <scheduling_policy>2</scheduling_policy>
                <priority>10</priority>
                <affinity>6</affinity>
                <stack_size>4096</stack_size>
            </reception_thread>
        </reception_threads>
    </transport_descriptor>
</transport_descriptors>
```

<a id="transportconfigqos"></a>

<a id="id33"></a>

## TransportConfigQos

This QoS Policy allows the configuration of the transport layer settings.
See [`TransportConfigQos`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQosE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                             | Type                                                         | Default Value   |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|-----------------|
| [`user_transports`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos15user_transportsE)                     | `std::vector<std::shared_ptr<TransportDescriptorInterface>>` | Empty vector    |
| [`use_builtin_transports`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos22use_builtin_transportsE)       | `bool`                                                       | `true`          |
| [`send_socket_buffer_size`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos23send_socket_buffer_sizeE)     | `uint32_t`                                                   | 0               |
| [`listen_socket_buffer_size`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos25listen_socket_buffer_sizeE) | `uint32_t`                                                   | 0               |
| `builtin_transports_reception_threads()`                                                                                                                                     | [ThreadSettings](#threadsettingsqos)                         |                 |
| [`max_msg_size_no_frag`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos20max_msg_size_no_fragE)           | `uint32_t`                                                   | 0               |
| [`netmask_filter`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos14netmask_filterE)                       | `fastdds::rtps::NetmaskFilterKind`                           | `AUTO`          |
* [`user_transports`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos15user_transportsE):
  This data member defines the list of transports to use alongside or in place of builtins.
* [`use_builtin_transports`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos22use_builtin_transportsE):
  It controls whether the built-in transport layer is enabled or disabled. If it is set to
  false, the default UDPv4 implementation is disabled.
* [`send_socket_buffer_size`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos23send_socket_buffer_sizeE):
  By default, Fast DDS creates socket buffers using the system default size. This data
  member allows to change the send socket buffer size used to send data.
  **[Pro]** On TCP transports, if it is not set or set to 0, the operating system auto-tunes the buffer.
* [`listen_socket_buffer_size`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos25listen_socket_buffer_sizeE):
  The listen socket buffer size is also created with the system default size, but it can
  be changed using this data member.
  **[Pro]** On TCP transports, if it is not set or set to 0, the operating system auto-tunes the buffer.
* `builtin_transports_reception_threads()`:
  The [ThreadSettings](#threadsettingsqos) for the reception threads of the builtin transports.
* [`max_msg_size_no_frag`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos20max_msg_size_no_fragE):
  Maximum message size used to avoid fragmentation.
  Useful when the configured transports allow for big datagrams to be sent (i.e. SHM or TCP).
* [`netmask_filter`](../../../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos14netmask_filterE):
  [Network filter](../../../transport/interfaces.md#netmask-filtering) configuration.

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) entities.
<br />
It cannot be changed on enabled entities.

<a id="transportdescriptorinterface"></a>

<a id="id34"></a>

### TransportDescriptorInterface

This structure is the base for the data type used to define transport configuration.

List of structure members:

| Member Name                                                                                                                                                                                                  | Type       |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|
| [`maxMessageSize`](../../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface14maxMessageSizeE)             | `uint32_t` |
| [`maxInitialPeersRange`](../../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface20maxInitialPeersRangeE) | `uint32_t` |
* [`maxMessageSize`](../../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface14maxMessageSizeE):
  This member sets the maximum size in bytes of the transport’s message buffer.
* [`maxInitialPeersRange`](../../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface20maxInitialPeersRangeE):
  This member states the maximum number of guessed initial peers to try to connect.

<a id="id35"></a>

### Example

### C++

```c++
// This example only applies to DomainParticipant entities
DomainParticipantQos participant_qos;
// Add new transport to the list of user transports
std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
        std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
descriptor->sendBufferSize = 9126;
descriptor->receiveBufferSize = 9126;
participant_qos.transport().user_transports.push_back(descriptor);
// Set use_builtin_transports to false
participant_qos.transport().use_builtin_transports = false;
// [OPTIONAL] Set ThreadSettings for the builtin transports reception threads
participant_qos.transport().builtin_transports_reception_threads_ =
        eprosima::fastdds::rtps::ThreadSettings{2, 2, 2, 2};
// Set max_msg_size_no_frag to a value > 65500 KB
participant_qos.transport().max_msg_size_no_frag = 70000;
// Configure netmask filter
participant_qos.transport().netmask_filter = eprosima::fastdds::rtps::NetmaskFilterKind::ON;
// Use modified QoS in the creation of the DomainParticipant entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

```xml
<transport_descriptors>
    <transport_descriptor>
        <transport_id>my_transport</transport_id>
        <type>UDPv4</type>
        <sendBufferSize>9216</sendBufferSize>
        <receiveBufferSize>9216</receiveBufferSize>
    </transport_descriptor>
</transport_descriptors>

<participant profile_name="my_transport">
    <rtps>
        <userTransports>
            <transport_id>my_transport</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
        <netmask_filter>ON</netmask_filter>
    </rtps>
</participant>
```

#### NOTE
[TransportConfigQos](#transportconfigqos) can also be configured modifying the builtin
transports configuration by selecting one of the available builtin transports options.
See [Managing the Builtin Transports](../../../rtps_layer/rtps_layer.md#rtps-layer-builtin-transports) or [`setup_transports()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos16setup_transportsEN4rtps17BuiltinTransportsERKN4rtps24BuiltinTransportsOptionsE).

<a id="wireprotocolconfigqos"></a>

<a id="id36"></a>

## WireProtocolConfigQos

This QoS Policy allows the configuration of the wire protocol.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                                   | Type                                                                                                                                   | Default Value   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`prefix`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos6prefixE)                                                        | [`GuidPrefix_t`](../../../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE)         | 0               |
| [`participant_id`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos14participant_idE)                                       | `int32_t`                                                                                                                              | -1              |
| [`builtin`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE)                                                      | [`BuiltinAttributes`](../../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributesE)   |                 |
| [`port`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos4portE)                                                            | [`PortParameters`](../../../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParametersE) |                 |
| [`default_unicast_locator_list`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28default_unicast_locator_listE)           | [`LocatorList`](../../../api_reference/rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE)                 | Empty List      |
| [`default_multicast_locator_list`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos30default_multicast_locator_listE)       | [`LocatorList`](../../../api_reference/rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE)                 | Empty List      |
| [`default_external_unicast_locators`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos33default_external_unicast_locatorsE) | [`ExternalLocators`](../../../api_reference/rtps/attributes/ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE)      | Empty           |
| [`ignore_non_matching_locators`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28ignore_non_matching_locatorsE)           | `bool`                                                                                                                                 | false           |

List of QoS Policy methods:

| Method name                                                                                                                                                      | Input parameters   | Return type    | Description                                |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------|----------------|--------------------------------------------|
| [`easy_mode()`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos9easy_modeERKNSt6stringE) | `std::string ip`   | `ReturnCode_t` | Setter for `easy_mode_ip_` private member. |
| [`easy_mode()`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos9easy_modeERKNSt6stringE) | Empty              | `std::string`  | Getter for `easy_mode_ip_` private member. |
* [`prefix`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos6prefixE):
  This data member allows the user to set manually the GUID prefix.
* [`participant_id`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos14participant_idE):
  It sets the participant identifier. By default, it will be automatically generated by the Domain.
* [`builtin`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE):
  This data member allows the configuration of the built-in parameters.
* [`port`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos4portE):
  This data member allows the configuration of the port parameters and gains related to the RTPS protocol
  ([Well Known Ports](../../../transport/listening_locators.md#listening-locators-defaultports)).
* [`default_unicast_locator_list`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28default_unicast_locator_listE):
  States the default list of unicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without unicast locators. This list should include at
  least one locator.
* [`default_multicast_locator_list`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos30default_multicast_locator_listE):
  Stores the default list of multicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without multicast locators. This list is usually left
  empty.
* [`default_external_unicast_locators`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos33default_external_unicast_locatorsE):
  Defines the [External Locators](../../../transport/announced_locators.md#external-locators) to be used for any endpoint defined inside the participant in the case that
  it was defined without unicast locators.
* [`ignore_non_matching_locators`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28ignore_non_matching_locatorsE):
  Defines whether to ignore locators received on announcements from other DDS participants when they don’t match with
  any of the locators announced by this DDS participant.
* [`easy_mode()`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos9easy_modeERKNSt6stringE):
  This method allows the user to set or get the `easy_mode_ip_` private member, which stores the IP address to be used
  in the [Discovery Server Easy Mode](https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html) configuration.
  When [`easy_mode()`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos9easy_modeERKNSt6stringE) setter is called, the input string parameter is validated to make sure
  it is a valid IPv4 address.
  If not, `easy_mode_ip_` will be set to an empty string, indicating that ROS 2 Easy Mode is disabled.

#### NOTE
If the Easy Mode Discovery Server IP is configured simultaneously using
the [ROS2_EASY_MODE](../../../env_vars/env_vars.md#env-vars-easy-mode) environment variable and manually via XML or with [`WireProtocolConfigQos`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE),
the manually configured value takes precedence, thereby ignoring the environment variable value.

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) entities.

#### IMPORTANT
The only mutable field on enabled entities is [`m_DiscoveryServers`](../../../api_reference/rtps/attributes/DiscoverySettings.md#_CPPv4N8eprosima7fastdds4rtps17DiscoverySettings18m_DiscoveryServersE), which is contained in
[`discovery_config`](../../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes16discovery_configE) within [`builtin`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) (see
[Modifying remote servers list at run time](../../../discovery/discovery_server.md#ds-modify-server-list)).

#### NOTE
Deployments where multiple DomainParticipants are created within the same host can lead into denying
available ports for them if the amount of DomainParticipants created reaches the value of
[`BuiltinAttributes::mutation_tries`](../../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes14mutation_triesE)
(100 by default).
When that happens, the DomainParticipants will not be able to create the listening ports (this is notified
with a log warning) and will be created without unicast locators configured.

<a id="wireprotocolconfigqos-example"></a>

<a id="id37"></a>

### Example

### C++

```c++
// This example only applies to DomainParticipant entities
DomainParticipantQos participant_qos;
// Set the guid prefix
std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> participant_qos.wire_protocol().prefix;
// Manually set the participantId
participant_qos.wire_protocol().participant_id = 11;
// Configure Builtin Attributes
participant_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        eprosima::fastdds::rtps::DiscoveryProtocol::SERVER;
// Add locator to unicast list
eprosima::fastdds::rtps::Locator_t server_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
server_locator.port = 56542;
participant_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);
// Add a metatraffic external locator with IP 100.100.100.10, port 34567, mask 24, externality 1, and cost 0
eprosima::fastdds::rtps::LocatorWithMask meta_external_locator;
meta_external_locator.kind = LOCATOR_KIND_UDPv4;
meta_external_locator.port = 34567;
meta_external_locator.mask(24);
participant_qos.wire_protocol().builtin.metatraffic_external_unicast_locators[1][0].push_back(
    meta_external_locator);
// Add locator to default unicast locator list
eprosima::fastdds::rtps::Locator_t unicast_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(unicast_locator, 192, 168, 1, 41);
unicast_locator.port = 7400;
participant_qos.wire_protocol().default_unicast_locator_list.push_back(unicast_locator);
// Add locator to default multicast locator list
eprosima::fastdds::rtps::Locator_t multicast_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(multicast_locator, 192, 168, 1, 41);
multicast_locator.port = 7400;
participant_qos.wire_protocol().default_multicast_locator_list.push_back(multicast_locator);
// Add a default external locator with IP 100.100.100.10, port 23456, mask 24, externality 1, and cost 0
eprosima::fastdds::rtps::LocatorWithMask external_locator;
external_locator.kind = LOCATOR_KIND_UDPv4;
external_locator.port = 23456;
external_locator.mask(24);
participant_qos.wire_protocol().default_external_unicast_locators[1][0].push_back(external_locator);
// Drop non matching locators
participant_qos.wire_protocol().ignore_non_matching_locators = true;
// Configure the ROS 2 Easy Mode master Discovery Server IP
participant_qos.wire_protocol().easy_mode("127.0.0.1");
// Increase mutation tries
participant_qos.wire_protocol().builtin.mutation_tries = 300u;
// Use a specific flow controller to control the builtin writers
participant_qos.wire_protocol().builtin.flow_controller_name = "AlreadyExistingFlowController";
// Use modified QoS in the creation of the DomainParticipant entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

```xml
<participant profile_name="UDP SERVER WP" is_default_profile="true">
    <rtps>
        <prefix>72.61.73.70.66.61.72.6d.74.65.73.74</prefix>
        <participantID>11</participantID>
        <easy_mode_ip>127.0.0.1</easy_mode_ip>
        <builtin>
            <discovery_config>
                <discoveryProtocol>SERVER</discoveryProtocol>
            </discovery_config>
            <metatrafficUnicastLocatorList>
                <locator>
                    <udpv4>
                        <address>192.168.10.57</address>
                        <port>56542</port>
                    </udpv4>
                </locator>
            </metatrafficUnicastLocatorList>
            <metatraffic_external_unicast_locators>
                <udpv4 externality="1" cost="0" mask="24">
                    <address>100.100.100.10</address>
                    <port>34567</port>
                </udpv4>
            </metatraffic_external_unicast_locators>
            <mutation_tries>300</mutation_tries>
        </builtin>
        <defaultUnicastLocatorList>
            <locator>
                <udpv4>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </udpv4>
            </locator>
        </defaultUnicastLocatorList>

        <defaultMulticastLocatorList>
            <locator>
                <udpv4>
                    <!-- Access as physical, like UDP -->
                    <port>7400</port>
                    <address>192.168.1.41</address>
                </udpv4>
            </locator>
        </defaultMulticastLocatorList>

        <default_external_unicast_locators>
            <udpv4 externality="1" cost="0" mask="24">
                <address>100.100.100.10</address>
                <port>23456</port>
            </udpv4>
        </default_external_unicast_locators>

        <ignore_non_matching_locators>true</ignore_non_matching_locators>
    </rtps>
</participant>
```

#### NOTE
For extended XML information, refer to [DomainParticipant configuration](../../../xml_configuration/domainparticipant.md#domainparticipantconfig) and [Builtin parameters](../../../xml_configuration/domainparticipant.md#builtin) XML sections.
