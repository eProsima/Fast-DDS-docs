<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="standard"></a>

<a id="standard-qos-policies"></a>

# Standard QoS Policies

This section explains each of the DDS standard QoS Policies:

<a id="deadlineqospolicy"></a>

<a id="id1"></a>

## DeadlineQosPolicy

This QoS policy raises an alarm when the frequency of new samples falls below a certain threshold.
It is useful for cases where data is expected to be updated periodically (see [`DeadlineQosPolicy`](../../../api_reference/dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE)).

On the publishing side, the deadline defines the maximum period in which the application is expected to supply a new
sample.
On the subscribing side, it defines the maximum period in which new samples should be received.

For [Topics](../../topic/topic.md#dds-layer-topic) with keys, this QoS is applied by key.
Suppose that the positions of some vehicles have to be published periodically.
In that case, it is possible to set the ID of the vehicle as the key of the data type and the deadline QoS to
the desired publication period.

List of QoS Policy data members:

| Data Member Name                                                                                                                    | Type         | Default Value                                                                                                                |
|-------------------------------------------------------------------------------------------------------------------------------------|--------------|------------------------------------------------------------------------------------------------------------------------------|
| [`period`](../../../api_reference/dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicy6periodE) | `Duration_t` | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It can be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#deadline-compatibilityrule) for further details.

<a id="deadline-compatibilityrule"></a>

<a id="compatibility-rule"></a>

### Compatibility Rule

To maintain the compatibility between DeadlineQosPolicy in DataReaders and DataWriters, the offered deadline period
(configured on the DataWriter) must be less than or equal to the requested deadline period (configured on the
DataReader), otherwise, the entities are considered to be incompatible.

The DeadlineQosPolicy must be set consistently with the [TimeBasedFilterQosPolicy](#timebasedfilterqospolicy), which means that the deadline
period must be higher or equal to the minimum separation.

<a id="deadline-example"></a>

<a id="example"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The DeadlineQosPolicy is constructed with an infinite period by default
// Change the period to 1 second
writer_qos.deadline().period.seconds = 1;
writer_qos.deadline().period.nanosec = 0;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_deadline_profile">
    <qos>
        <deadline>
            <period>
                <sec>1</sec>
            </period>
        </deadline>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_deadline_profile">
    <qos>
        <deadline>
            <period>
                <sec>1</sec>
            </period>
        </deadline>
    </qos>
</data_reader>
```

<a id="destinationorderqospolicy"></a>

<a id="id2"></a>

## DestinationOrderQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

Multiple [DataWriters](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) can send messages in the same [Topic](../../topic/topic.md#dds-layer-topic) using the same key, and on the [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) side all those
messages are stored within the same instance of data (see [`DestinationOrderQosPolicy`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE)).
This QoS policy controls the criteria used to determine the logical order of those messages.
The behavior of the system depends on the value of the [DestinationOrderQosPolicyKind](#destinationorderqospolicykind).

List of QoS Policy data members:

| Data Member Name                                                                                                                                | Type                                                            | Default Value                                                                                                                                                                                                                          |
|-------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicy4kindE) | [DestinationOrderQosPolicyKind](#destinationorderqospolicykind) | [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#destinationorder-compatibilityrule) for further details.

<a id="destinationorderqospolicykind"></a>

<a id="id3"></a>

### DestinationOrderQosPolicyKind

There are two possible values (see [`DestinationOrderQosPolicyKind`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKindE)):

* [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE): This indicates that the data is ordered based on the reception
  time at each DataReader,
  which means that the last received value should be the one kept.
  This option may cause that each DataReader ends up with a different final value, since the DataReaders may receive
  the data at different times.
* [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE): This indicates that the data is ordered based on the DataWriter
  timestamp at the time the message is sent.
  This option guarantees the consistency of the final value.

Both options depend on the values of the [OwnershipQosPolicy](#ownershipqospolicy) and [OwnershipStrengthQosPolicy](#ownershipstrengthqospolicy), meaning that
if the Ownership is set to EXCLUSIVE and the last value came from a DataWriter with low ownership strength, it will be
discarded.

<a id="destinationorder-compatibilityrule"></a>

<a id="id4"></a>

### Compatibility Rule

To maintain the compatibility between DestinationOrderQosPolicy in DataReaders and DataWriters when they have different
kind values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is:

[`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) < [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE)

Table with the possible combinations:

| DataWriter kind                                                                                                                                                                                                                        | DataReader kind                                                                                                                                                                                                                        | Compatibility   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) | [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) | Yes             |
| [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) | [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE)       | No              |
| [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE)       | [`BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind43BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOSE) | Yes             |
| [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE)       | [`BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS`](../../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKind40BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOSE)       | Yes             |

<a id="durabilityqospolicy"></a>

<a id="id5"></a>

## DurabilityQosPolicy

A [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) can send messages throughout a [Topic](../../topic/topic.md#dds-layer-topic) even if there are no [DataReaders](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) on the network.
Moreover, a DataReader that joins to the Topic after some data has been written could be interested in accessing
that information (see [`DurabilityQosPolicy`](../../../api_reference/dds_pim/core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE)).

The DurabilityQoSPolicy defines how the system will behave regarding those samples that existed on the Topic before
the DataReader joins.
The behavior of the system depends on the value of the [DurabilityQosPolicyKind](#durabilitykind).

List of QoS Policy data members:

| Data Member Name                                                                                                                    | Type                                       | Default Value                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicy4kindE) | [DurabilityQosPolicyKind](#durabilitykind) | [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE) for DataReaders<br/><br /><br/>[`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) for DataWriters |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

#### IMPORTANT
In order to receive past samples in the DataReader, besides setting this Qos Policy, it is required that the
[ReliabilityQosPolicy](#reliabilityqospolicy) is set to [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#durability-compatibilityrule) for further details.

<a id="durabilitykind"></a>

<a id="durabilityqospolicykind"></a>

### DurabilityQosPolicyKind

There are four possible values (see [`DurabilityQosPolicyKind`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE)):

* [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE): Past samples are ignored and a joining DataReader receives samples generated after the
  moment it matches.
* [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE): When a new DataReader joins, its History is filled with past samples.
* [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE): When a new DataReader joins, its History is filled with past samples, which are stored
  on persistent storage (see [Persistence Service](../../../persistence/persistence.md#persistence-service)).
* [`PERSISTENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind25PERSISTENT_DURABILITY_QOSE): When a new DataReader joins, its History is filled with past samples,
  which are stored on persistent storage (see [Persistence Service](../../../persistence/persistence.md#persistence-service)).

<a id="durability-compatibilityrule"></a>

<a id="id6"></a>

### Compatibility Rule

To maintain the compatibility between DurabilityQosPolicy in DataReaders and DataWriters when they have different kind
values, the DataWriter kind must be higher or equal to the DataReader kind.
And the order between the different kinds is:

[`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE) < [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) < [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE) <
[`PERSISTENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind25PERSISTENT_DURABILITY_QOSE)

Table with the possible combinations:

| DataWriter kind                                                                                                                                                                                  | DataReader kind                                                                                                                                                                                  | Compatibility   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | Yes             |
| [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | No              |
| [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | No              |
| [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | Yes             |
| [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | Yes             |
| [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | No              |
| [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)               | Yes             |
| [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) | Yes             |
| [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE)             | Yes             |

<a id="durability-example"></a>

<a id="id7"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The DurabilityQosPolicy is constructed with kind = VOLATILE_DURABILITY_QOS by default
// Change the kind to TRANSIENT_LOCAL_DURABILITY_QOS
writer_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_durability_profile">
    <qos>
        <durability>
            <kind>TRANSIENT_LOCAL</kind>
         </durability>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_durability_profile">
    <qos>
        <durability>
            <kind>VOLATILE</kind>
        </durability>
    </qos>
</data_reader>
```

<a id="durabilityserviceqospolicy"></a>

<a id="id8"></a>

## DurabilityServiceQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

This QoS Policy is used to configure the [HistoryQosPolicy](#historyqospolicy) and [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) of the fictitious
[DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) used when the [DurabilityQosPolicy](#durabilityqospolicy) kind is set to [`TRANSIENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE) or
[`PERSISTENT_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind25PERSISTENT_DURABILITY_QOSE) (see [`DurabilityServiceQosPolicy`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE)).

Those entities are used to simulate the persistent storage.
The fictitious DataReader reads the data written on the [Topic](../../topic/topic.md#dds-layer-topic) and stores it, so that if the user DataWriter does not
have the information requested by the user DataReaders, the fictitious DataWriter takes care of sending that
information.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                           | Type                                          | Default Value                                                                                                                                                            |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`service_cleanup_delay`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy21service_cleanup_delayE)       | `Duration_t`                                  | [`c_TimeZero`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds10c_TimeZeroE)                                                     |
| [`history_kind`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy12history_kindE)                         | [HistoryQosPolicyKind](#historyqospolicykind) | [`KEEP_LAST_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) |
| [`history_depth`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13history_depthE)                       | `int32_t`                                     | 1                                                                                                                                                                        |
| [`max_samples`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy11max_samplesE)                           | `int32_t`                                     | -1 (Length Unlimited)                                                                                                                                                    |
| [`max_instances`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13max_instancesE)                       | `int32_t`                                     | -1 (Length Unlimited)                                                                                                                                                    |
| [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy24max_samples_per_instanceE) | `int32_t`                                     | -1 (Length Unlimited)                                                                                                                                                    |
* [`service_cleanup_delay`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy21service_cleanup_delayE): It controls when the service can remove all the information regarding a data instance.
  That information is kept until all the following conditions are met:
  * The instance has been explicitly disposed and its InstanceState becomes [`NOT_ALIVE_DISPOSED_INSTANCE_STATE`](../../../api_reference/dds_pim/subscriber/instancestatekind.md#_CPPv4N8eprosima7fastdds3dds17InstanceStateKind33NOT_ALIVE_DISPOSED_INSTANCE_STATEE).
  * There is not any alive DataWriter writing the instance, which means that all existing writers either unregister the
    instance or lose their liveliness.
  * A time interval longer than the one established on the [`service_cleanup_delay`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy21service_cleanup_delayE) has elapsed since the moment the
    service detected that the two previous conditions were met.
* [`history_kind`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy12history_kindE): Controls the kind of the [HistoryQosPolicy](#historyqospolicy) associated with the Durability Service fictitious
  entities.
* [`history_depth`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13history_depthE): Controls the depth of the [HistoryQosPolicy](#historyqospolicy) associated with the Durability Service
  fictitious entities.
* [`max_samples`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy11max_samplesE): Controls the maximum number of samples of the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) associated with the
  Durability Service fictitious entities.
  This value must be higher than the maximum number of samples per instance.
* [`max_instances`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13max_instancesE): Controls the maximum number of instances of the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) associated with
  the Durability Service fictitious entities.
* [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy24max_samples_per_instanceE): Controls the maximum number of samples within an instance of the
  [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) associated with the Durability Service fictitious entities.
  This value must be lower than the maximum number of samples.

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

<a id="entityfactoryqospolicy"></a>

<a id="id9"></a>

## EntityFactoryQosPolicy

This QoS Policy controls the behavior of an [Entity](../entity/entity.md#dds-layer-core-entity) when it acts as a factory for other entities.
By default, all the entities are created enabled, but if you change the value of the [`autoenable_created_entities`](../../../api_reference/dds_pim/core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicy27autoenable_created_entitiesE)
to `false`, the new entities will be created disabled (see [`EntityFactoryQosPolicy`](../../../api_reference/dds_pim/core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE)).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                         | Type   | Default Value   |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|-----------------|
| [`autoenable_created_entities`](../../../api_reference/dds_pim/core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicy27autoenable_created_entitiesE) | `bool` | `true`          |

#### NOTE
This QoS Policy applies to [DomainParticipantFactory](../../domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) (as factory for [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)), DomainParticipant
(as factory for
[Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher), [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) and [Topic](../../topic/topic.md#dds-layer-topic)), Publisher (as factory for [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)) and Subscriber (as factory for
[DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader)).
<br />
It can be changed on enabled entities, but it only affects those entities created after the change.

<a id="id10"></a>

### Example

### C++

```c++
// This example uses a Participant, but it can also be applied to
// DomainParticipantFactory, Publisher and Subscriber entities
DomainParticipantQos participant_qos;
// The EntityFactoryQosPolicy is constructed with autoenable_created_entities = true by default
// Change it to false
participant_qos.entity_factory().autoenable_created_entities = false;
// Use modified QoS in the creation of the corresponding entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

This QoS Policy cannot be configured using XML for the moment.

<a id="groupqospolicy"></a>

<a id="groupdataqospolicy"></a>

## GroupDataQosPolicy

Allows the application to attach additional information to created [Publishers](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher) or [Subscribers](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber).
This data is common to all [DataWriters](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)/[DataReaders](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) belonging to the Publisher/Subscriber and it is propagated by
means of the built-in topics (see  [`GroupDataQosPolicy`](../../../api_reference/dds_pim/core/policy/groupdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18GroupDataQosPolicyE)).

This QoS Policy can be used in combination with DataWriter and DataReader listeners to implement a matching policy
similar to the [PartitionQosPolicy](#partitionqospolicy).

List of QoS Policy data members:

| Data Member Name   | Type                                                                                                           | Default Value   |
|--------------------|----------------------------------------------------------------------------------------------------------------|-----------------|
| collection         | std::vector<[`octet`](../../../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)> | Empty vector    |

#### NOTE
This QoS Policy applies to [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher) and [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) entities.
<br />
It can be changed on enabled entities.

<a id="id11"></a>

### Example

### C++

```c++
// This example uses a Publisher, but it can also be applied to Subscriber entities
PublisherQos publisher_qos;
// The GroupDataQosPolicy is constructed with an empty collection by default
// Collection is a private member so you need to use getters and setters to access

// Add data to the collection in initialization
std::vector<eprosima::fastdds::rtps::octet> vec;
// Add two new octets to group data vector
eprosima::fastdds::rtps::octet val = 3;
vec.push_back(val);
val = 10;
vec.push_back(val);
publisher_qos.group_data().data_vec(vec); // Setter function
// Use modified QoS in the creation of the corresponding entity
publisher_ = participant_->create_publisher(publisher_qos);

// Add data to the collection at runtime
vec = publisher_qos.group_data().data_vec(); // Getter to keep old values
val = 31;
vec.push_back(val);
publisher_qos.group_data().data_vec(vec); // Setter function
// Update the QoS in the corresponding entity
publisher_->set_qos(publisher_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_groupdata_profile">
    <qos>
        <groupData>
            <value>3.a</value>
        </groupData>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_groupdata_profile">
    <qos>
        <groupData>
            <value>3.a</value>
        </groupData>
    </qos>
</data_reader>
```

<a id="historyqospolicy"></a>

<a id="id12"></a>

## HistoryQosPolicy

This QoS Policy controls the behavior of the system when the value of an instance changes one or more times before it
can be successfully communicated to the existing DataReader entities.

List of QoS Policy data members:

| Data Member Name                                                                                                                | Type                                          | Default Value                                                                                                                                                            |
|---------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE)   | [HistoryQosPolicyKind](#historyqospolicykind) | [`KEEP_LAST_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) |
| [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) | `int32_t`                                     | 1                                                                                                                                                                        |
* [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE): Controls if the service should deliver only the most recent values, all the
  intermediate values or do something in between.
  See [HistoryQosPolicyKind](#historyqospolicykind) for further details.
* [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE): Establishes the maximum number of samples that must be kept on the history.
  It only has effect if the kind is set to [`KEEP_LAST_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) and it needs to be consistent with the
  [ResourceLimitsQosPolicy](#resourcelimitsqospolicy), which means that its value must be lower or equal to max_samples_per_instance.

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="historyqospolicykind"></a>

<a id="id13"></a>

### HistoryQosPolicyKind

There are two possible values (see [`HistoryQosPolicyKind`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE)):

* [`KEEP_LAST_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE): The service will only attempt to keep the most recent values of the instance and discard
  the older ones.
  The maximum number of samples to keep and deliver is defined by the depth of the HistoryQosPolicy, which needs to
  be consistent with the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) settings.
  If the limit defined by depth is reached, the system will discard the oldest sample to make room for a new one.
* [`KEEP_ALL_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE): The service will attempt to keep all the values of the instance until it can be delivered
  to all the existing Subscribers.
  If this option is selected, the depth will not have any effect, so the history is only limited by the values set in
  [ResourceLimitsQosPolicy](#resourcelimitsqospolicy).

<a id="historyqospolicy-consistency"></a>

<a id="consistency-rule"></a>

### Consistency rule

The HistoryQos must be set consistently with the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy), but also other QoS as
[DurabilityQosPolicy](#durabilityqospolicy) and [ReliabilityQosPolicy](#reliabilityqospolicy), so there are several cases to take into account:

* The [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) is only considered if the [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE) is set to
  [`KEEP_LAST_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE).
* The [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) must be consistent with the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) settings, which means
  that the [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) must be lower or equal than the [ResourceLimitsQosPolicy](#resourcelimitsqospolicy)’s
  [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE).
  Also, [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) must be equal or higher than the product of
  [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE) times [`max_instances`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13max_instancesE).
* The [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) cannot be lower or equal than zero.
  If an unlimited depth is required, please consider using [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE) as [`KEEP_ALL_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE).
* Setting the [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE) as [`KEEP_ALL_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE) entails that limits are set by the
  [ResourceLimitsQosPolicy](#resourcelimitsqospolicy) limits ([`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE) prior than
  [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE)).
* In the case of the [ReliabilityQosPolicy](#reliabilityqospolicy) [`ReliabilityQosPolicyKind`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE) being set to
  [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) and the [HistoryQosPolicy](#historyqospolicy) [`kind`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy4kindE) being set to
  [`KEEP_ALL_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE), when the resource limits are reached the DataWriter [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) call will
  block (up to [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE)) until the oldest sample has been acknowledged by all
  readers and can be removed.
  If [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) is reached, the [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) call will return with a
  timeout error.

<a id="id14"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The HistoryQosPolicy is constructed with kind = KEEP_LAST and depth = 1 by default
// It is possible to adjust the depth and keep the kind as KEEP_LAST
writer_qos.history().depth = 20;
// Or you can also change the kind to KEEP_ALL (depth will not be used).
writer_qos.history().kind = KEEP_ALL_HISTORY_QOS;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<topic>
    <historyQos>
        <kind>KEEP_LAST</kind> <!-- string -->
        <depth>20</depth> <!-- uint32 -->
    </historyQos>
</topic>
```

<a id="latencybudgetqospolicy"></a>

<a id="id15"></a>

## LatencyBudgetQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

This QoS Policy specifies the maximum acceptable delay from the time the data is
written until the data is inserted on the DataReader History and notified of the fact.
That delay by default is set to 0 in order to optimize the internal operations (see [`LatencyBudgetQosPolicy`](../../../api_reference/dds_pim/core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE)).

List of QoS Policy data members:

| Data Member Name                                                                                                                                  | Type         | Default Value                                                                                                        |
|---------------------------------------------------------------------------------------------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------|
| [`duration`](../../../api_reference/dds_pim/core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicy8durationE) | `Duration_t` | [`c_TimeZero`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds10c_TimeZeroE) |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It can be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#latencybudget-compatibilityrule) for further details.

<a id="latencybudget-compatibilityrule"></a>

<a id="id16"></a>

### Compatibility Rule

To maintain the compatibility between LatencyBudgetQosPolicy in DataReaders and DataWriters, the DataWriter duration
must be lower or equal to the DataReader duration.

<a id="lifespanqospolicy"></a>

<a id="id17"></a>

## LifespanQosPolicy

Each data sample written by a [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) has an associated expiration time beyond which the data is removed from the
DataWriter and DataReader history as well as from the transient and persistent information caches
(see [`LifespanQosPolicy`](../../../api_reference/dds_pim/core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE)).

By default, the duration is infinite, which means that there is not a maximum duration for the validity of the
samples written by the DataWriter.

The expiration time is computed by adding the duration to the source timestamp, which can be calculated automatically
if [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) member function is called or supplied by the application by means of
`write_w_timestamp()` member function.
The DataReader is allowed to use the reception timestamp instead of the source timestamp.

List of QoS Policy data members:

| Data Member Name                                                                                                                        | Type         | Default Value                                                                                                                |
|-----------------------------------------------------------------------------------------------------------------------------------------|--------------|------------------------------------------------------------------------------------------------------------------------------|
| [`duration`](../../../api_reference/dds_pim/core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicy8durationE) | `Duration_t` | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It can be changed on enabled entities.

<a id="id18"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The LifespanQosPolicy is constructed with duration set to infinite by default
// Change the duration to 5 s
writer_qos.lifespan().duration = {5, 0};
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_lifespan_profile">
    <qos>
        <lifespan>
            <duration>
                <sec>5</sec>
            </duration>
        </lifespan>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_lifespan_profile">
    <qos>
        <lifespan>
            <duration>
                <sec>5</sec>
            </duration>
        </lifespan>
    </qos>
</data_reader>
```

<a id="livelinessqospolicy"></a>

<a id="id19"></a>

## LivelinessQosPolicy

This QoS Policy controls the mechanism used by the service to ensure that a particular entity on the network is still
alive.
There are different settings that allow distinguishing between applications where data is updated periodically and
applications where data is changed sporadically.
It also allows customizing the application regarding the kind of failures that should be detected by the liveliness
mechanism (see [`LivelinessQosPolicy`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE)).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                   | Type                                                | Default Value                                                                                                                                                                        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy4kindE)                                | [LivelinessQosPolicyKind](#livelinessqospolicykind) | [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE) |
| [`lease_duration`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy14lease_durationE)           | `Duration_t`                                        | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE)                                                         |
| [`announcement_period`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy19announcement_periodE) | `Duration_t`                                        | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE)                                                         |
* [`kind`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy4kindE): This data member establishes if the service needs to assert the liveliness
  automatically or if it needs to wait until the liveliness is asserted by the publishing side.
  See [LivelinessQosPolicyKind](#livelinessqospolicykind) for further details.
* [`lease_duration`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy14lease_durationE): Amount of time to wait since the last time the DataWriter asserts its
  liveliness to consider that it is no longer alive.
* [`announcement_period`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy19announcement_periodE): Amount of time between consecutive liveliness messages sent by the
  DataWriter.
  This data member only takes effect if the kind is [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE) or
  [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) and needs to be lower than the
  [`lease_duration`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy14lease_durationE).

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#liveliness-compatibilityrule) for further details.

<a id="livelinessqospolicykind"></a>

<a id="id20"></a>

### LivelinessQosPolicyKind

There are three possible values (see [`LivelinessQosPolicyKind`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKindE)):

* [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE): The service takes the responsibility for renewing the leases at the required rates, as
  long as the local process where the participant is running and the link connecting it to remote participants exists,
  the entities within the remote participant will be considered alive.
  This kind is suitable for applications that only need to detect whether a remote application is still running.
* The two Manual modes require that the application on the publishing side asserts the liveliness periodically
  before the lease_duration timer expires.
  Publishing any new data value implicitly asserts the DataWriter’s liveliness, but it can be done explicitly
  by calling the assert_liveliness member function.
  * [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE): If one of the entities in the publishing side asserts its liveliness,
    the service deduces that all other entities within the same DomainParticipant are also alive.
  * [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE): This mode is more restrictive and requires that at least one instance within
    the DataWriter is asserted to consider that the DataWriter is alive.

<a id="liveliness-compatibilityrule"></a>

<a id="id21"></a>

### Compatibility Rule

To maintain the compatibility between LivelinessQosPolicy in DataReaders and DataWriters, the DataWriter kind must be
higher or equal to the DataReader kind.
And the order between the different kinds is:

[`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE) < [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) < [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)

Table with the possible combinations:

| DataWriter kind                                                                                                                                                                                              | DataReader kind                                                                                                                                                                                              | Compatibility   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | Yes             |
| [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | No              |
| [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | No              |
| [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | Yes             |
| [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | Yes             |
| [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | No              |
| [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | [`AUTOMATIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)                         | Yes             |
| [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | [`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind36MANUAL_BY_PARTICIPANT_LIVELINESS_QOSE) | Yes             |
| [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | [`MANUAL_BY_TOPIC_LIVELINESS_QOS`](../../../api_reference/dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind30MANUAL_BY_TOPIC_LIVELINESS_QOSE)             | Yes             |

Additionally, the [`lease_duration`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy14lease_durationE) of the DataWriter must not be greater than
the [`lease_duration`](../../../api_reference/dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicy14lease_durationE) of the DataReader.

<a id="id22"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The LivelinessQosPolicy is constructed with kind = AUTOMATIC by default
// Change the kind to MANUAL_BY_PARTICIPANT
writer_qos.liveliness().kind = MANUAL_BY_PARTICIPANT_LIVELINESS_QOS;
// The LivelinessQosPolicy is constructed with lease_duration set to infinite by default
// Change the lease_duration to 1 second
writer_qos.liveliness().lease_duration = {1, 0};
// The LivelinessQosPolicy is constructed with announcement_period set to infinite by default
// Change the announcement_period to 1 ms
writer_qos.liveliness().announcement_period = {0, 1000000};
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_liveliness_profile">
    <qos>
        <liveliness>
            <announcement_period>
                <nanosec>1000000</nanosec>
            </announcement_period>
            <lease_duration>
                <sec>1</sec>
            </lease_duration>
            <kind>AUTOMATIC</kind>
        </liveliness>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_liveliness_profile">
    <qos>
        <liveliness>
            <lease_duration>
                <sec>1</sec>
            </lease_duration>
            <kind>AUTOMATIC</kind>
        </liveliness>
    </qos>
</data_reader>
```

<a id="ownershipqospolicy"></a>

<a id="id23"></a>

## OwnershipQosPolicy

This QoS Policy specifies whether it is allowed for multiple DataWriters to update the same instance of data, and if
so, how these modifications should be arbitrated (see [`OwnershipQosPolicy`](../../../api_reference/dds_pim/core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE)).

List of QoS Policy data members:

| Data Member Name                                                                                                                  | Type                                              | Default Value                                                                                                                                                              |
|-----------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicy4kindE) | [OwnershipQosPolicyKind](#ownershipqospolicykind) | [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE) |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#ownership-compatibilityrule) for further details.

<a id="ownershipqospolicykind"></a>

<a id="id24"></a>

### OwnershipQosPolicyKind

There are two possible values (see [`OwnershipQosPolicyKind`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKindE)):

* [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE): This option indicates that the service does not enforce unique ownership for each
  instance.
  In this case, multiple DataWriters are allowed to update the same data instance and all the updates are made
  available to the existing DataReaders.
  Those updates are also subject to the [TimeBasedFilterQosPolicy](#timebasedfilterqospolicy) or [HistoryQosPolicy](#historyqospolicy) settings, so they
  can be filtered.
* [`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE): This option indicates that each instance can only be updated by one DataWriter, meaning
  that at any point in time a single DataWriter owns each instance and is the only one whose modifications will be
  visible for the existing DataReaders.
  The owner can be changed dynamically according to the highest strength between the alive DataWriters, which has not
  violated the deadline contract concerning the data instances.
  That strength can be changed using the [OwnershipStrengthQosPolicy](#ownershipstrengthqospolicy).
  In case two DataWriters have the same strength value,
  the DataWriter with a lower GUID value would be the owner of the topic.

<a id="ownership-compatibilityrule"></a>

<a id="id25"></a>

### Compatibility Rule

To maintain the compatibility between OwnershipQosPolicy in [DataReaders](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriters](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), the DataWriter kind must be
equal to the DataReader kind.

Table with the possible combinations:

| DataWriter kind                                                                                                                                                                  | DataReader kind                                                                                                                                                                  | Compatibility   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE)       | [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE)       | Yes             |
| [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE)       | [`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE) | No              |
| [`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE) | [`SHARED_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE)       | No              |
| [`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE) | [`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE) | Yes             |

<a id="id26"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The OwnershipQosPolicy is constructed with kind = SHARED by default
// Change the kind to EXCLUSIVE
writer_qos.ownership().kind = EXCLUSIVE_OWNERSHIP_QOS;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_ownership_profile">
    <qos>
      <ownership>
        <kind>EXCLUSIVE</kind>
      </ownership>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_ownership_profile">
    <qos>
        <ownership>
            <kind>EXCLUSIVE</kind>
        </ownership>
    </qos>
</data_reader>
```

<a id="ownershipstrengthqospolicy"></a>

<a id="id27"></a>

## OwnershipStrengthQosPolicy

This QoS Policy specifies the value of the strength used to arbitrate among multiple DataWriters that attempt to
modify the same data instance. It is only applicable if the [OwnershipQosPolicy](#ownershipqospolicy) kind is set to
[`EXCLUSIVE_OWNERSHIP_QOS`](../../../api_reference/dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind23EXCLUSIVE_OWNERSHIP_QOSE).
See [`OwnershipStrengthQosPolicy`](../../../api_reference/dds_pim/core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                    | Type       |   Default Value |
|-----------------------------------------------------------------------------------------------------------------------------------------------------|------------|-----------------|
| [`value`](../../../api_reference/dds_pim/core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicy5valueE) | `uint32_t` |               0 |

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It can be changed on enabled entities.

<a id="id28"></a>

### Example

### C++

```c++
// This example only applies to DataWriter entities
DataWriterQos writer_qos;
// The OwnershipStrengthQosPolicy is constructed with value 0 by default
// Change the strength to 10
writer_qos.ownership_strength().value = 10;
// Use modified QoS in the creation of the corresponding DataWriter
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_ownership_strength_profile">
    <qos>
        <ownershipStrength>
            <value>10</value>
        </ownershipStrength>
    </qos>
</data_writer>
```

<a id="partitionqospolicy"></a>

<a id="id29"></a>

## PartitionQosPolicy

This Qos Policy allows the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match, but also they have to share
at least one logical partition (see [`PartitionQosPolicy`](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicyE)).

The empty string is also considered as a valid partition and it matches with other partition names using the same rules
of string matching and regular-expression matching used for any other partition name.

List of QoS Policy data members:

| Data Member Name                                                                                                                            | Type                                                                                                                                                     | Default Value        |
|---------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------|
| [`max_size`](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#_CPPv4NK8eprosima7fastdds3dds18PartitionQosPolicy8max_sizeEv) | `uint32_t`                                                                                                                                               | 0 (Length Unlimited) |
| [`names`](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#_CPPv4NK8eprosima7fastdds3dds18PartitionQosPolicy5namesEv)       | [`SerializedPayload_t`](../../../api_reference/rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) | Empty List           |
* [`max_size`](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#_CPPv4NK8eprosima7fastdds3dds18PartitionQosPolicy8max_sizeEv): Maximum size for the list of partition names.
* [`names`](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#_CPPv4NK8eprosima7fastdds3dds18PartitionQosPolicy5namesEv): List of partition names.

#### NOTE
This QoS Policy applies to [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher) and [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) entities.
<br />
Partitions can also be explicitly defined at the endpoint level to override this configuration. Information
to do so can be found [here](../../../property_policies/non_consolidated_qos.md#property-policies-partitions).
<br />
It can be changed on enabled entities.

<a id="id30"></a>

### Example

### C++

```c++
// This example uses a Publisher, but it can also be applied to Subscriber entities
PublisherQos publisher_qos;
// The PartitionsQosPolicy is constructed with max_size = 0 by default
// Max_size is a private member so you need to use getters and setters to access
// Change the max_size to 20
publisher_qos.partition().set_max_size(20); // Setter function
// The PartitionsQosPolicy is constructed with an empty list of partitions by default
// Partitions is a private member so you need to use getters and setters to access

// Add new partitions in initialization
std::vector<std::string> part;
part.push_back("part1");
part.push_back("part2");
publisher_qos.partition().names(part); // Setter function
// Use modified QoS in the creation of the corresponding entity
publisher_ = participant_->create_publisher(publisher_qos);

// Add data to the collection at runtime
part = publisher_qos.partition().names(); // Getter to keep old values
part.push_back("part3");
publisher_qos.partition().names(part); // Setter function
// Update the QoS in the corresponding entity
publisher_->set_qos(publisher_qos);
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

<a id="presentationqospolicy"></a>

<a id="id31"></a>

## PresentationQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

This QoS Policy specifies how the samples representing changes to data instances are presented to the subscribing
application.
It controls the extent to which changes to data instances can be made dependent on each other, as well as the kind
of dependencies that can be propagated and maintained.
See [`PresentationQosPolicy`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                               | Type                                                                          | Default Value                                                                                                                                                                                                    |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`access_scope`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy12access_scopeE)       | [PresentationQosPolicyAccessScopeKind](#presentationqospolicyaccessscopekind) | [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) |
| [`coherent_access`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy15coherent_accessE) | `bool`                                                                        | `false`                                                                                                                                                                                                          |
| [`ordered_access`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy14ordered_accessE)   | `bool`                                                                        | `false`                                                                                                                                                                                                          |
* [`access_scope`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy12access_scopeE): Determines the largest scope spanning the entities for which the order and coherency can be
  preserved.
  See [PresentationQosPolicyAccessScopeKind](#presentationqospolicyaccessscopekind) for further details.
* [`coherent_access`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy15coherent_accessE): Controls whether the service will preserve grouping of changes made on the publishing side,
  such that they are received as a unit on the subscribing side.
* [`ordered_access`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy14ordered_accessE): Controls whether the service supports the ability of the subscriber to see changes in the same
  order as they occurred on the publishing side.

#### NOTE
This QoS Policy applies to [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher) and [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) entities.
<br />
It cannot be changed on enabled entities.

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule. See [Compatibility Rule](#presentation-compatibilityrule)
for further details.

<a id="presentationqospolicyaccessscopekind"></a>

<a id="id32"></a>

### PresentationQosPolicyAccessScopeKind

There are three possible values, which have different behaviors depending on the values of coherent_access and
ordered_access variables (see [`PresentationQosPolicyAccessScopeKind`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKindE)):

* [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE): The changes to a data instance do not need to be coherent nor ordered with respect to
  the changes to any other instance, which means that the order and coherent changes apply to each instance separately.
  * Enabling the coherent_access, in this case, has no effect on how the subscriber can access the data as the scope
    is limited to each instance, changes to separate instances are considered independent and thus cannot be grouped
    by a coherent change.
  * Enabling the ordered_access, in this case, only affects to the changes within the same instance.
    Therefore, the changes made to two instances are not necessarily seen in the order they occur even if the same
    application thread and DataWriter made them.
* [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE): The scope spans to all the instances within the same DataWriter.
  * Enabling the coherent_access makes that the grouping made with changes within the same DataWriter will be
    available as coherent with respect to other changes to instances in that DataWriter, but will not be grouped with
    changes made to instances belonging to different DataWriters.
  * Enabling the ordered_access means that the changes made by a single DataWriter are made available to the
    subscribers in the same order that they occur, but the changes made to instances through different DataWriters are
    not necessarily seen in order.
* [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE): The scope spans to all the instances belonging to DataWriters within the same Publisher.
  * Enabling the coherent_access, means that the coherent changes made to instances through DataWriters attached to a
    common Publisher are made available as a unit to remote subscribers.
  * Enabling the ordered_access with this scope makes that the changes done by any of the DataWriters attached to the
    same Publisher are made available to the subscribers in the same order they occur.

<a id="presentation-compatibilityrule"></a>

<a id="id33"></a>

### Compatibility Rule

To maintain the compatibility between PresentationQosPolicy in DataReaders and DataWriters, the Publisher
[`access_scope`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy12access_scopeE) must be higher or equal to the Subscriber [`access_scope`](../../../api_reference/dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicy12access_scopeE).
And the order between the different access scopes is:

```default
|INSTANCE_PRESENTATION_QOS-api| < |TOPIC_PRESENTATION_QOS-api| < |GROUP_PRESENTATION_QOS-api|
```

Table with the possible combinations:

| Publisher scope                                                                                                                                                                                                  | Subscriber scope                                                                                                                                                                                                 | Compatibility   |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | Yes             |
| [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | No              |
| [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | No              |
| [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | Yes             |
| [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | Yes             |
| [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | No              |
| [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | [`INSTANCE_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind25INSTANCE_PRESENTATION_QOSE) | Yes             |
| [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | [`TOPIC_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22TOPIC_PRESENTATION_QOSE)       | Yes             |
| [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | [`GROUP_PRESENTATION_QOS`](../../../api_reference/dds_pim/core/policy/presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKind22GROUP_PRESENTATION_QOSE)       | Yes             |

Additionally, the coherent_access and ordered_access of the Subscriber can only be enabled if they are also enabled on
the Publisher.

<a id="readerdatalifecycleqospolicy"></a>

<a id="id34"></a>

## ReaderDataLifecycleQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) with respect to the lifecycle of the data instances it
manages, that is, the instances that have been received and for which the DataReader maintains some internal resources.
The DataReader maintains the samples that have not been taken by the application, subject to the constraints imposed by
[HistoryQosPolicy](#historyqospolicy) and [ResourceLimitsQosPolicy](#resourcelimitsqospolicy).
See [`ReaderDataLifecycleQosPolicy`](../../../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicyE).

Under normal circumstances, the DataReader can only reclaim the resources associated with data instances if there are
no writers and all the samples have been taken. But this fact can cause problems if the application does not take those
samples as the service will prevent the DataReader from reclaiming the resources and they will remain in the DataReader
indefinitely. This QoS exist to avoid that situation.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                                                 | Type         | Default Value                                                                                                                |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|------------------------------------------------------------------------------------------------------------------------------|
| [`autopurge_no_writer_samples_delay`](../../../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicy33autopurge_no_writer_samples_delayE) | `Duration_t` | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |
| [`autopurge_disposed_samples_delay`](../../../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicy32autopurge_disposed_samples_delayE)   | `Duration_t` | [`c_TimeInfinite`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds14c_TimeInfiniteE) |
* [`autopurge_no_writer_samples_delay`](../../../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicy33autopurge_no_writer_samples_delayE):
  Defines the maximum duration the DataReader must retain the information
  regarding an instance once its [`instance_state`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo14instance_stateE) becomes [`NOT_ALIVE_NO_WRITERS_INSTANCE_STATE`](../../../api_reference/dds_pim/subscriber/instancestatekind.md#_CPPv4N8eprosima7fastdds3dds17InstanceStateKind35NOT_ALIVE_NO_WRITERS_INSTANCE_STATEE).
  After this time elapses, the DataReader purges all the internal information of the instance, including the untaken
  samples that will be lost.
* [`autopurge_disposed_samples_delay`](../../../api_reference/dds_pim/core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicy32autopurge_disposed_samples_delayE):
  Defines the maximum duration the DataReader must retain the information
  regarding an instance once its [`instance_state`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo14instance_stateE) becomes [`NOT_ALIVE_DISPOSED_INSTANCE_STATE`](../../../api_reference/dds_pim/subscriber/instancestatekind.md#_CPPv4N8eprosima7fastdds3dds17InstanceStateKind33NOT_ALIVE_DISPOSED_INSTANCE_STATEE).
  After this time elapses, the DataReader purges all the samples for the instance.

#### NOTE
This QoS Policy applies to [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It can be changed on enabled entities.

<a id="reliabilityqospolicy"></a>

<a id="id35"></a>

## ReliabilityQosPolicy

This QoS Policy indicates the level of reliability offered and requested by the service.
See [`ReliabilityQosPolicy`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                 | Type                                                  | Default Value                                                                                                                                                                                                                                                                                                                                                                                                                              |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`kind`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy4kindE)                            | [ReliabilityQosPolicyKind](#reliabilityqospolicykind) | [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE)<br/>for DataReaders<br/><br /><br/>[`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)<br/>for DataWriters |
| [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) | `Duration_t`                                          | 100 ms                                                                                                                                                                                                                                                                                                                                                                                                                                     |
* [`kind`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy4kindE): Specifies the behavior of the service regarding delivery of the samples.
  See [ReliabilityQosPolicyKind](#reliabilityqospolicykind) for further details.
* [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE): Configures the maximum duration that the write operation can be
  blocked.

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

#### NOTE
It is inconsistent to set an infinite [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE).

#### IMPORTANT
Setting this QoS Policy to [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) affects to the [DurabilityQosPolicy](#durabilityqospolicy), making the
endpoints behave as [`VOLATILE_DURABILITY_QOS`](../../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE).

#### WARNING
For DataWriters and DataReaders to match, they must follow the compatibility rule.
See [Compatibility Rule](#reliability-compatibilityrule) for further details.

<a id="reliabilityqospolicykind"></a>

<a id="id36"></a>

### ReliabilityQosPolicyKind

There are two possible values ():

* [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE): It indicates that it is acceptable not to retransmit the missing samples, so the
  messages are sent without waiting for an arrival confirmation.
  Presumably new values for the samples are generated often enough that it is not necessary to re-send any sample.
  However, the data samples sent by the same DataWriter will be stored in the DataReader history in the same order they
  occur.
  In other words, even if the DataReader misses some data samples, an older value will never overwrite a newer value.
* [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE): It indicates that the service will attempt to deliver all samples of the DataWriter’s
  history expecting an arrival confirmation from the DataReader.
  The data samples sent by the same DataWriter cannot be made available to the DataReader if there are previous samples
  that have not been received yet.
  The service will retransmit the lost data samples in order to reconstruct a correct snapshot of the DataWriter
  history before it is accessible by the DataReader.

  This option may block the write operation, hence the [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) is set that
  will unblock it once the time expires.
  But if the [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) expires before the data is sent, the write operation will
  return an error.

<a id="reliability-compatibilityrule"></a>

<a id="id37"></a>

### Compatibility Rule

To maintain the compatibility between ReliabilityQosPolicy in DataReaders and DataWriters, the DataWriter kind
must be higher or equal to the DataReader kind.
And the order between the different kinds is:

[`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) < [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)

Table with the possible combinations:

| DataWriter kind                                                                                                                                                                              | DataReader kind                                                                                                                                                                              | Compatibility   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) | [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) | Yes             |
| [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) | [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)       | No              |
| [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)       | [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) | Yes             |
| [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)       | [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)       | Yes             |

<a id="id38"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The ReliabilityQosPolicy is constructed with kind = BEST_EFFORT by default
// Change the kind to RELIABLE
writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
// The ReliabilityQosPolicy is constructed with max_blocking_time = 100ms by default
// Change the max_blocking_time to 1s
writer_qos.reliability().max_blocking_time = {1, 0};
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_reliability_profile">
    <qos>
        <reliability>
            <kind>RELIABLE</kind>
            <max_blocking_time>
                <sec>1</sec>
            </max_blocking_time>
        </reliability>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_reliability_profile">
    <qos>
        <reliability>
            <kind>BEST_EFFORT</kind>
        </reliability>
    </qos>
</data_reader>
```

<a id="resourcelimitsqospolicy"></a>

<a id="id39"></a>

## ResourceLimitsQosPolicy

This QoS Policy controls the resources that the service can use in order to meet the requirements imposed by the
application and other QoS Policies.
See [`ResourceLimitsQosPolicy`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                     | Type      | Default Value         |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|-----------------------|
| [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE)                           | `int32_t` | -1 (Length Unlimited) |
| [`max_instances`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13max_instancesE)                       | `int32_t` | -1 (Length Unlimited) |
| [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE) | `int32_t` | -1 (Length Unlimited) |
| [`allocated_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy17allocated_samplesE)               | `int32_t` | 100                   |
| [`extra_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE)                       | `int32_t` | 1                     |
* [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE): Controls the maximum number of samples that the DataWriter or DataReader
  can manage across all the instances associated with it.
  In other words, it represents the maximum samples that the middleware can store for a DataReader or DataWriter.
  **Value less or equal to 0 means infinite resources.**
* [`max_instances`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13max_instancesE): Controls the maximum number of instances that a DataWriter or
  DataReader can manage.
  It will be internally fixed to 1 for topics without keys.
  **Value less or equal to 0 means infinite resources.**
* [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE): Controls the maximum number of samples within an instance
  that the DataWriter or DataReader can manage.
  It will be ignored for topics without keys ([`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) will be used instead).
  **Value less or equal to 0 means infinite resources.**
* [`allocated_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy17allocated_samplesE): States the number of samples that will be allocated on
  initialization.
  Will be limited by [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) (i.e. the minimum between both values will be used).
* [`extra_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE): States the number of extra samples that will be allocated on
  the pool, so the maximum number of samples on the pool will be
  [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) plus [`extra_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE).
  These extra samples act as a reservoir of samples even when the history is full.

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="resourcelimits-consistencyrule"></a>

<a id="id40"></a>

### Consistency Rule

To maintain the consistency within the ResourceLimitsQosPolicy, the values of the data members must follow the next
conditions:

* The value of [`max_samples`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) must be higher or equal to the value of
  [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE).
* The value established for the [HistoryQosPolicy](#historyqospolicy) [`depth`](../../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) must be lower or equal to the
  value stated for
  [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE).

<a id="id41"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
DataWriterQos writer_qos;
// The ResourceLimitsQosPolicy is constructed with max_samples = LENGTH_UNLIMITED by default
// Change max_samples to 2000
writer_qos.resource_limits().max_samples = 2000;
// The ResourceLimitsQosPolicy is constructed with max_instances = LENGTH_UNLIMITED by default
// Change max_instances to 20
writer_qos.resource_limits().max_instances = 20;
// The ResourceLimitsQosPolicy is constructed with max_samples_per_instance = LENGTH_UNLIMITED by default
// Change max_samples_per_instance to 100
writer_qos.resource_limits().max_samples_per_instance = 100;
// The ResourceLimitsQosPolicy is constructed with allocated_samples = 100 by default
// Change allocated_samples to 50
writer_qos.resource_limits().allocated_samples = 50;
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_resource_limits_profile">
    <topic>
        <resourceLimitsQos>
            <max_samples>2000</max_samples>
            <max_instances>20</max_instances>
            <max_samples_per_instance>100</max_samples_per_instance>
            <allocated_samples>50</allocated_samples>
        </resourceLimitsQos>
    </topic>
</data_writer>

<data_reader profile_name="reader_xml_conf_resource_limits_profile">
    <topic>
        <resourceLimitsQos>
            <max_samples>2000</max_samples>
            <max_instances>20</max_instances>
            <max_samples_per_instance>100</max_samples_per_instance>
            <allocated_samples>50</allocated_samples>
        </resourceLimitsQos>
    </topic>
</data_reader>
```

<a id="timebasedfilterqospolicy"></a>

<a id="id42"></a>

## TimeBasedFilterQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

Filter that allows a [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) to specify that it is interested only in a subset of the values of the data.
This filter states that the DataReader does not want to receive more than one value each
[`minimum_separation`](../../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicy18minimum_separationE), regardless of how fast the changes occur.
See [`TimeBasedFilterQosPolicy`](../../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicyE).

The [`minimum_separation`](../../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicy18minimum_separationE) must be lower than the [DeadlineQosPolicy](#deadlineqospolicy)
[`period`](../../../api_reference/dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicy6periodE).
By default, the [`minimum_separation`](../../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicy18minimum_separationE) is zero, which means that the DataReader is
potentially interested in all the values.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                           | Type         | Default Value                                                                                                        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------|
| [`minimum_separation`](../../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicy18minimum_separationE) | `Duration_t` | [`c_TimeZero`](../../../api_reference/dds_pim/core/Time_t/const_values.md#_CPPv4N8eprosima7fastdds3dds10c_TimeZeroE) |

#### NOTE
This QoS Policy applies to [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It can be changed on enabled entities.

<a id="topicdataqospolicy"></a>

<a id="id43"></a>

## TopicDataQosPolicy

Allows the application to attach additional information to a created [Topic](../../topic/topic.md#dds-layer-topic) so that when it is discovered by a remote
application, it can access the data and use it.
See [`TopicDataQosPolicy`](../../../api_reference/dds_pim/core/policy/topicdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18TopicDataQosPolicyE).

List of QoS Policy data members:

| Data Member Name   | Type                                                                                                           | Default Value   |
|--------------------|----------------------------------------------------------------------------------------------------------------|-----------------|
| collection         | std::vector<[`octet`](../../../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)> | Empty vector    |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic) entities.
<br />
It can be changed even if it is already created.

<a id="id44"></a>

### Example

### C++

```c++
// This example only applies to Topic entities
TopicQos topic_qos;
// The TopicDataQosPolicy is constructed with an empty vector by default
std::vector<eprosima::fastdds::rtps::octet> vec;
// Add two new octets to topic data vector
eprosima::fastdds::rtps::octet val = 3;
vec.push_back(val);
val = 10;
vec.push_back(val);
topic_qos.topic_data().data_vec(vec); // Setter Function
// Use modified QoS in the creation of the corresponding Topic
topic_ = participant_->create_topic("<topic_name>", "<type_name>", topic_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_topicdata_profile">
    <qos>
        <topicData>
            <value>3.a</value>
        </topicData>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_topicdata_profile">
    <qos>
        <topicData>
            <value>3.a</value>
        </topicData>
    </qos>
</data_reader>
```

<a id="transportpriorityqospolicy"></a>

<a id="id45"></a>

## TransportPriorityQosPolicy

The purpose of this QoS Policy is to allow the service to take advantage of those transports capable of sending
messages with different priorities. It establishes the priority of the underlying transport used to send the data.
See [`TransportPriorityQosPolicy`](../../../api_reference/dds_pim/core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE)

You can choose any value within the 32-bit range for the priority. The higher the value, the higher the priority.

List of QoS Policy data members:

| Data Member Name                                                                                                                                    | Type      |   Default Value |
|-----------------------------------------------------------------------------------------------------------------------------------------------------|-----------|-----------------|
| [`value`](../../../api_reference/dds_pim/core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicy5valueE) | `int32_t` |               0 |

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It can be changed on enabled entities.

<a id="id46"></a>

### Example

### C++

```c++
// This example only applies to DataWriter entities
DataWriterQos writer_qos;
// The TransportPriorityQosPolicy is constructed with value 0 by default
// Change the value to 12
writer_qos.transport_priority().value = 12;
// Use modified QoS in the creation of the corresponding DataWriter
writer_ = publisher_->create_datawriter(topic_, writer_qos);

// Change the TransportPriorityQosPolicy at runtime
writer_qos.transport_priority().value = 23;
// Update the QoS in the corresponding entity
writer_->set_qos(writer_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_transport_priority_profile">
    <qos>
        <transport_priority>12</transport_priority>
    </qos>
</data_writer>
```

<a id="userdataqospolicy"></a>

<a id="id47"></a>

## UserDataQosPolicy

Allows the application to attach additional information to the [Entity](../entity/entity.md#dds-layer-core-entity) object so that when the entity is discovered
the remote application can access the data and use it.
For example, it can be used to attach the security credentials to authenticate the source from the remote application.
See [`UserDataQosPolicy`](../../../api_reference/dds_pim/core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE).

List of QoS Policy data members:

| Data Member Name   | Type                                                                                                           | Default Value   |
|--------------------|----------------------------------------------------------------------------------------------------------------|-----------------|
| collection         | std::vector<[`octet`](../../../api_reference/rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)> | Empty vector    |

#### NOTE
This QoS Policy applies to [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It can be changed on enabled entities.

<a id="id48"></a>

### Example

### C++

```c++
// This example uses a DataWriter, but it can also be applied to DomainParticipant and DataReader entities
DataWriterQos writer_qos;
std::vector<eprosima::fastdds::rtps::octet> vec;
// Add two new octets to user data vector
eprosima::fastdds::rtps::octet val = 3;
vec.push_back(val);
val = 10;
vec.push_back(val);
writer_qos.user_data().data_vec(vec); // Setter Function
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);
```

### XML

```xml
<participant profile_name="participant_xml_conf_userdata_profile">
    <rtps>
        <userData>
            <value>3.a</value>
        </userData>
    </rtps>
</participant>

<data_writer profile_name="writer_xml_conf_userdata_profile">
    <qos>
        <userData>
            <value>3.a</value>
        </userData>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_userdata_profile">
    <qos>
        <userData>
            <value>3.a</value>
        </userData>
    </qos>
</data_reader>
```

<a id="writerdatalifecycleqospolicy"></a>

<a id="id49"></a>

## WriterDataLifecycleQosPolicy

#### WARNING
This QoS Policy will be implemented in future releases.

This QoS Policy specifies the behavior of the DataWriter with respect to the lifecycle of the data instances it manages
, that is, the instance that has been either explicitly registered with the DataWriter using the register operations
or implicitly by directly writing data.

The [`autodispose_unregistered_instances`](../../../api_reference/dds_pim/core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicy34autodispose_unregistered_instancesE)
controls whether a DataWriter will automatically dispose an instance each time
it is unregistered. Even if it is disabled, the application can still get the same result if it uses the dispose
operation before unregistering the instance.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                                                   | Type   | Default Value   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|-----------------|
| [`autodispose_unregistered_instances`](../../../api_reference/dds_pim/core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicy34autodispose_unregistered_instancesE) | `bool` | `true`          |

#### NOTE
This QoS Policy applies to [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It can be changed on enabled entities.
