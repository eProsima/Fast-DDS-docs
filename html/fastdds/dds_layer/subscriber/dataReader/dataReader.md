<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-datareader"></a>

<a id="datareader"></a>

# DataReader

A [`DataReader`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) is attached to exactly one
[Subscriber](../subscriber/subscriber.md#dds-layer-subscriber-subscriber) that acts as a factory for it.
Additionally, each DataReader is bound to a single
[Topic](../../topic/topic/topic.md#dds-layer-topic-topic) since its creation.
This Topic must exist prior to the creation of the
DataReader,
and must be bound to the data type that the DataReader wants to publish.

The effect of creating a new DataReader in a
Subscriber for a specific
Topic is to initiate a new subscription with the name and data type described
by the Topic.

Once the DataReader is created, the application will be informed
when changes in the data value are received from remote publications.
These changes can then be retrieved using the [`DataReader::read_next_sample()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16read_next_sampleEPvP10SampleInfo) or [`DataReader::take_next_sample()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16take_next_sampleEPvP10SampleInfo)
member functions of the DataReader.

<a id="dds-layer-subscriber-datareaderqos"></a>

<a id="datareaderqos"></a>

## DataReaderQos

[`DataReaderQoS`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) controls the behavior of the [DataReader](#dds-layer-subscriber-datareader).
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                                                  | Accessor/Mutator                                                                                                                                               | Mutable   |
|------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [DurabilityQosPolicy](../../core/policy/standardQosPolicies.md#durabilityqospolicy)                              | [`durability()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos10durabilityEv)                         | No        |
| [DurabilityServiceQosPolicy](../../core/policy/standardQosPolicies.md#durabilityserviceqospolicy)                | [`durability_service()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos18durability_serviceEv)         | Yes       |
| [DeadlineQosPolicy](../../core/policy/standardQosPolicies.md#deadlineqospolicy)                                  | [`deadline()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos8deadlineEv)                              | Yes       |
| [LatencyBudgetQosPolicy](../../core/policy/standardQosPolicies.md#latencybudgetqospolicy)                        | [`latency_budget()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos14latency_budgetEv)                 | Yes       |
| [LivelinessQosPolicy](../../core/policy/standardQosPolicies.md#livelinessqospolicy)                              | [`liveliness()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos10livelinessEv)                         | No        |
| [ReliabilityQosPolicy](../../core/policy/standardQosPolicies.md#reliabilityqospolicy)                            | [`reliability()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos11reliabilityEv)                       | No (\*)   |
| [DestinationOrderQosPolicy](../../core/policy/standardQosPolicies.md#destinationorderqospolicy)                  | [`destination_order()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos17destination_orderEv)           | No        |
| [HistoryQosPolicy](../../core/policy/standardQosPolicies.md#historyqospolicy)                                    | [`history()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos7historyEv)                                | No        |
| [ResourceLimitsQosPolicy](../../core/policy/standardQosPolicies.md#resourcelimitsqospolicy)                      | [`resource_limits()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos15resource_limitsEv)               | No        |
| [LifespanQosPolicy](../../core/policy/standardQosPolicies.md#lifespanqospolicy)                                  | [`lifespan()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos8lifespanEv)                              | Yes       |
| [UserDataQosPolicy](../../core/policy/standardQosPolicies.md#userdataqospolicy)                                  | [`user_data()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos9user_dataEv)                            | Yes       |
| [OwnershipQosPolicy](../../core/policy/standardQosPolicies.md#ownershipqospolicy)                                | [`ownership()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos9ownershipEv)                            | No        |
| [PropertyPolicyQos](../../core/policy/eprosimaExtensions.md#propertypolicyqos)                                   | [`properties()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos10propertiesEv)                         | Yes       |
| [RTPSEndpointQos](../../core/policy/eprosimaExtensions.md#rtpsendpointqos)                                       | [`endpoint()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos8endpointEv)                              | Yes       |
| [ReaderResourceLimitsQos](../../core/policy/eprosimaExtensions.md#readerresourcelimitsqos)                       | [`reader_resource_limits()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos22reader_resource_limitsEv) | Yes       |
| [RTPSEndpointTimeBasedFilterQosPolicy](../../core/policy/standardQosPolicies.md#timebasedfilterqospolicy)        | [`time_based_filter()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos17time_based_filterEv)           | Yes       |
| [ReaderDataLifecycleQosPolicy](../../core/policy/standardQosPolicies.md#readerdatalifecycleqospolicy)            | [`reader_data_lifecycle()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos21reader_data_lifecycleEv)   | Yes       |
| [RTPSReliableReaderQos](../../core/policy/eprosimaExtensions.md#rtpsreliablereaderqos)                           | [`reliable_reader_qos()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos19reliable_reader_qosEv)       | Yes (\*)  |
| [TypeConsistencyEnforcementQosPolicy](../../core/policy/xtypesExtensions.md#typeconsistencyenforcementqospolicy) | [`type_consistency()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos16type_consistencyEv)             | Yes       |
| [DataRepresentationQosPolicy](../../core/policy/xtypesExtensions.md#datarepresentationqospolicy)                 | [`representation()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4NK8eprosima7fastdds3dds13DataReaderQos14representationEv)                | Yes       |
| [DataSharingQosPolicy](../../core/policy/eprosimaExtensions.md#datasharingqospolicy)                             | [`data_sharing()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos12data_sharingEv)                     | No        |
| `boolean`                                                                                                        | [`expects_inline_qos()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4NK8eprosima7fastdds3dds13DataReaderQos18expects_inline_qosEv)        | Yes       |

The following non-consolidated property-assigned QoS apply to DataReaders:

| Property name   | Non-consolidated QoS                                                                                   |
|-----------------|--------------------------------------------------------------------------------------------------------|
| partitions      | [Endpoint Partitions](../../../property_policies/non_consolidated_qos.md#property-policies-partitions) |

Refer to the detailed description of each [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) class for more information about their usage and
default values.

#### NOTE
Reliability kind (whether the publication is reliable or best effort) is not mutable.
However, the [`max_blocking_time`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) data member of [`ReliabilityQosPolicy`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) can be modified
any time.

#### NOTE
Not all data members of RTPSReliableReaderQos are mutable, please refer to [RTPSReliableReaderQos](../../core/policy/eprosimaExtensions.md#rtpsreliablereaderqos)
for more information.

The QoS value of a previously created DataReader can be modified using the
[`DataReader::set_qos()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader7set_qosERK13DataReaderQos) member function.

```c++
// Create a DataReader with default DataReaderQos
DataReader* data_reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DataReaderQos qos = data_reader->get_qos();

// Modify QoS attributes
// (...)

// Assign the new Qos to the object
data_reader->set_qos(qos);
```

<a id="dds-layer-defaultdatareaderqos"></a>

<a id="default-datareaderqos"></a>

### Default DataReaderQos

The default DataReaderQos refers to the value returned by the
[`get_default_datareader_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4NK8eprosima7fastdds3dds10Subscriber26get_default_datareader_qosEv) member function on the
[Subscriber](../subscriber/subscriber.md#dds-layer-subscriber-subscriber) instance.
The special value `DATAREADER_QOS_DEFAULT` can be used as QoS argument on
[`create_datareader()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber17create_datareaderEP16TopicDescriptionRK13DataReaderQosP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) or
[`DataReader::set_qos()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader7set_qosERK13DataReaderQos) member functions to indicate that the current default
DataReaderQos should be used.

When the system starts, the default DataReaderQos is equivalent to
the default constructed value [`DataReaderQos()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos13DataReaderQosEv).
The default DataReaderQos can be modified at any time using the
[`set_default_datareader_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber26set_default_datareader_qosERK13DataReaderQos) member function on the
Subscriber instance.
Modifying the default DataReaderQos will not affect already existing
[DataReader](#dds-layer-subscriber-datareader) instances.

```c++
// Get the current QoS or create a new one from scratch
DataReaderQos qos_type1 = subscriber->get_default_datareader_qos();

// Modify QoS attributes
// (...)

// Set as the new default DataReaderQos
if (subscriber->set_default_datareader_qos(qos_type1) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DataReader with the new default DataReaderQos.
DataReader* data_reader_with_qos_type1 =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DataReaderQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default DataReaderQos
if (subscriber->set_default_datareader_qos(qos_type2) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DataReader with the new default DataReaderQos.
DataReader* data_reader_with_qos_type2 =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default DataReaderQos to the original default constructed values
if (subscriber->set_default_datareader_qos(DATAREADER_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (subscriber->set_default_datareader_qos(DataReaderQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`set_default_datareader_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber26set_default_datareader_qosERK13DataReaderQos) member function also accepts
the special value `DATAREADER_QOS_DEFAULT` as input argument.
This will reset the current default DataReaderQos to default constructed
value [`DataReaderQos()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos13DataReaderQosEv).

```c++
// Create a custom DataReaderQos
DataReaderQos custom_qos;

// Modify QoS attributes
// (...)

// Create a DataWriter with a custom DataReaderQos
DataReader* data_reader = subscriber->create_datareader(topic, custom_qos);
if (nullptr == data_reader)
{
    // Error
    return;
}

// Set the QoS on the DataWriter to the default
if (data_reader->set_qos(DATAREADER_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (data_reader->set_qos(subscriber->get_default_datareader_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `DATAREADER_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_datareader()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber17create_datareaderEP16TopicDescriptionRK13DataReaderQosP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE)
  and [`DataReader::set_qos()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader7set_qosERK13DataReaderQos) it refers to the default
  DataReaderQos as returned by
  [`get_default_datareader_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4NK8eprosima7fastdds3dds10Subscriber26get_default_datareader_qosEv).
* On [`set_default_datareader_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber26set_default_datareader_qosERK13DataReaderQos) it refers
  to the default constructed [`DataReaderQos()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos13DataReaderQosEv).

<a id="dds-layer-subscriber-datareader-type-support-context"></a>

<a id="type-support-context"></a>

## Type support context

A DataReader can be configured with a [type support context](../../topic/typeSupport/typeSupport.md#dds-layer-topic-type-support-context)
that is forwarded to the [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) callbacks on every read operation.
This allows the type implementation to rely on per-reader information (e.g., bounded sizes) without
resorting to global state.

The context must be set **before** enabling the DataReader using
[`DataReader::set_type_support_context()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE).
Calling this method on an already-enabled DataReader returns `RETCODE_ILLEGAL_OPERATION`.

```c++
// User-defined context struct carrying per-reader configuration.
struct MyContext : public TopicDataType::Context
{
    uint32_t string_max_length = 0;
    uint32_t sequence_max_length = 0;
};

// Create a DataReader (initially disabled, e.g. via participant QoS).
DataReaderQos reader_qos = DATAREADER_QOS_DEFAULT;
DataReader* reader = subscriber->create_datareader(topic, reader_qos);

// Create a context carrying per-reader configuration.
auto context = std::make_shared<MyContext>();
context->string_max_length = 512;
context->sequence_max_length = 1024;

// Set it before enabling the DataReader.
ReturnCode_t ret = reader->set_type_support_context(context);
assert(ret == RETCODE_OK);

// Then enable the reader (e.g., by enabling its DomainParticipant).
```
