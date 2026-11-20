<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-datawriter"></a>

<a id="datawriter"></a>

# DataWriter

A [`DataWriter`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) is attached to exactly one [Publisher](../publisher/publisher.md#dds-layer-publisher-publisher) that acts as a factory for it.
Additionally, each DataWriter is bound to a single [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) since its creation.
This Topic must exist prior to the creation of the DataWriter,
and must be bound to the data type that the DataWriter wants to publish.

The effect of creating a new DataWriter in a Publisher for a specific
Topic is to initiate a new publication with the name and data type described
by the Topic.

Once the DataWriter is created, the application can inform of changes in the data value using the
[`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) member function on the DataWriter.
These changes will be transmitted to all subscriptions matched with this publication.

<a id="dds-layer-publisher-datawriterqos"></a>

<a id="datawriterqos"></a>

## DataWriterQos

[`DataWriterQos`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) controls the behavior of the DataWriter.
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                                       | Accessor/Mutator                                                                                                                                              | Mutable   |
|-------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [DurabilityQosPolicy](../../core/policy/standardQosPolicies.md#durabilityqospolicy)                   | [`durability()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos10durabilityEv)                         | No        |
| [DurabilityServiceQosPolicy](../../core/policy/standardQosPolicies.md#durabilityserviceqospolicy)     | [`durability_service()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos18durability_serviceEv)         | No        |
| [DeadlineQosPolicy](../../core/policy/standardQosPolicies.md#deadlineqospolicy)                       | [`deadline()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos8deadlineEv)                              | Yes       |
| [LatencyBudgetQosPolicy](../../core/policy/standardQosPolicies.md#latencybudgetqospolicy)             | [`latency_budget()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos14latency_budgetEv)                 | Yes       |
| [LivelinessQosPolicy](../../core/policy/standardQosPolicies.md#livelinessqospolicy)                   | [`liveliness()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos10livelinessEv)                         | No        |
| [ReliabilityQosPolicy](../../core/policy/standardQosPolicies.md#reliabilityqospolicy)                 | [`reliability()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos11reliabilityEv)                       | No (\*)   |
| [DestinationOrderQosPolicy](../../core/policy/standardQosPolicies.md#destinationorderqospolicy)       | [`destination_order()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos17destination_orderEv)           | No        |
| [HistoryQosPolicy](../../core/policy/standardQosPolicies.md#historyqospolicy)                         | [`history()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos7historyEv)                                | No        |
| [ResourceLimitsQosPolicy](../../core/policy/standardQosPolicies.md#resourcelimitsqospolicy)           | [`resource_limits()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos15resource_limitsEv)               | No        |
| [TransportPriorityQosPolicy](../../core/policy/standardQosPolicies.md#transportpriorityqospolicy)     | [`transport_priority()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos18transport_priorityEv)         | Yes       |
| [LifespanQosPolicy](../../core/policy/standardQosPolicies.md#lifespanqospolicy)                       | [`lifespan()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos8lifespanEv)                              | Yes       |
| [UserDataQosPolicy](../../core/policy/standardQosPolicies.md#userdataqospolicy)                       | [`user_data()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos9user_dataEv)                            | Yes       |
| [OwnershipQosPolicy](../../core/policy/standardQosPolicies.md#ownershipqospolicy)                     | [`ownership()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos9ownershipEv)                            | No        |
| [OwnershipStrengthQosPolicy](../../core/policy/standardQosPolicies.md#ownershipstrengthqospolicy)     | [`ownership_strength()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos18ownership_strengthEv)         | Yes       |
| [WriterDataLifecycleQosPolicy](../../core/policy/standardQosPolicies.md#writerdatalifecycleqospolicy) | [`writer_data_lifecycle()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos21writer_data_lifecycleEv)   | Yes       |
| [PublishModeQosPolicy](../../core/policy/eprosimaExtensions.md#publishmodeqospolicy)                  | [`publish_mode()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos12publish_modeEv)                     | No        |
| [DataRepresentationQosPolicy](../../core/policy/xtypesExtensions.md#datarepresentationqospolicy)      | [`representation()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos14representationEv)                 | No        |
| [PropertyPolicyQos](../../core/policy/eprosimaExtensions.md#propertypolicyqos)                        | [`properties()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos10propertiesEv)                         | No        |
| [RTPSReliableWriterQos](../../core/policy/eprosimaExtensions.md#rtpsreliablewriterqos)                | [`reliable_writer_qos()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos19reliable_writer_qosEv)       | Yes (\*)  |
| [RTPSEndpointQos](../../core/policy/eprosimaExtensions.md#rtpsendpointqos)                            | [`endpoint()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos8endpointEv)                              | No        |
| [WriterResourceLimitsQos](../../core/policy/eprosimaExtensions.md#writerresourcelimitsqos)            | [`writer_resource_limits()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos22writer_resource_limitsEv) | No        |
| [DataSharingQosPolicy](../../core/policy/eprosimaExtensions.md#datasharingqospolicy)                  | [`data_sharing()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos12data_sharingEv)                     | No        |

The following non-consolidated property-assigned QoS apply to DataWriters:

| Property name     | Non-consolidated QoS                                                                                            |
|-------------------|-----------------------------------------------------------------------------------------------------------------|
| fastdds.push_mode | [DataWriter operating mode QoS Policy](../../../property_policies/non_consolidated_qos.md#push-mode-qos-policy) |
| partitions        | [Endpoint Partitions](../../../property_policies/non_consolidated_qos.md#property-policies-partitions)          |

Refer to the detailed description of each [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) class for more information about their usage and
default values.

#### NOTE
Reliability kind (whether the publication is reliable or best effort) is not mutable.
However, the `max_blocking_time` data member of [ReliabilityQosPolicy](../../core/policy/standardQosPolicies.md#reliabilityqospolicy) can be modified any time.

#### NOTE
Not all data members of RTPSReliableWriterQos are mutable, please refer to [RTPSReliableWriterQos](../../core/policy/eprosimaExtensions.md#rtpsreliablewriterqos)
for more information.

The QoS value of a previously created DataWriter can be modified using the
[`DataWriter::set_qos()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter7set_qosERK13DataWriterQos) member function.

```c++
// Create a DataWriter with default DataWriterQos
DataWriter* data_writer =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DataWriterQos qos = data_writer->get_qos();

// Modify QoS attributes
// (...)

// Assign the new Qos to the object
data_writer->set_qos(qos);
```

<a id="dds-layer-defaultdatawriterqos"></a>

<a id="default-datawriterqos"></a>

### Default DataWriterQos

The default [DataWriterQos](#dds-layer-publisher-datawriterqos) refers to the value returned by the
[`get_default_datawriter_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4NK8eprosima7fastdds3dds9Publisher26get_default_datawriter_qosEv) member function on the Publisher instance.
The special value `DATAWRITER_QOS_DEFAULT` can be used as QoS argument on [`create_datawriter()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher17create_datawriterEP5TopicRK13DataWriterQosP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE)
or [`DataWriter::set_qos()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter7set_qosERK13DataWriterQos) member functions to indicate that the current default
DataWriterQos should be used.

When the system starts, the default DataWriterQos is equivalent to the default constructed
value [`DataWriterQos()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos13DataWriterQosEv).
The default DataWriterQos can be modified at any time using the
[`set_default_datawriter_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher26set_default_datawriter_qosERK13DataWriterQos) member function on the Publisher instance.
Modifying the default DataWriterQos will not affect already existing
DataWriter instances.

```c++
// Get the current QoS or create a new one from scratch
DataWriterQos qos_type1 = publisher->get_default_datawriter_qos();

// Modify QoS attributes
// (...)

// Set as the new default DataWriterQos
if (publisher->set_default_datawriter_qos(qos_type1) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DataWriter with the new default DataWriterQos.
DataWriter* data_writer_with_qos_type1 =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DataWriterQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default DataWriterQos
if (publisher->set_default_datawriter_qos(qos_type2) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DataWriter with the new default DataWriterQos.
DataWriter* data_writer_with_qos_type2 =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default DataWriterQos to the original default constructed values
if (publisher->set_default_datawriter_qos(DATAWRITER_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (publisher->set_default_datawriter_qos(DataWriterQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`set_default_datawriter_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher26set_default_datawriter_qosERK13DataWriterQos) member function also accepts the special value
`DATAWRITER_QOS_DEFAULT` as input argument.
This will reset the current default DataWriterQos to default constructed
value [`DataWriterQos()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos13DataWriterQosEv).

```c++
// Create a custom DataWriterQos
DataWriterQos custom_qos;

// Modify QoS attributes
// (...)

// Create a DataWriter with a custom DataWriterQos
DataWriter* data_writer = publisher->create_datawriter(topic, custom_qos);
if (nullptr == data_writer)
{
    // Error
    return;
}

// Set the QoS on the DataWriter to the default
if (data_writer->set_qos(DATAWRITER_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (data_writer->set_qos(publisher->get_default_datawriter_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `DATAWRITER_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_datawriter()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher17create_datawriterEP5TopicRK13DataWriterQosP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) and [`DataWriter::set_qos()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter7set_qosERK13DataWriterQos) it refers to the default DataWriterQos
  as returned by [`get_default_datawriter_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4NK8eprosima7fastdds3dds9Publisher26get_default_datawriter_qosEv).
* On [`set_default_datawriter_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher26set_default_datawriter_qosERK13DataWriterQos) it refers to the default constructed
  [`DataWriterQos()`](../../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos13DataWriterQosEv).

<a id="dds-layer-publisher-datawriter-type-support-context"></a>

<a id="type-support-context"></a>

## Type support context

A DataWriter can be configured with a [type support context](../../topic/typeSupport/typeSupport.md#dds-layer-topic-type-support-context)
that is forwarded to the [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) callbacks on every write operation.
This allows the type implementation to rely on per-writer information (e.g., bounded sizes)

The context must be set **before** enabling the DataWriter using
[`DataWriter::set_type_support_context()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE).
Calling this method on an already-enabled DataWriter returns `RETCODE_ILLEGAL_OPERATION`.

```c++
// User-defined context struct carrying per-writer configuration.
struct MyContext : public TopicDataType::Context
{
    uint32_t string_max_length = 0;
    uint32_t sequence_max_length = 0;
};

// Create a DataWriter (initially disabled, e.g. via participant QoS).
DataWriterQos writer_qos = DATAWRITER_QOS_DEFAULT;
DataWriter* writer = publisher->create_datawriter(topic, writer_qos);

// Create a context carrying per-writer configuration.
auto context = std::make_shared<MyContext>();
context->string_max_length = 512;
context->sequence_max_length = 1024;

// Set it before enabling the DataWriter.
ReturnCode_t ret = writer->set_type_support_context(context);
assert(ret == RETCODE_OK);

// Then enable the writer (e.g., by enabling its DomainParticipant).
```
