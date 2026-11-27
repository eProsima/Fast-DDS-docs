<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-topic"></a>

<a id="topic"></a>

# Topic

A [`Topic`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) is a specialization of the broader concept of [TopicDescription](../topicDescription/topicDescription.md#dds-layer-topic-topicdescription).
A Topic represents a single data flow between [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher)
and [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber), providing:

* The name to identify the data flow.
* The data type that is transmitted on that flow.
* The QoS values related to the data itself.

The behavior of the Topic can be modified with the QoS values
specified on [TopicQos](#dds-layer-topic-topicqos).
The QoS values can be set at the creation of the Topic,
or modified later with the [`Topic::set_qos()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic7set_qosERK8TopicQos) member function.

Like other Entities, Topic accepts a Listener that will be notified of
status changes on the Topic.

Please refer to [Creating a Topic](createTopic.md#dds-layer-topic-creation) for more information about how to create a [`Topic`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE).

<a id="dds-layer-topic-topicqos"></a>

<a id="topicqos"></a>

## TopicQos

[`TopicQos`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) controls the behavior of the Topic.
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                                   | Accessor                                                                                                                                | Mutable   |
|---------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [TopicDataQosPolicy](../../core/policy/standardQosPolicies.md#topicdataqospolicy)                 | [`topic_data()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos10topic_dataEv)                 | Yes       |
| [DurabilityQosPolicy](../../core/policy/standardQosPolicies.md#durabilityqospolicy)               | [`durability()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos10durabilityEv)                 | Yes       |
| [DurabilityServiceQosPolicy](../../core/policy/standardQosPolicies.md#durabilityserviceqospolicy) | [`durability_service()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos18durability_serviceEv) | Yes       |
| [DeadlineQosPolicy](../../core/policy/standardQosPolicies.md#deadlineqospolicy)                   | [`deadline()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos8deadlineEv)                      | Yes       |
| [LatencyBudgetQosPolicy](../../core/policy/standardQosPolicies.md#latencybudgetqospolicy)         | [`latency_budget()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos14latency_budgetEv)         | Yes       |
| [LivelinessQosPolicy](../../core/policy/standardQosPolicies.md#livelinessqospolicy)               | [`liveliness()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos10livelinessEv)                 | Yes       |
| [ReliabilityQosPolicy](../../core/policy/standardQosPolicies.md#reliabilityqospolicy)             | [`reliability()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos11reliabilityEv)               | Yes       |
| [DestinationOrderQosPolicy](../../core/policy/standardQosPolicies.md#destinationorderqospolicy)   | [`destination_order()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos17destination_orderEv)   | Yes       |
| [HistoryQosPolicy](../../core/policy/standardQosPolicies.md#historyqospolicy)                     | [`history()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos7historyEv)                        | Yes       |
| [ResourceLimitsQosPolicy](../../core/policy/standardQosPolicies.md#resourcelimitsqospolicy)       | [`resource_limits()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos15resource_limitsEv)       | Yes       |
| [TransportPriorityQosPolicy](../../core/policy/standardQosPolicies.md#transportpriorityqospolicy) | [`transport_priority()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos18transport_priorityEv) | Yes       |
| [LifespanQosPolicy](../../core/policy/standardQosPolicies.md#lifespanqospolicy)                   | [`lifespan()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos8lifespanEv)                      | Yes       |
| [OwnershipQosPolicy](../../core/policy/standardQosPolicies.md#ownershipqospolicy)                 | [`ownership()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos9ownershipEv)                    | Yes       |
| [DataRepresentationQosPolicy](../../core/policy/xtypesExtensions.md#datarepresentationqospolicy)  | [`representation()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4NK8eprosima7fastdds3dds8TopicQos14representationEv)         | Yes       |

Refer to the detailed description of each QosPolicy-api class for more information about their usage and
default values.

The QoS value of a previously created Topic can be modified using the
[`Topic::set_qos()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic7set_qosERK8TopicQos) member function.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Topic with default TopicQos
Topic* topic =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
TopicQos qos = topic->get_qos();

// Modify QoS attributes
// (...)

// Assign the new Qos to the object
topic->set_qos(qos);
```

<a id="dds-layer-defaulttopicqos"></a>

<a id="default-topicqos"></a>

### Default TopicQos

The default [TopicQos](#dds-layer-topic-topicqos) refers to the value returned by the
[`get_default_topic_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant21get_default_topic_qosEv) member function on the [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) instance.
The special value `TOPIC_QOS_DEFAULT` can be used as QoS argument on [`create_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12create_topicERKNSt6stringERKNSt6stringERK8TopicQosR12ReturnCode_tP13TopicListenerRK10StatusMask)
or [`Topic::set_qos()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic7set_qosERK8TopicQos) member functions to indicate that the current default TopicQos
should be used.

When the system starts, the default TopicQos is equivalent to the default constructed
value [`TopicQos()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQos8TopicQosEv).
The default TopicQos can be modified at any time using the
[`get_default_topic_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant21get_default_topic_qosEv) member function on the DomainParticipant instance.
Modifying the default TopicQos will not affect already existing Topic
instances.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
TopicQos qos_type1 = participant->get_default_topic_qos();

// Modify QoS attributes
// (...)

// Set as the new default TopicQos
if (participant->set_default_topic_qos(qos_type1) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Topic with the new default TopicQos.
Topic* topic_with_qos_type1 =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
if (nullptr == topic_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
TopicQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default TopicQos
if (participant->set_default_topic_qos(qos_type2) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Topic with the new default TopicQos.
Topic* topic_with_qos_type2 =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
if (nullptr == topic_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default TopicQos to the original default constructed values
if (participant->set_default_topic_qos(TOPIC_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (participant->set_default_topic_qos(TopicQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`get_default_topic_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant21get_default_topic_qosEv) member function also accepts the value `TOPIC_QOS_DEFAULT`
as input argument.
This will reset the current default TopicQos to default constructed
value [`TopicQos()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQos8TopicQosEv).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a custom TopicQos
TopicQos custom_qos;

// Modify QoS attributes
// (...)

// Create a topic with a custom TopicQos
Topic* topic = participant->create_topic("TopicName", "DataTypeName", custom_qos);
if (nullptr == topic)
{
    // Error
    return;
}

// Set the QoS on the topic to the default
if (topic->set_qos(TOPIC_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (topic->set_qos(participant->get_default_topic_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `TOPIC_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12create_topicERKNSt6stringERKNSt6stringERK8TopicQosR12ReturnCode_tP13TopicListenerRK10StatusMask) and [`Topic::set_qos()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic7set_qosERK8TopicQos) it refers to the default TopicQos
  as returned by [`get_default_topic_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant21get_default_topic_qosEv).
* On [`get_default_topic_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant21get_default_topic_qosEv) it refers to the default constructed [`TopicQos()`](../../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQos8TopicQosEv).
