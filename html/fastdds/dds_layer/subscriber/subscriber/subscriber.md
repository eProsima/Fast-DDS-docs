<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-subscriber"></a>

<a id="subscriber"></a>

# Subscriber

The [`Subscriber`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) acts on behalf of one or several [DataReader](../dataReader/dataReader.md#dds-layer-subscriber-datareader) objects
that belong to it.
It serves as a container that allows grouping different DataReader objects under
a common configuration given by the [SubscriberQos](#dds-layer-subscriber-subscriberqos) of the Subscriber.

DataReader objects that belong to the same Subscriber
do not have any other relation among each other beyond the [`SubscriberQos`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) of the Subscriber
and act independently otherwise.
Specifically, a Subscriber can host DataReader objects
for different topics and data types.

<a id="dds-layer-subscriber-subscriberqos"></a>

<a id="subscriberqos"></a>

## SubscriberQos

[`SubscriberQos`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) controls the behavior of the [Subscriber](#dds-layer-subscriber-subscriber).
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                           | Accessor/Mutator                                                                                                                                | Mutable   |
|-------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [PresentationQosPolicy](../../core/policy/standardQosPolicies.md#presentationqospolicy)   | [`presentation()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4NK8eprosima7fastdds3dds13SubscriberQos12presentationEv)     | Yes       |
| [PartitionQosPolicy](../../core/policy/standardQosPolicies.md#partitionqospolicy)         | [`partition()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4NK8eprosima7fastdds3dds13SubscriberQos9partitionEv)            | Yes       |
| [GroupDataQosPolicy](../../core/policy/standardQosPolicies.md#groupqospolicy)             | [`group_data()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4NK8eprosima7fastdds3dds13SubscriberQos10group_dataEv)         | Yes       |
| [EntityFactoryQosPolicy](../../core/policy/standardQosPolicies.md#entityfactoryqospolicy) | [`entity_factory()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4NK8eprosima7fastdds3dds13SubscriberQos14entity_factoryEv) | Yes       |

Refer to the detailed description of each [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) class for more information about their usage and
default values.

The QoS value of a previously created Subscriber can be modified using the
[`Subscriber::set_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber7set_qosERK13SubscriberQos) member function.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Subscriber with default SubscriberQos
Subscriber* subscriber =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
SubscriberQos qos = subscriber->get_qos();

// Modify QoS attributes
qos.entity_factory().autoenable_created_entities = false;

// Assign the new Qos to the object
subscriber->set_qos(qos);
```

<a id="dds-layer-defaultsubscriberqos"></a>

<a id="default-subscriberqos"></a>

### Default SubscriberQos

The default [SubscriberQos](#dds-layer-subscriber-subscriberqos) refers to the value returned by the
[`get_default_subscriber_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant26get_default_subscriber_qosEv) member function
on the [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) instance.
The special value `SUBSCRIBER_QOS_DEFAULT` can be used as QoS argument on
[`create_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17create_subscriberERK13SubscriberQosR12ReturnCode_tP18SubscriberListenerRK10StatusMask) or [`Subscriber::set_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber7set_qosERK13SubscriberQos)
member functions to indicate that the current default SubscriberQos
should be used.

When the system starts, the default SubscriberQos is equivalent to the default constructed
value [`SubscriberQos()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQos13SubscriberQosEv).
The default SubscriberQos can be modified at any time using the
[`set_default_subscriber_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant26set_default_subscriber_qosERK13SubscriberQos) member function on the
DomainParticipant instance.
Modifying the default SubscriberQos will not affect already existing
Subscriber instances.

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
SubscriberQos qos_type1 = participant->get_default_subscriber_qos();

// Modify QoS attributes
// (...)

// Set as the new default SubscriberQos
if (participant->set_default_subscriber_qos(qos_type1) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Subscriber with the new default SubscriberQos.
Subscriber* subscriber_with_qos_type1 =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
SubscriberQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default SubscriberQos
if (participant->set_default_subscriber_qos(qos_type2) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Subscriber with the new default SubscriberQos.
Subscriber* subscriber_with_qos_type2 =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default SubscriberQos to the original default constructed values
if (participant->set_default_subscriber_qos(SUBSCRIBER_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (participant->set_default_subscriber_qos(SubscriberQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`set_default_subscriber_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant26set_default_subscriber_qosERK13SubscriberQos) member function also accepts
the special value `SUBSCRIBER_QOS_DEFAULT` as input argument.
This will reset the current default SubscriberQos to default constructed
value [`SubscriberQos()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQos13SubscriberQosEv).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a custom SubscriberQos
SubscriberQos custom_qos;

// Modify QoS attributes
// (...)

// Create a subscriber with a custom SubscriberQos
Subscriber* subscriber = participant->create_subscriber(custom_qos);
if (nullptr == subscriber)
{
    // Error
    return;
}

// Set the QoS on the subscriber to the default
if (subscriber->set_qos(SUBSCRIBER_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (subscriber->set_qos(participant->get_default_subscriber_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `SUBSCRIBER_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17create_subscriberERK13SubscriberQosR12ReturnCode_tP18SubscriberListenerRK10StatusMask) and [`Subscriber::set_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber7set_qosERK13SubscriberQos) it refers to the default
  SubscriberQos as returned by [`get_default_subscriber_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant26get_default_subscriber_qosEv).
* On [`set_default_subscriber_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant26set_default_subscriber_qosERK13SubscriberQos) it refers to the default constructed
  [`SubscriberQos()`](../../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQos13SubscriberQosEv).
