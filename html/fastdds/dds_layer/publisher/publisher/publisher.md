<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-publisher"></a>

<a id="publisher"></a>

# Publisher

The [Publisher](#dds-layer-publisher-publisher) acts on behalf of one or several [DataWriter](../dataWriter/dataWriter.md#dds-layer-publisher-datawriter) objects
that belong to it.
It serves as a container that allows grouping different DataWriter objects under
a common configuration given by the [PublisherQos](#dds-layer-publisher-publisherqos) of the Publisher.

DataWriter objects that belong to the same Publisher do not have any other
relation among each other beyond the PublisherQos of the Publisher and act independently
otherwise.
Specifically, a Publisher can host DataWriter objects for different [Topics](../../topic/topic.md#dds-layer-topic)
and data types.

<a id="dds-layer-publisher-publisherqos"></a>

<a id="publisherqos"></a>

## PublisherQos

[`PublisherQos`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) controls the behavior of the [`Publisher`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE).
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                           | Accessor/Mutator                                                                                                                             | Mutable   |
|-------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [PresentationQosPolicy](../../core/policy/standardQosPolicies.md#presentationqospolicy)   | [`presentation()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4NK8eprosima7fastdds3dds12PublisherQos12presentationEv)     | Yes       |
| [PartitionQosPolicy](../../core/policy/standardQosPolicies.md#partitionqospolicy)         | [`partition()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4NK8eprosima7fastdds3dds12PublisherQos9partitionEv)            | Yes       |
| [GroupDataQosPolicy](../../core/policy/standardQosPolicies.md#groupqospolicy)             | [`group_data()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4NK8eprosima7fastdds3dds12PublisherQos10group_dataEv)         | Yes       |
| [EntityFactoryQosPolicy](../../core/policy/standardQosPolicies.md#entityfactoryqospolicy) | [`entity_factory()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4NK8eprosima7fastdds3dds12PublisherQos14entity_factoryEv) | Yes       |

Refer to the detailed description of each [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) class for more information about their usage and
default values.

The QoS value of a previously created Publisher can be modified using the
[`Publisher::set_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher7set_qosERK12PublisherQos) member function.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Publisher with default PublisherQos
Publisher* publisher =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
PublisherQos qos = publisher->get_qos();

// Modify QoS attributes
// (...)

// Assign the new Qos to the object
publisher->set_qos(qos);
```

<a id="dds-layer-defaultpublisherqos"></a>

<a id="default-publisherqos"></a>

### Default PublisherQos

The default [PublisherQos](#dds-layer-publisher-publisherqos) refers to the value returned by the
[`get_default_publisher_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant25get_default_publisher_qosEv) member function on the DomainParticipant instance.
The special value `PUBLISHER_QOS_DEFAULT` can be used as QoS argument on
[`create_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16create_publisherERK12PublisherQosR12ReturnCode_tP17PublisherListenerRK10StatusMask) or [`Publisher::set_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher7set_qosERK12PublisherQos) member functions to indicate that the current
default PublisherQos should be used.

When the system starts, the default PublisherQos is equivalent to the default constructed
value [`PublisherQos()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQos12PublisherQosEv).
The default PublisherQos can be modified at any time using the
[`set_default_publisher_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25set_default_publisher_qosERK12PublisherQos) member function on the [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) instance.
Modifying the default PublisherQos will not affect already existing
[Publisher](#dds-layer-publisher-publisher) instances.

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
PublisherQos qos_type1 = participant->get_default_publisher_qos();

// Modify QoS attributes
// (...)

// Set as the new default PublisherQos
if (participant->set_default_publisher_qos(qos_type1) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Publisher with the new default PublisherQos.
Publisher* publisher_with_qos_type1 =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
PublisherQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default PublisherQos
if (participant->set_default_publisher_qos(qos_type2) != RETCODE_OK)
{
    // Error
    return;
}

// Create a Publisher with the new default PublisherQos.
Publisher* publisher_with_qos_type2 =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default PublisherQos to the original default constructed values
if (participant->set_default_publisher_qos(PUBLISHER_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (participant->set_default_publisher_qos(PublisherQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`set_default_publisher_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25set_default_publisher_qosERK12PublisherQos) member function also accepts the special value
`PUBLISHER_QOS_DEFAULT` as input argument.
This will reset the current default PublisherQos to default constructed
value [`PublisherQos()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQos12PublisherQosEv).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a custom PublisherQos
PublisherQos custom_qos;

// Modify QoS attributes
// (...)

// Create a publisher with a custom PublisherQos
Publisher* publisher = participant->create_publisher(custom_qos);
if (nullptr == publisher)
{
    // Error
    return;
}

// Set the QoS on the publisher to the default
if (publisher->set_qos(PUBLISHER_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (publisher->set_qos(participant->get_default_publisher_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `PUBLISHER_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16create_publisherERK12PublisherQosR12ReturnCode_tP17PublisherListenerRK10StatusMask) and [`Publisher::set_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher7set_qosERK12PublisherQos) it refers to the default
  [PublisherQos](#dds-layer-publisher-publisherqos).
  as returned by [`get_default_publisher_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant25get_default_publisher_qosEv).
* On [`set_default_publisher_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25set_default_publisher_qosERK12PublisherQos) it refers to the default constructed
  [`PublisherQos()`](../../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQos12PublisherQosEv).
