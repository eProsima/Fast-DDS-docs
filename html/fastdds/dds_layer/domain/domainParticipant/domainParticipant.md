<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domainparticipant"></a>

<a id="domainparticipant"></a>

# DomainParticipant

A [DomainParticipant](#dds-layer-domainparticipant) is the entry point of the application to a domain.
Every DomainParticipant is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher), [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber)
and [Topic](../../topic/topic/topic.md#dds-layer-topic-topic).

The behavior of the DomainParticipant can be modified with the QoS values
specified on DomainParticipantQos.
The QoS values can be set at the creation of the DomainParticipant,
or modified later with [`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) member function.

As an Entity, DomainParticipant accepts a [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener)
that will be notified of status changes on the DomainParticipant instance.

<a id="dds-layer-domainparticipantqos"></a>

<a id="domainparticipantqos"></a>

## DomainParticipantQos

[`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) controls the behavior of the DomainParticipant.
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                                      | Accessor/Mutator                                                                                                                                                                                                                                                                                                                                                                | Mutable   |
|------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [UserDataQosPolicy](../../core/policy/standardQosPolicies.md#userdataqospolicy)                      | [`user_data()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos9user_dataEv)                                                                                                                                                                                                                                  | Yes       |
| [EntityFactoryQosPolicy](../../core/policy/standardQosPolicies.md#entityfactoryqospolicy)            | [`entity_factory()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos14entity_factoryEv)                                                                                                                                                                                                                       | Yes       |
| [ParticipantResourceLimitsQos](../../core/policy/eprosimaExtensions.md#participantresourcelimitsqos) | [`allocation()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10allocationEv)                                                                                                                                                                                                                               | No        |
| [PropertyPolicyQos](../../core/policy/eprosimaExtensions.md#propertypolicyqos)                       | [`properties()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)                                                                                                                                                                                                                               | No        |
| [WireProtocolConfigQos](../../core/policy/eprosimaExtensions.md#wireprotocolconfigqos)               | [`wire_protocol()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos13wire_protocolEv)                                                                                                                                                                                                                         | No\*      |
| [TransportConfigQos](../../core/policy/eprosimaExtensions.md#transportconfigqos)                     | [`transport()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos9transportEv) and<br/>[`setup_transports()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos16setup_transportsEN4rtps17BuiltinTransportsERKN4rtps24BuiltinTransportsOptionsE) | No        |
| [FlowControllersQos](../../core/policy/eprosimaExtensions.md#flowcontrollersqos)                     | [`flow_controllers()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos16flow_controllersEv)                                                                                                                                                                                                                    | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)                          | [`builtin_controllers_sender_thread()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos33builtin_controllers_sender_threadEv)                                                                                                                                                                                  | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)                          | [`timed_events_thread()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos19timed_events_threadEv)                                                                                                                                                                                                              | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)                          | [`discovery_server_thread()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos23discovery_server_threadEv)                                                                                                                                                                                                      | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)                          | [`typelookup_service_thread()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos25typelookup_service_threadEv)                                                                                                                                                                                                  | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)                          | [`security_log_thread()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos19security_log_threadEv)                                                                                                                                                                                                              | No        |

#### IMPORTANT
The only mutable field in [WireProtocolConfigQos](../../core/policy/eprosimaExtensions.md#wireprotocolconfigqos) is [`m_DiscoveryServers`](../../../api_reference/rtps/attributes/DiscoverySettings.md#_CPPv4N8eprosima7fastdds4rtps17DiscoverySettings18m_DiscoveryServersE), which is contained in
[`discovery_config`](../../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes16discovery_configE) within [`builtin`](../../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) (see
[Modifying remote servers list at run time](../../../discovery/discovery_server.md#ds-modify-server-list)).

#### IMPORTANT
Upon the call to [`create_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask), if Fast DDS is compiled with statistics support
(enabled by default, see [CMake options](../../../../installation/configuration/cmake_options.md#cmake-options), and always in *Fast DDS* **[Pro]**), the internal
[`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) may differ from the input [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE)
(see [Statistics Module Settings](../../../property_policies/non_consolidated_qos.md#property-policies-statistics)).
This entails that applications willing to further modify the [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) after
[`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) creation should:

1. Retrieve the internal [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) by the means of [`DomainParticipant::get_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7get_qosER20DomainParticipantQos).
2. Perform the desired modifications.
3. Update the [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) by the means of [`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos).

Refer to the detailed description of each QosPolicy class for more information about their usage and
default values.

The QoS value of a previously created DomainParticipant can be modified using the
[`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) member function.
Trying to modify an immutable QosPolicy on an already enabled DomainParticipant
will result on an error.
In such case, no changes will be applied and the DomainParticipant will keep its
previous DomainParticipantQos.

```c++
// Create a DomainParticipant with default DomainParticipantQos
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DomainParticipantQos qos = participant->get_qos();

// Modify QoS attributes
qos.entity_factory().autoenable_created_entities = false;

// Assign the new Qos to the object
participant->set_qos(qos);
```

<a id="dds-layer-defaultdomainparticipantqos"></a>

<a id="default-domainparticipantqos"></a>

### Default DomainParticipantQos

The default DomainParticipantQos refers to the value returned by the
[`get_default_participant_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory27get_default_participant_qosER20DomainParticipantQos) member function on the
[DomainParticipantFactory](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton.
The special value `PARTICIPANT_QOS_DEFAULT` can be used as QoS argument on
[`create_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask)
or [`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) member functions to indicate that the current default
DomainParticipantQos should be used.

When the system starts, the default DomainParticipantQos is equivalent to the default constructed
value [`DomainParticipantQos()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos20DomainParticipantQosEv).
The default DomainParticipantQos can be modified at any time using the
[`set_default_participant_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory27set_default_participant_qosERK20DomainParticipantQos)
member function on the DomainParticipantFactory singleton.
Modifying the default DomainParticipantQos will not affect already existing
DomainParticipant instances.

```c++
// Get the current QoS or create a new one from scratch
DomainParticipantQos qos_type1 = DomainParticipantFactory::get_instance()->get_default_participant_qos();

// Modify QoS attributes
// (...)

// Set as the new default TopicQos
if (DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type1) !=
        RETCODE_OK)
{
    // Error
    return;
}

// Create a DomainParticipant with the new default DomainParticipantQos.
DomainParticipant* participant_with_qos_type1 =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant_with_qos_type1)
{
    // Error
    return;
}

// Get the current QoS or create a new one from scratch
DomainParticipantQos qos_type2;

// Modify QoS attributes
// (...)

// Set as the new default TopicQos
if (DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type2) !=
        RETCODE_OK)
{
    // Error
    return;
}

// Create a Topic with the new default TopicQos.
DomainParticipant* participant_with_qos_type2 =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant_with_qos_type2)
{
    // Error
    return;
}

// Resetting the default DomainParticipantQos to the original default constructed values
if (DomainParticipantFactory::get_instance()->set_default_participant_qos(PARTICIPANT_QOS_DEFAULT)
        != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following
if (DomainParticipantFactory::get_instance()->set_default_participant_qos(DomainParticipantQos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

[`set_default_participant_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory27set_default_participant_qosERK20DomainParticipantQos)
member function also accepts the value `PARTICIPANT_QOS_DEFAULT`
as input argument.
This will reset the current default DomainParticipantQos to the default constructed value
[`DomainParticipantQos()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos20DomainParticipantQosEv).

```c++
// Create a custom DomainParticipantQos
DomainParticipantQos custom_qos;

// Modify QoS attributes
// (...)

// Create a DomainParticipant with a custom DomainParticipantQos

DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, custom_qos);
if (nullptr == participant)
{
    // Error
    return;
}

// Set the QoS on the participant to the default
if (participant->set_qos(PARTICIPANT_QOS_DEFAULT) != RETCODE_OK)
{
    // Error
    return;
}

// The previous instruction is equivalent to the following:
if (participant->set_qos(DomainParticipantFactory::get_instance()->get_default_participant_qos())
        != RETCODE_OK)
{
    // Error
    return;
}
```

#### NOTE
The value `PARTICIPANT_QOS_DEFAULT` has different meaning depending on where it is used:

* On [`create_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask) and [`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) it refers to the
  default DomainParticipantQos as returned by [`get_default_participant_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory27get_default_participant_qosER20DomainParticipantQos).
* On [`set_default_participant_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory27set_default_participant_qosERK20DomainParticipantQos) it refers to the default constructed
  [`DomainParticipantQos()`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos20DomainParticipantQosEv).

<a id="dds-layer-domainparticipantextendedqos"></a>

<a id="domainparticipantextendedqos"></a>

## DomainParticipantExtendedQos

[`DomainParticipantExtendedQos`](../../../api_reference/dds_pim/domain/domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) is an extension of [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) that includes both
the `DomainId` and the [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) objects of a DomainParticipant.
This class is useful for simplifying the creation and configuration of a DomainParticipant,
as it allows specifying all necessary settings in a single object.

This [`DomainParticipantExtendedQos`](../../../api_reference/dds_pim/domain/domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) can be obtained from the loaded profiles with
[`get_participant_extended_qos_from_profile()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory41get_participant_extended_qos_from_profileERKNSt6stringER28DomainParticipantExtendedQos) and then create [`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE)
with those [`DomainParticipantExtendedQos`](../../../api_reference/dds_pim/domain/domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE).
The QoS value of a previously created DomainParticipant can be modified similarly as when creating
the [`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) from [`DomainParticipantQos`](../../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE).

```c++
// Create a DomainParticipant with DomainParticipantExtendedQos from profile
DomainParticipantExtendedQos profile_extended_qos;
DomainParticipantFactory::get_instance()->get_participant_extended_qos_from_profile("participant_profile",
        profile_extended_qos);

DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(profile_extended_qos);
if (nullptr == participant)
{
    // Error
    return;
}
```

It is also possible to fill a [`DomainParticipantExtendedQos`](../../../api_reference/dds_pim/domain/domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) object directly from a raw XML string without the need
to previously load any profile.
Please refer to [Get QoS from raw XML profiles](../../../xml_configuration/making_xml_profiles.md#loading-raw-xml-profiles) section for more information.
