<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-creation"></a>

<a id="creating-a-subscriber"></a>

# Creating a Subscriber

A [Subscriber](subscriber.md#dds-layer-subscriber-subscriber) always belongs to a [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
Creation of a Subscriber is done with the [`create_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17create_subscriberERK13SubscriberQosR12ReturnCode_tP18SubscriberListenerRK10StatusMask) member function on the
DomainParticipant instance, that acts as a factory for the Subscriber.

Mandatory arguments are:

* The [SubscriberQos](subscriber.md#dds-layer-subscriber-subscriberqos) describing the behavior of the Subscriber.
  If the provided value is `SUBSCRIBER_QOS_DEFAULT`,
  the value of the [Default SubscriberQos](subscriber.md#dds-layer-defaultsubscriberqos) is used.

Optional arguments are:

* A Listener derived from [SubscriberListener](../subscriberListener/subscriberListener.md#dds-layer-subscriber-subscriberlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the Subscriber.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  SubscriberListener.
  By default all events are enabled.

[`create_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17create_subscriberERK13SubscriberQosR12ReturnCode_tP18SubscriberListenerRK10StatusMask) will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Subscriber with default SubscriberQos and no Listener
// The value SUBSCRIBER_QOS_DEFAULT is used to denote the default QoS.
Subscriber* subscriber_with_default_qos =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber_with_default_qos)
{
    // Error
    return;
}

// A custom SubscriberQos can be provided to the creation method
SubscriberQos custom_qos;

// Modify QoS attributes
// (...)

Subscriber* subscriber_with_custom_qos =
        participant->create_subscriber(custom_qos);
if (nullptr == subscriber_with_custom_qos)
{
    // Error
    return;
}

// Create a Subscriber with default QoS and a custom Listener.
// CustomSubscriberListener inherits from SubscriberListener.
// The value SUBSCRIBER_QOS_DEFAULT is used to denote the default QoS.
CustomSubscriberListener custom_listener;
Subscriber* subscriber_with_default_qos_and_custom_listener =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, &custom_listener);
if (nullptr == subscriber_with_default_qos_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-subscriber-creation-profile"></a>

<a id="profile-based-creation-of-a-subscriber"></a>

## Profile based creation of a Subscriber

Instead of using a SubscriberQos, the name of a profile
can be used to create a Subscriber with the [`create_subscriber_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant30create_subscriber_with_profileERKNSt6stringER12ReturnCode_tP18SubscriberListenerRK10StatusMask)
member function on the DomainParticipant instance.

Mandatory arguments are:

* A string with the name that identifies the Subscriber.

Optional arguments are:

* A Listener derived from SubscriberListener, implementing the callbacks
  that will be triggered in response to events and state changes on the Subscriber.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  SubscriberListener.
  By default all events are enabled.

[`create_subscriber_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant30create_subscriber_with_profileERKNSt6stringER12ReturnCode_tP18SubscriberListenerRK10StatusMask) will return a null pointer if there was an error during
the operation, e.g. if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

#### NOTE
XML profiles must have been loaded previously. See [Loading profiles from an XML file](../../domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory-load-profiles).

```c++
// First load the XML with the profiles
DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Subscriber using a profile and no Listener
Subscriber* subscriber_with_profile =
        participant->create_subscriber_with_profile("subscriber_profile");
if (nullptr == subscriber_with_profile)
{
    // Error
    return;
}

// Create a Subscriber using a profile and a custom Listener.
// CustomSubscriberListener inherits from SubscriberListener.
CustomSubscriberListener custom_listener;
Subscriber* subscriber_with_profile_and_custom_listener =
        participant->create_subscriber_with_profile("subscriber_profile", &custom_listener);
if (nullptr == subscriber_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-subscriber-deletion"></a>

<a id="deleting-a-subscriber"></a>

## Deleting a Subscriber

A Subscriber can be deleted with the [`delete_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17delete_subscriberEPK10Subscriber) member function on the
DomainParticipant instance where the Subscriber was created.

#### NOTE
A Subscriber can only be deleted if all Entities belonging to the Subscriber
(DataReaders) have already been deleted.
Otherwise, the function will issue an error and the Subscriber will not be deleted.
This can be performed by using the [`delete_contained_entities()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber25delete_contained_entitiesEv) member function of the
[Subscriber](subscriber.md#dds-layer-subscriber-subscriber).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Subscriber
Subscriber* subscriber =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber)
{
    // Error
    return;
}

// Use the Subscriber to communicate
// (...)

// Delete the entities the subscriber created
if (subscriber->delete_contained_entities() != RETCODE_OK)
{
    // Subscriber failed to delete the entities it created
    return;
}

// Delete the Subscriber
if (participant->delete_subscriber(subscriber) != RETCODE_OK)
{
    // Error
    return;
}
```
