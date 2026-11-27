<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-creation"></a>

<a id="creating-a-publisher"></a>

# Creating a Publisher

A [Publisher](publisher.md#dds-layer-publisher-publisher) always belongs to a [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
Creation of a Publisher is done with the [`create_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16create_publisherERK12PublisherQosR12ReturnCode_tP17PublisherListenerRK10StatusMask) member function on the
DomainParticipant instance, that acts as a factory for the Publisher.

Mandatory arguments are:

* The [PublisherQos](publisher.md#dds-layer-publisher-publisherqos) describing the behavior of the Publisher.
  If the provided value is `PUBLISHER_QOS_DEFAULT`,
  the value of the [Default PublisherQos](publisher.md#dds-layer-defaultpublisherqos) is used.

Optional arguments are:

* A Listener derived from [PublisherListener](../publisherListener/publisherListener.md#dds-layer-publisher-publisherlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the Publisher.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  PublisherListener.
  By default all events are enabled.

[`create_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16create_publisherERK12PublisherQosR12ReturnCode_tP17PublisherListenerRK10StatusMask) will return a null pointer if there was an error during the operation, e.g.
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

// Create a Publisher with default PublisherQos and no Listener
// The value PUBLISHER_QOS_DEFAULT is used to denote the default QoS.
Publisher* publisher_with_default_qos =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher_with_default_qos)
{
    // Error
    return;
}

// A custom PublisherQos can be provided to the creation method
PublisherQos custom_qos;

// Modify QoS attributes
// (...)

Publisher* publisher_with_custom_qos =
        participant->create_publisher(custom_qos);
if (nullptr == publisher_with_custom_qos)
{
    // Error
    return;
}

// Create a Publisher with default QoS and a custom Listener.
// CustomPublisherListener inherits from PublisherListener.
// The value PUBLISHER_QOS_DEFAULT is used to denote the default QoS.
CustomPublisherListener custom_listener;
Publisher* publisher_with_default_qos_and_custom_listener =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT, &custom_listener);
if (nullptr == publisher_with_default_qos_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-publisher-creation-profile"></a>

<a id="profile-based-creation-of-a-publisher"></a>

## Profile based creation of a Publisher

Instead of using a [PublisherQos](publisher.md#dds-layer-publisher-publisherqos), the name of a profile
can be used to create a Publisher with the [`create_publisher_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant29create_publisher_with_profileERKNSt6stringER12ReturnCode_tP17PublisherListenerRK10StatusMask)
member function on the DomainParticipant instance.

Mandatory arguments are:

* A string with the name that identifies the Publisher.

Optional arguments are:

* A Listener derived from [PublisherListener](../publisherListener/publisherListener.md#dds-layer-publisher-publisherlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the Publisher.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  PublisherListener.
  By default all events are enabled.

[`create_publisher_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant29create_publisher_with_profileERKNSt6stringER12ReturnCode_tP17PublisherListenerRK10StatusMask) will return a null pointer if there was an error during the
operation, e.g. if the provided QoS is not compatible or is not supported.
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

// Create a Publisher using a profile and no Listener
Publisher* publisher_with_profile =
        participant->create_publisher_with_profile("publisher_profile");
if (nullptr == publisher_with_profile)
{
    // Error
    return;
}

// Create a Publisher using a profile and a custom Listener.
// CustomPublisherListener inherits from PublisherListener.
CustomPublisherListener custom_listener;
Publisher* publisher_with_profile_and_custom_listener =
        participant->create_publisher_with_profile("publisher_profile", &custom_listener);
if (nullptr == publisher_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-publisher-deletion"></a>

<a id="deleting-a-publisher"></a>

## Deleting a Publisher

A Publisher can be deleted with the [`delete_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16delete_publisherEPK9Publisher) member function on the
DomainParticipant instance where the Publisher was created.

#### NOTE
A Publisher can only be deleted if all Entities belonging to the Publisher
(DataWriters) have already been deleted.
Otherwise, the function will issue an error and the Publisher will not be deleted.
This can be performed by using the [`delete_contained_entities()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher25delete_contained_entitiesEv) member function of the
[Publisher](publisher.md#dds-layer-publisher-publisher).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Publisher
Publisher* publisher =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher)
{
    // Error
    return;
}

// Use the Publisher to communicate
// (...)

// Delete the entities the Publisher created.
if (publisher->delete_contained_entities() != RETCODE_OK)
{
    // Publisher failed to delete the entities it created.
    return;
}

// Delete the Publisher
if (participant->delete_publisher(publisher) != RETCODE_OK)
{
    // Error
    return;
}
```
