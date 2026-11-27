<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domainparticipant-creation"></a>

<a id="creating-a-domainparticipant"></a>

# Creating a DomainParticipant

Creation of a [DomainParticipant](domainParticipant.md#dds-layer-domainparticipant) is done with the [`create_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask)
member function on the
[DomainParticipantFactory](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton, that acts as a factory for the DomainParticipant.
Then, when the lifecycle of the participant finishes, every participant must be deleted with
[`delete_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18delete_participantEP17DomainParticipant).
Please refer to [Deleting a DomainParticipant](#dds-layer-domainparticipant-deletion) for further details on the deletion of a participant.

Mandatory arguments are:

* The `DomainId` that identifies the domain where the DomainParticipant will be created.
* The [DomainParticipantQos](domainParticipant.md#dds-layer-domainparticipantqos) describing the behavior of the DomainParticipant.
  If the provided value is `PARTICIPANT_QOS_DEFAULT`, the value of the DomainParticipantQos is used.

Alternatively, instead of the two mandatory arguments above, you can use:

* The [DomainParticipantExtendedQos](domainParticipant.md#dds-layer-domainparticipantextendedqos) which encompasses both the `DomainId` and
  the [DomainParticipantQos](domainParticipant.md#dds-layer-domainparticipantqos).

Optional arguments are:

* A Listener derived from [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the DomainParticipant.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).

#### WARNING
Following the [DDSI-RTPS V2.2](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) standard (Section 9.6.1.1), the default ports are calculated depending on the
`DomainId`, as it is explained in section [Well Known Ports](../../../transport/listening_locators.md#listening-locators-defaultports).
Thus, it is encouraged to use `DomainId` lower than 200
(over `DomainId` 233 default port assign will fail consistently).

[`create_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask)
will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

```c++
// Create a DomainParticipant with default DomainParticipantQos and no Listener
// The value PARTICIPANT_QOS_DEFAULT is used to denote the default QoS.
DomainParticipant* participant_with_default_attributes =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant_with_default_attributes)
{
    // Error
    return;
}

// A custom DomainParticipantQos can be provided to the creation method
DomainParticipantQos custom_qos;

// Modify QoS attributes
// (...)

DomainParticipant* participant_with_custom_qos =
        DomainParticipantFactory::get_instance()->create_participant(0, custom_qos);
if (nullptr == participant_with_custom_qos)
{
    // Error
    return;
}

// Create a DomainParticipant with default QoS and a custom Listener.
// CustomDomainParticipantListener inherits from DomainParticipantListener.
// The value PARTICIPANT_QOS_DEFAULT is used to denote the default QoS.
CustomDomainParticipantListener custom_listener;
DomainParticipant* participant_with_default_qos_and_custom_listener =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT,
                &custom_listener);
if (nullptr == participant_with_default_qos_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-domainparticipant-creation-profile"></a>

<a id="profile-based-creation-of-a-domainparticipant"></a>

## Profile based creation of a DomainParticipant

Instead of using a DomainParticipantQos, the name of a profile
can be used to create a DomainParticipant with the [`create_participant_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileE10DomainId_tRKNSt6stringEP25DomainParticipantListenerRK10StatusMask)
member function on the [DomainParticipantFactory](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton.

Mandatory arguments are:

* The `DomainId` that identifies the domain where the DomainParticipant will be created.
  Do not use `DomainId` higher than 200 (see [Creating a DomainParticipant](#dds-layer-domainparticipant-creation)).
* The name of the profile to be applied to the DomainParticipant.

Optional arguments are:

* A Listener derived from [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the DomainParticipant.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).
  By default all events are enabled.

[`create_participant_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileE10DomainId_tRKNSt6stringEP25DomainParticipantListenerRK10StatusMask) will return a null pointer if there was an error during
the operation, e.g if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

#### NOTE
XML profiles must have been loaded previously. See [Loading profiles from an XML file](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory-load-profiles).

```c++
// First load the XML with the profiles
DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

// Create a DomainParticipant using a profile and no Listener
DomainParticipant* participant_with_profile =
        DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile");
if (nullptr == participant_with_profile)
{
    // Error
    return;
}

// Create a DomainParticipant using a profile and a custom Listener.
// CustomDomainParticipantListener inherits from DomainParticipantListener.
CustomDomainParticipantListener custom_listener;
DomainParticipant* participant_with_profile_and_custom_listener =
        DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile",
                &custom_listener);
if (nullptr == participant_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-domainparticipant-creation-default-profile"></a>

<a id="default-profile-domainparticipant-creation"></a>

## Default profile DomainParticipant creation

If there is a profile already exported in the environment (please refer to [XML profiles](../../../xml_configuration/xml_configuration.md#xml-profiles) for related
information), creating a DomainParticipant with the
[`create_participant_with_default_profile()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory39create_participant_with_default_profileEv) member function on the
[DomainParticipantFactory](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton would use that settings to configure the participant.
If the profile has not been exported, the DomainParticipant will be created with the default values per
[DomainParticipantQos](domainParticipant.md#dds-layer-domainparticipantqos), and `0` as `DomainId`.

Optional arguments are:

* A Listener derived from [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the DomainParticipant.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  [DomainParticipantListener](../domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).
  By default all events are enabled.

[`create_participant_with_default_profile()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory39create_participant_with_default_profileEv) will return a null pointer if there was an
error during the operation.
It is advisable to check that the returned value is a valid pointer.

#### NOTE
XML profiles must have been loaded previously. See [XML profiles](../../../xml_configuration/xml_configuration.md#xml-profiles).

```c++
// Create a DomainParticipant using the environment profile and no Listener
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant_with_default_profile();
if (nullptr == participant)
{
    // Error
    return;
}

// Create a DomainParticipant using the environment profile and a custom Listener.
// CustomDomainParticipantListener inherits from DomainParticipantListener.
CustomDomainParticipantListener custom_listener;
DomainParticipant* participant_with_custom_listener =
        DomainParticipantFactory::get_instance()->create_participant_with_default_profile(
    &custom_listener, StatusMask::none());
if (nullptr == participant_with_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-domainparticipant-deletion"></a>

<a id="deleting-a-domainparticipant"></a>

## Deleting a DomainParticipant

A DomainParticipant shall be deleted with the [`delete_participant()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18delete_participantEP17DomainParticipant) member function on the
[DomainParticipantFactory](../domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton.

#### IMPORTANT
A DomainParticipant can only be deleted if all Entities belonging to the participant
(Publisher, Subscriber or Topic) have already been deleted.
Otherwise, the function will issue an error and the DomainParticipant will not be deleted.
This can be performed either by using the [`delete_contained_entities()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25delete_contained_entitiesEv) member function of the
[DomainParticipant](domainParticipant.md#dds-layer-domainparticipant) or manually deleting each entity with the corresponding *delete_* method from the
DomainParticipant e.g [`delete_publisher()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant16delete_publisherEPK9Publisher), [`delete_subscriber()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant17delete_subscriberEPK10Subscriber),
[`delete_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12delete_topicEPK5Topic), etc.

```c++
// Create a DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Use the DomainParticipant to communicate
// (...)

// Delete entities created by the DomainParticipant
if (participant->delete_contained_entities() != RETCODE_OK)
{
    // DomainParticipant failed to delete the entities it created.
    return;
}

// Delete the DomainParticipant
if (DomainParticipantFactory::get_instance()->delete_participant(participant) != RETCODE_OK)
{
    // Error
    return;
}
```
