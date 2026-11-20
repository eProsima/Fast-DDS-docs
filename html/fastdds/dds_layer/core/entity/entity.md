<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-core-entity"></a>

<a id="entity"></a>

# Entity

[`Entity`](../../../api_reference/dds_pim/core/entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE) is the abstract base class for all the DDS entities, meaning an object that supports QoS policies,
a listener, and statuses.

<a id="dds-layer-core-entity-types"></a>

<a id="types-of-entities"></a>

## Types of Entities

- **DomainParticipant**: This entity is the entry-point of the Service and acts as a factory for Publishers,
  Subscribers, and Topics.
  See [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) for further details.
- **Publisher**: It acts as a factory that can create any number of DataWriters.
  See [Publisher](../../publisher/publisher/publisher.md#dds-layer-publisher-publisher) for further details.
- **Subscriber**:  It acts as a factory that can create any number of DataReaders.
  See [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) for further details.
- **Topic**: This entity fits between the publication and subscription entities and acts as a channel.
  See [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) for further details.
- **DataWriter**: Is the object responsible for the data distribution.
  See [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) for further details.
- **DataReader**: Is the object used to access the received data.
  See [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) for further details.

The following figure shows the hierarchy between all DDS entities:

![image](01-figures/entity_diagram.svg)

<a id="dds-layer-core-entity-commonchars"></a>

<a id="common-entity-characteristics"></a>

## Common Entity Characteristics

All entity types share some characteristics that are common to the concept of an entity.
Those are:

<a id="dds-layer-core-entity-commonchars-identifier"></a>

<a id="entity-identifier"></a>

### Entity Identifier

Each entity is identified by a unique ID, which is shared between the DDS entity and its corresponding RTPS entity
if it exists.
That ID is stored on an Instance Handle object declared on Entity base class, which can be accessed using the getter
function [`get_instance_handle()`](../../../api_reference/dds_pim/core/entity.md#_CPPv4NK8eprosima7fastdds3dds6Entity19get_instance_handleEv).

<a id="dds-layer-core-entity-commonchars-qos"></a>

<a id="qos-policy"></a>

### QoS policy

The behavior of each entity can be configured with a set of configuration policies.
For each entity type, there is a corresponding Quality of Service (QoS) class that groups all the policies that affect
said entity type.
Users can create instances of these QoS classes, modify the contained policies to their needs,
and use them to configure the entities, either during their creation or at a later time with the `set_qos()`
function that every entity exposes ([`DomainParticipant::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos), [`Publisher::set_qos()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher7set_qosERK12PublisherQos),
[`Subscriber::set_qos()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber7set_qosERK13SubscriberQos), [`Topic::set_qos()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic7set_qosERK8TopicQos), [`DataWriter::set_qos()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter7set_qosERK13DataWriterQos), [`DataReader::set_qos()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader7set_qosERK13DataReaderQos)).
See [Policy](../policy/policy.md#dds-layer-core-policy) for a list of the available policies and their description.
The QoS classes and the policies they contain are explained in the documentation for each entity type.

<a id="dds-layer-core-entity-commonchars-listener"></a>

<a id="listener"></a>

### Listener

A listener is an object with functions that an entity will call in response to events.
Therefore, the listener acts as an asynchronous notification system that allows the entity to notify the application
about the [Status](#dds-layer-core-entity-commonchars-status) changes in the entity.

All entity types define an abstract listener interface, which contains the callback functions that the entity will
trigger to communicate the Status changes to the application.
Users can implement their own listeners inheriting from these interfaces and implementing the callbacks that
are needed on their application.
Then they can link these listeners to each entity, either during their creation or at a later time with the
`set_listener()` function that every entity exposes
([`DomainParticipant::set_listener()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12set_listenerEP25DomainParticipantListener), [`Publisher::set_listener()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher12set_listenerEP17PublisherListener),
[`Subscriber::set_listener()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber12set_listenerEP18SubscriberListener), [`Topic::set_listener()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic12set_listenerEP13TopicListenerRK10StatusMask), [`DataWriter::set_listener()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter12set_listenerEP18DataWriterListener),
[`DataReader::set_listener()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader12set_listenerEP18DataReaderListener)).
The listener interfaces that each entity type and their callbacks are explained in the documentation
for each entity type.
When an event occurs it is handled by the lowest level entity with a listener that is non-null
and has the corresponding callback enabled in its [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE).
Higher level listeners inherit from the lower level ones as shown in the following
diagram:

![image](01-figures/listeners_inheritance_diagram.svg)

*Listeners inheritance diagram.*

#### NOTE
The [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber) callback intercepts messages before
[`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader).
This implies that if [`DomainParticipantListener`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) is enabled, users should take into account that by default
the listener uses [`StatusMask::all()`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv).
As the callback entity hierarchy is kept, the [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber) is going to be called
in this case.
If an application wants to use [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader) instead, the corresponding bit of
[`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) should be disabled.

#### IMPORTANT
Using [`StatusMask::none()`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask4noneEv) when creating the [`Entity`](../../../api_reference/dds_pim/core/entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE) only disables the DDS standard callbacks:

* [`on_sample_rejected()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener18on_sample_rejectedEP10DataReaderRK20SampleRejectedStatus)
* [`on_liveliness_changed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener21on_liveliness_changedEP10DataReaderRK23LivelinessChangedStatus)
* [`on_requested_deadline_missed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus)
* [`on_requested_incompatible_qos()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener29on_requested_incompatible_qosEP10DataReaderRK30RequestedIncompatibleQosStatus)
* [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader)
* [`on_subscription_matched()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE)
* [`on_sample_lost()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener14on_sample_lostEP10DataReaderRK16SampleLostStatus)
* [`on_offered_incompatible_qos()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener27on_offered_incompatible_qosEP10DataWriterRK28OfferedIncompatibleQosStatus)
* [`on_offered_deadline_missed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener26on_offered_deadline_missedEP10DataWriterRK27OfferedDeadlineMissedStatus)
* [`on_liveliness_lost()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener18on_liveliness_lostEP10DataWriterRK20LivelinessLostStatus)
* [`on_publication_matched()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus)
* [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus)
* [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber)

Any callback specific to *Fast DDS* is always enabled:

* [`on_participant_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_participant_discoveryEP17DomainParticipantN7fastdds4rtps26ParticipantDiscoveryStatusERK27ParticipantBuiltinTopicDataRb)
* [`onParticipantAuthentication()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener27onParticipantAuthenticationEP17DomainParticipantRRN4rtps29ParticipantAuthenticationInfoE)
* [`on_data_reader_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_reader_discoveryEP17DomainParticipantN4rtps21ReaderDiscoveryStatusERK28SubscriptionBuiltinTopicDataRb)
* [`on_data_writer_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_writer_discoveryEP17DomainParticipantN4rtps21WriterDiscoveryStatusERK27PublicationBuiltinTopicDataRb)
* [`on_unacknowledged_sample_removed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener32on_unacknowledged_sample_removedEP10DataWriterRK16InstanceHandle_t)

#### WARNING
Only one thread is created to listen for every listener implemented, so it is encouraged to
keep listener functions simple, leaving the process of such information to the proper class.

#### WARNING
Do not create or delete any Entity within the scope of a Listener member function, since it could lead to an undefined
behavior. It is recommended instead to use the Listener class as an information channel and the upper
Entity class to encapsulate such behaviour.

<a id="dds-layer-core-entity-commonchars-status"></a>

<a id="status"></a>

### Status

Each entity is associated with a set of status objects whose values represent the *communication status*
of that entity.
The changes on these status values are the ones that trigger the invocation of the appropriate
Listener callback to asynchronously inform the application.
See [Status](../status/status.md#dds-layer-core-status) for a list of all the status objects and a description of their content.
There you can also find which status applies to which entity type.

<a id="dds-layer-core-entity-commonchars-statuscondition"></a>

<a id="statuscondition"></a>

### StatusCondition

Every entity owns a StatusCondition that will be notified whenever its enabled statuses change.
The StatusCondition provides the link between an Entity and a Wait-set.
See section [Conditions and Wait-sets](../waitsets/waitsets.md#dds-layer-core-waitsets) for more information.

<a id="dds-layer-core-entity-commonchars-enabling"></a>

<a id="enabling-entities"></a>

### Enabling Entities

All the entities can be created either enabled or not enabled.
By default, the factories are configured to create the
entities enabled, but it can be changed using the [EntityFactoryQosPolicy](../policy/standardQosPolicies.md#entityfactoryqospolicy) on enabled
factories.
A disabled factory creates disabled entities regardless of its QoS.
A disabled entity has its operations limited to the following ones:

- Set/Get the entity QoS Policy.
- Set/Get the entity Listener.
- Create/Delete subentities.
- Get the Status of the entity, even if they will not change.
- Lookup operations.

Any other function called in this state will return `NOT_ENABLED`.
