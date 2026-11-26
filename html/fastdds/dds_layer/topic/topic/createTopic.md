<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-creation"></a>

<a id="creating-a-topic"></a>

# Creating a Topic

A [Topic](topic.md#dds-layer-topic-topic) always belongs to a [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
Creation of a Topic is done with the [`create_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12create_topicERKNSt6stringERKNSt6stringERK8TopicQosR12ReturnCode_tP13TopicListenerRK10StatusMask) member function on the
[`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) instance, that acts as a factory for the [`Topic`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE).

Mandatory arguments are:

* A string with the name that identifies the Topic.
* The name of the registered [data type](../typeSupport/typeSupport.md#dds-layer-definition-data-types) that will be transmitted.
* The [TopicQos](topic.md#dds-layer-topic-topicqos) describing the behavior of the Topic.
  If the provided value is `TOPIC_QOS_DEFAULT`,
  the value of the [Default TopicQos](topic.md#dds-layer-defaulttopicqos) is used.

Optional arguments are:

* A Listener derived from [TopicListener](../topicListener/topicListener.md#dds-layer-topic-topiclistener), implementing the callbacks
  that will be triggered in response to events and state changes on the Topic.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  TopicListener.
  By default all events are enabled.

[`create_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12create_topicERKNSt6stringERKNSt6stringERK8TopicQosR12ReturnCode_tP13TopicListenerRK10StatusMask) will return a null pointer if there was an error during the operation, e.g.
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

// Create a Topic with default TopicQos and no Listener
// The symbol TOPIC_QOS_DEFAULT is used to denote the default QoS.
Topic* topic_with_default_qos =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
if (nullptr == topic_with_default_qos)
{
    // Error
    return;
}

// A custom TopicQos can be provided to the creation method
TopicQos custom_qos;

// Modify QoS attributes
// (...)

Topic* topic_with_custom_qos =
        participant->create_topic("TopicName", "DataTypeName", custom_qos);
if (nullptr == topic_with_custom_qos)
{
    // Error
    return;
}

// Create a Topic with default QoS and a custom Listener.
// CustomTopicListener inherits from TopicListener.
// The symbol TOPIC_QOS_DEFAULT is used to denote the default QoS.
CustomTopicListener custom_listener;
Topic* topic_with_default_qos_and_custom_listener =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT, &custom_listener);
if (nullptr == topic_with_default_qos_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-topic-creation-profile"></a>

<a id="profile-based-creation-of-a-topic"></a>

## Profile based creation of a Topic

Instead of using a TopicQos, the name of a profile
can be used to create a Topic with the [`create_topic_with_profile()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25create_topic_with_profileERKNSt6stringERKNSt6stringERKNSt6stringER12ReturnCode_tP13TopicListenerRK10StatusMask)
member function on the DomainParticipant instance.

Mandatory arguments are:

* A string with the name that identifies the Topic.
* The name of the registered [data type](../typeSupport/typeSupport.md#dds-layer-definition-data-types) that will be transmitted.
* The name of the profile to be applied to the Topic.

Optional arguments are:

* A Listener derived from TopicListener, implementing the callbacks
  that will be triggered in response to events and state changes on the Topic.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  TopicListener.
  By default all events are enabled.

`create_topic_with_profile()` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
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

// Create a Topic using a profile and no Listener
Topic* topic_with_profile =
        participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile");
if (nullptr == topic_with_profile)
{
    // Error
    return;
}

// Create a Topic using a profile and a custom Listener.
// CustomTopicListener inherits from TopicListener.
CustomTopicListener custom_listener;
Topic* topic_with_profile_and_custom_listener =
        participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile", &custom_listener);
if (nullptr == topic_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-topic-deletion"></a>

<a id="deleting-a-topic"></a>

## Deleting a Topic

A Topic can be deleted with the [`delete_topic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12delete_topicEPK5Topic) member function on the
DomainParticipant instance where the Topic was created.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create a Topic
Topic* topic =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Use the Topic to communicate
// (...)

// Delete the Topic
if (participant->delete_topic(topic) != RETCODE_OK)
{
    // Error
    return;
}
```
