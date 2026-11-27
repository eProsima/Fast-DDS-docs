<a id="api-pim-topiclistener"></a>

<a id="topiclistener"></a>

# TopicListener

### class TopicListener

Class [TopicListener](#classeprosima_1_1fastdds_1_1dds_1_1TopicListener), it should be used by the end user to implement specific callbacks to certain actions. 

Subclassed by [eprosima::fastdds::dds::DomainParticipantListener](../domain/domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)

### Public Functions

### inline TopicListener()

Constructor. 

### inline virtual ~TopicListener()

Destructor. 

### inline virtual void on_inconsistent_topic([Topic](topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*topic, [InconsistentTopicStatus](../core/status/inconsistenttopicstatus.md#_CPPv4N8eprosima7fastdds3dds23InconsistentTopicStatusE) status)

Virtual function to be implemented by the user containing the actions to be performed when another topic exists with the same name but different characteristics.

It is only called by the default implementation of [on_inconsistent_topic(Topic\*, const InconsistentTopicStatus&)](#classeprosima_1_1fastdds_1_1dds_1_1TopicListener_1ac8acb663d5af21681bb5966e08a643fd).

* **Parameters:**
  * **topic** – [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)
  * **status** – The inconsistent topic status 

### inline virtual void on_inconsistent_topic([Topic](topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*topic, const [InconsistentTopicStatus](../core/status/inconsistenttopicstatus.md#_CPPv4N8eprosima7fastdds3dds23InconsistentTopicStatusE) &status)

 **[Pro]**  Virtual function to be implemented by the user containing the actions to be performed when another topic exists with the same name but different characteristics.

The default implementation calls the deprecated [on_inconsistent_topic(Topic\*, InconsistentTopicStatus)](#classeprosima_1_1fastdds_1_1dds_1_1TopicListener_1a52b46978f56cc8c9936f965316105ab0), so listeners overriding it are still notified.

#### NOTE
Both overloads are equally good for a call with an InconsistentTopicStatus argument, so a direct call through a [TopicListener](#classeprosima_1_1fastdds_1_1dds_1_1TopicListener) is ambiguous. Call it through a pointer to this member function.

* **Parameters:**
  * **topic** – [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)
  * **status** – The inconsistent topic status
