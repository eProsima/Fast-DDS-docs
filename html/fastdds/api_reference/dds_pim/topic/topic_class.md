<a id="api-pim-topic-class"></a>

<a id="topic"></a>

# Topic

### class Topic : public eprosima::fastdds::dds::[DomainEntity](../core/domainentity.md#_CPPv4N8eprosima7fastdds3dds12DomainEntityE), public eprosima::fastdds::dds::[TopicDescription](topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE)

Class [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic), represents the fact that both publications and subscriptions are tied to a single data-type 

### Public Functions

### virtual [DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*get_participant() const override

Getter for the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). 

* **Returns:**
  [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer 

### ReturnCode_t get_inconsistent_topic_status([InconsistentTopicStatus](../core/status/inconsistenttopicstatus.md#_CPPv4N8eprosima7fastdds3dds23InconsistentTopicStatusE) &status)

Allows the application to retrieve the INCONSISTENT_TOPIC_STATUS status of a [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic).

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **status** – [out] Status to be retrieved. 
* **Returns:**
  RETCODE_OK 

### const [TopicQos](topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &get_qos() const

Allows accessing the [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic) Qos.

* **Returns:**
  reference to [TopicQos](topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos)

### ReturnCode_t get_qos([TopicQos](topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos) const

Retrieves the [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic) Qos.

* **Parameters:**
  **qos** – [TopicQos](topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) where the qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [TopicQos](topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos)

Allows modifying the [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic) Qos. The given Qos must be supported by the [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic).

* **Parameters:**
  **qos** – new [TopicQos](topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) value to set for the [Topic](#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
* **Return values:**
  * **RETCODE_IMMUTABLE_POLICY** – if a change was not allowed. 
  * **RETCODE_INCONSISTENT_POLICY** – if new qos has inconsistent values. 
  * **RETCODE_OK** – if qos was updated. 

### const [TopicListener](topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*get_listener() const

Retrieves the attached [TopicListener](topiclistener.md#classeprosima_1_1fastdds_1_1dds_1_1TopicListener).

* **Returns:**
  pointer to [TopicListener](topiclistener.md#classeprosima_1_1fastdds_1_1dds_1_1TopicListener)

### ReturnCode_t set_listener([TopicListener](topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Modifies the [TopicListener](topiclistener.md#classeprosima_1_1fastdds_1_1dds_1_1TopicListener).

* **Parameters:**
  * **listener** – new value for the [TopicListener](topiclistener.md#classeprosima_1_1fastdds_1_1dds_1_1TopicListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
* **Returns:**
  RETCODE_OK
