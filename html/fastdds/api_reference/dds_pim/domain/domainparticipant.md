<a id="api-pim-domainparticipant"></a>

<a id="domainparticipant"></a>

# DomainParticipant

### class DomainParticipant : public eprosima::fastdds::dds::[Entity](../core/entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE)

Class [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) used to group Publishers and Subscribers into a single working unit. 

### Public Functions

### virtual ~DomainParticipant()

Destructor. 

### ReturnCode_t get_qos([DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

This operation returns the value of the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS policies

* **Parameters:**
  **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) reference where the qos is going to be returned 
* **Returns:**
  RETCODE_OK 

### const [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &get_qos() const

This operation returns the value of the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS policies. 

* **Returns:**
  A reference to the [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos)

### ReturnCode_t set_qos(const [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

This operation sets the value of the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS policies.

* **Parameters:**
  **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) to be set 
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*get_listener() const

Allows accessing the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener).

* **Returns:**
  [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) pointer 

### ReturnCode_t set_listener([DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener)

Modifies the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8)

#### WARNING
Do not call this method from a `[DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)` callback. 

* **Parameters:**
  **listener** – New value for the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR otherwise. 

### ReturnCode_t set_listener([DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener, const std::chrono::seconds timeout)

Modifies the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8)

#### WARNING
Do not call this method from a `[DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)` callback. 

* **Parameters:**
  * **listener** – New value for the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)
  * **timeout** – Maximum time to wait for executing callbacks to finish. 
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR if failed (timeout expired). 

### ReturnCode_t set_listener([DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Modifies the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener).

#### WARNING
Do not call this method from a `[DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)` callback. 

* **Parameters:**
  * **listener** – New value for the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to 
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR otherwise. 

### ReturnCode_t set_listener([DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask, const std::chrono::seconds timeout)

Modifies the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener).

#### WARNING
Do not call this method from a `[DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)` callback. 

* **Parameters:**
  * **listener** – New value for the [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to 
  * **timeout** – Maximum time to wait for executing callbacks to finish. 
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR if failed (timeout expired) 

### ReturnCode_t set_congestion_control_listener(fastdds::rtps::CongestionControlListener \*listener)

 **[Pro]**  Sets the listener that observes congestion-control meta-information.

Congestion control is participant-scoped (one controller per participant covering all reliable writers, evaluated per remote reader). The listener is purely observational and does not change congestion-control behavior; it only takes effect when congestion control is enabled through the participant’s properties.

#### WARNING
The listener object must outlive the participant; it is not copied. 

* **Parameters:**
  **listener** – New congestion-control listener, or nullptr to detach. 
* **Returns:**
  RETCODE_OK if successful.

### virtual ReturnCode_t enable() override

This operation enables the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). 

* **Returns:**
  RETCODE_OK 

### [Publisher](../publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*create_publisher(const [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos, ReturnCode_t &ret_code, [PublisherListener](../publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) in this Participant.

* **Parameters:**
  * **qos** – QoS of the [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the publisher is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

### [Publisher](../publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*create_publisher(const [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos, [PublisherListener](../publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) in this Participant.

* **Parameters:**
  * **qos** – QoS of the [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

### [Publisher](../publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*create_publisher_with_profile(const std::string &profile_name, ReturnCode_t &ret_code, [PublisherListener](../publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) in this Participant.

* **Parameters:**
  * **profile_name** – [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) profile name. 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the publisher is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

### [Publisher](../publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*create_publisher_with_profile(const std::string &profile_name, [PublisherListener](../publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) in this Participant.

* **Parameters:**
  * **profile_name** – [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) profile name. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

### ReturnCode_t delete_publisher(const [Publisher](../publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*publisher)

Deletes an existing [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher).

* **Parameters:**
  **publisher** – to be deleted. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the publisher does not belong to this participant or if it has active DataWriters, RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise. 

### [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*create_subscriber(const [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos, ReturnCode_t &ret_code, [SubscriberListener](../subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) in this Participant.

* **Parameters:**
  * **qos** – QoS of the [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the subscriber is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*create_subscriber(const [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos, [SubscriberListener](../subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) in this Participant.

* **Parameters:**
  * **qos** – QoS of the [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*create_subscriber_with_profile(const std::string &profile_name, ReturnCode_t &ret_code, [SubscriberListener](../subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) in this Participant.

* **Parameters:**
  * **profile_name** – [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) profile name. 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the subscriber is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*create_subscriber_with_profile(const std::string &profile_name, [SubscriberListener](../subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) in this Participant.

* **Parameters:**
  * **profile_name** – [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) profile name. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### ReturnCode_t delete_subscriber(const [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*subscriber)

Deletes an existing [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber).

* **Parameters:**
  **subscriber** – to be deleted. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the subscriber does not belong to this participant or if it has active DataReaders, RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise. 

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*create_topic(const std::string &topic_name, const std::string &type_name, const [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, ReturnCode_t &ret_code, [TopicListener](../topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) in this Participant.

* **Parameters:**
  * **topic_name** – Name of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **type_name** – Data type of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **qos** – QoS of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the topic is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*create_topic(const std::string &topic_name, const std::string &type_name, const [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, [TopicListener](../topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) in this Participant.

* **Parameters:**
  * **topic_name** – Name of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **type_name** – Data type of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **qos** – QoS of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*create_topic_with_profile(const std::string &topic_name, const std::string &type_name, const std::string &profile_name, ReturnCode_t &ret_code, [TopicListener](../topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) in this Participant.

* **Parameters:**
  * **topic_name** – Name of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **type_name** – Data type of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the topic is created, RETCODE_ERROR otherwise. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*create_topic_with_profile(const std::string &topic_name, const std::string &type_name, const std::string &profile_name, [TopicListener](../topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) in this Participant.

* **Parameters:**
  * **topic_name** – Name of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **type_name** – Data type of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all) 
* **Returns:**
  Pointer to the created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). 

### ReturnCode_t delete_topic(const [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*topic)

Deletes an existing [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic).

* **Parameters:**
  **topic** – to be deleted. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to this participant or if it is referenced by any entity and RETCODE_OK if the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) was deleted. 

### [ContentFilteredTopic](../topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) \*create_contentfilteredtopic(const std::string &name, [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*related_topic, const std::string &filter_expression, const std::vector<std::string> &expression_parameters, ReturnCode_t &ret_code)

Create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) in this Participant.

* **Parameters:**
  * **name** – Name of the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic)
  * **related_topic** – Related [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) to being subscribed 
  * **filter_expression** – Logic expression to create filter 
  * **expression_parameters** – Parameters to filter content 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) is created, RETCODE_ERROR otherwise. 
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). 
* **Returns:**
  nullptr if `related_topic` does not belong to this participant. 
* **Returns:**
  nullptr if a topic with the specified `name` has already been created. 
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and `expression_parameters`. 

### [ContentFilteredTopic](../topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) \*create_contentfilteredtopic(const std::string &name, [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*related_topic, const std::string &filter_expression, const std::vector<std::string> &expression_parameters)

Create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) in this Participant.

* **Parameters:**
  * **name** – Name of the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic)
  * **related_topic** – Related [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) to being subscribed 
  * **filter_expression** – Logic expression to create filter 
  * **expression_parameters** – Parameters to filter content 
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). 
* **Returns:**
  nullptr if `related_topic` does not belong to this participant. 
* **Returns:**
  nullptr if a topic with the specified `name` has already been created. 
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and `expression_parameters`. 

### [ContentFilteredTopic](../topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) \*create_contentfilteredtopic(const std::string &name, [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*related_topic, const std::string &filter_expression, const std::vector<std::string> &expression_parameters, const char \*filter_class_name, ReturnCode_t &ret_code)

Create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) in this Participant using a custom filter.

* **Parameters:**
  * **name** – Name of the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic)
  * **related_topic** – Related [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) to being subscribed 
  * **filter_expression** – Logic expression to create filter 
  * **expression_parameters** – Parameters to filter content 
  * **filter_class_name** – Name of the filter class to use 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) is created, RETCODE_ERROR otherwise.
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). 
* **Returns:**
  nullptr if `related_topic` does not belong to this participant. 
* **Returns:**
  nullptr if a topic with the specified `name` has already been created. 
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and `expression_parameters`. 
* **Returns:**
  nullptr if the specified `filter_class_name` has not been registered. 

### [ContentFilteredTopic](../topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) \*create_contentfilteredtopic(const std::string &name, [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*related_topic, const std::string &filter_expression, const std::vector<std::string> &expression_parameters, const char \*filter_class_name)

Create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) in this Participant using a custom filter.

* **Parameters:**
  * **name** – Name of the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic)
  * **related_topic** – Related [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) to being subscribed 
  * **filter_expression** – Logic expression to create filter 
  * **expression_parameters** – Parameters to filter content 
  * **filter_class_name** – Name of the filter class to use
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). 
* **Returns:**
  nullptr if `related_topic` does not belong to this participant. 
* **Returns:**
  nullptr if a topic with the specified `name` has already been created. 
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and `expression_parameters`. 
* **Returns:**
  nullptr if the specified `filter_class_name` has not been registered. 

### ReturnCode_t delete_contentfilteredtopic(const [ContentFilteredTopic](../topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) \*a_contentfilteredtopic)

Deletes an existing [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic).

* **Parameters:**
  **a_contentfilteredtopic** – [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) to be deleted 
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to this participant or if it is referenced by any entity and RETCODE_OK if the [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) was deleted. 

### MultiTopic \*create_multitopic(const std::string &name, const std::string &type_name, const std::string &subscription_expression, const std::vector<std::string> &expression_parameters, ReturnCode_t &ret_code)

Create a MultiTopic in this Participant.

* **Parameters:**
  * **name** – Name of the MultiTopic 
  * **type_name** – Result type of the MultiTopic 
  * **subscription_expression** – Logic expression to combine filter 
  * **expression_parameters** – Parameters to subscription content 
  * **ret_code** – **[out]** Return code of the operation, RETCODE_OK if the MultiTopic is created, RETCODE_ERROR otherwise. 
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic), nullptr in error case 

### MultiTopic \*create_multitopic(const std::string &name, const std::string &type_name, const std::string &subscription_expression, const std::vector<std::string> &expression_parameters)

Create a MultiTopic in this Participant.

* **Parameters:**
  * **name** – Name of the MultiTopic 
  * **type_name** – Result type of the MultiTopic 
  * **subscription_expression** – Logic expression to combine filter 
  * **expression_parameters** – Parameters to subscription content 
* **Returns:**
  Pointer to the created [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic), nullptr in error case 

### ReturnCode_t delete_multitopic(const MultiTopic \*a_multitopic)

Deletes an existing MultiTopic.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **a_multitopic** – MultiTopic to be deleted 
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to this participant or if it is referenced by any entity and RETCODE_OK if the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) was deleted.

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*find_topic(const std::string &topic_name, const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &timeout)

Gives access to an existing (or ready to exist) enabled [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). It should be noted that the returned [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) is a local object that acts as a proxy to designate the global concept of topic. Topics obtained by means of find_topic, must also be deleted by means of delete_topic so that the local resources can be released. If a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) is obtained multiple times by means of find_topic or create_topic, it must also be deleted that same number of times using delete_topic.

* **Parameters:**
  * **topic_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) name 
  * **timeout** – Maximum time to wait for the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)
* **Returns:**
  Pointer to the existing [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic), nullptr in case of error or timeout 

### rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*create_service(const std::string &service_name, const std::string &service_type_name, ReturnCode_t &ret_code)

Create an enabled RPC service.

* **Parameters:**
  * **service_name** – Name of the service. 
  * **service_type_name** – Type name of the service (Request & reply types) 
  * **ret_code** – **[out]** Return code indicating the result of the operation. 
* **Returns:**
  Pointer to the created service. nullptr in error case. 

### rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*create_service(const std::string &service_name, const std::string &service_type_name)

Create a RPC service.

* **Parameters:**
  * **service_name** – Name of the service. 
  * **service_type_name** – Type name of the service (Request & reply types)
* **Returns:**
  Pointer to the created service. nullptr in error case. 

### rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*find_service(const std::string &service_name) const

Find a RPC service by name

* **Parameters:**
  **service_name** – Name of the service to search for. 
* **Returns:**
  Pointer to the service object if found, nullptr if not found. 

### ReturnCode_t delete_service(const rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*service)

Delete a registered RPC service

* **Parameters:**
  **service** – Pointer to the service to be deleted. 
* **Returns:**
  RETCODE_OK if the service was deleted, or an specific error code otherwise. 

### rpc::[Requester](../rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) \*create_service_requester(rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*service, const [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &requester_qos, ReturnCode_t &ret_code)

Create a RPC Requester in a given Service. 

* **Parameters:**
  * **service** – Pointer to a service object where the requester will be created. 
  * **requester_qos** – QoS of the requester. 
  * **ret_code** – **[out]** Return code indicating the result of the operation. 
* **Returns:**
  Pointer to the created requester. nullptr in error case. 

### rpc::[Requester](../rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) \*create_service_requester(rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*service, const [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &requester_qos)

Create a RPC Requester in a given Service.

* **Parameters:**
  * **service** – Pointer to a service object where the requester will be created. 
  * **requester_qos** – QoS of the requester.
* **Returns:**
  Pointer to the created requester. nullptr in error case. 

### ReturnCode_t delete_service_requester(const std::string &service_name, rpc::[Requester](../rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) \*requester)

Deletes an existing RPC Requester

* **Parameters:**
  * **service_name** – Name of the service where the requester is created. 
  * **requester** – Pointer to the requester to be deleted. 
* **Returns:**
  RETCODE_OK if the requester was deleted, or an specific error code otherwise. 

### rpc::[Replier](../rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) \*create_service_replier(rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*service, const [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &replier_qos, ReturnCode_t &ret_code)

Create a RPC Replier in a given Service.

* **Parameters:**
  * **service** – Pointer to a service object where the Replier will be created. 
  * **replier_qos** – QoS of the replier. 
  * **ret_code** – **[out]** Return code indicating the result of the operation.
* **Returns:**
  Pointer to the created replier. nullptr in error case. 

### rpc::[Replier](../rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) \*create_service_replier(rpc::[Service](../rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) \*service, const [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &replier_qos)

Create a RPC Replier in a given Service. It will override the current service’s replier

* **Parameters:**
  * **service** – Pointer to a service object where the Replier will be created. 
  * **replier_qos** – QoS of the replier.
* **Returns:**
  Pointer to the created replier. nullptr in error case. 

### ReturnCode_t delete_service_replier(const std::string &service_name, rpc::[Replier](../rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) \*replier)

Deletes an existing RPC Replier

* **Parameters:**
  * **service_name** – Name of the service where the replier is created. 
  * **replier** – Pointer to the replier to be deleted. 
* **Returns:**
  RETCODE_OK if the replier was deleted, or an specific error code otherwise. 

### [TopicDescription](../topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) \*lookup_topicdescription(const std::string &topic_name) const

Looks up an existing, locally created [TopicDescription](../topic/topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription), based on its name. May be called on a disabled participant.

**Remark**

UNSAFE. It is unsafe to lookup a topic description while another thread is creating a topic. 

* **Parameters:**
  **topic_name** – Name of the [TopicDescription](../topic/topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription) to search for. 
* **Returns:**
  Pointer to the topic description, if it has been created locally. Otherwise, nullptr is returned.

### const [Subscriber](../subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*get_builtin_subscriber() const

Allows access to the builtin [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber).

* **Returns:**
  Pointer to the builtin [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber), nullptr in error case 

### ReturnCode_t ignore_participant(const InstanceHandle_t &handle)

Locally ignore a remote domain participant.

#### NOTE
This action is not reversible.

* **Parameters:**
  **handle** – Identifier of the remote participant to ignore 
* **Returns:**
  RETURN_OK code if everything correct, RETCODE_BAD_PARAMENTER otherwise 

### ReturnCode_t ignore_topic(const InstanceHandle_t &handle)

Locally ignore a topic.

#### NOTE
This action is not reversible.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **handle** – Identifier of the topic to ignore 
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

### ReturnCode_t ignore_publication(const InstanceHandle_t &handle)

Locally ignore a remote datawriter.

#### NOTE
This action is not reversible.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **handle** – Identifier of the datawriter to ignore 
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

### ReturnCode_t ignore_subscription(const InstanceHandle_t &handle)

Locally ignore a remote datareader.

#### NOTE
This action is not reversible.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **handle** – Identifier of the datareader to ignore 
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

### DomainId_t get_domain_id() const

This operation retrieves the domain_id used to create the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). The domain_id identifies the DDS domain to which the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) belongs.

* **Returns:**
  The Participant’s domain_id 

### ReturnCode_t delete_contained_entities()

Deletes all the entities that were created by means of the “create” methods

* **Returns:**
  RETURN_OK code if everything correct, error code otherwise 

### ReturnCode_t assert_liveliness()

This operation manually asserts the liveliness of the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). This is used in combination with the LIVELINESS QoS policy to indicate to the Service that the entity remains active.

This operation needs to only be used if the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) contains [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities with the LIVELINESS set to MANUAL_BY_PARTICIPANT and it only affects the liveliness of those [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities. Otherwise, it has no effect.

#### NOTE
Writing data via the write operation on a [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) asserts liveliness on the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) itself and its [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). Consequently the use of assert_liveliness is only needed if the application is not writing data regularly.

* **Returns:**
  RETCODE_OK if the liveliness was asserted, RETCODE_ERROR otherwise. 

### ReturnCode_t set_default_publisher_qos(const [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos)

This operation sets a default value of the [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) QoS policies which will be used for newly created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) entities in the case where the QoS policies are defaulted in the create_publisher operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return false.

The special value PUBLISHER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_publisher_qos operation had never been called.

* **Parameters:**
  **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) to be set 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &get_default_publisher_qos() const

This operation retrieves the default value of the [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) QoS, that is, the QoS policies which will be used for newly created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) entities in the case where the QoS policies are defaulted in the create_publisher operation.

The values retrieved get_default_publisher_qos will match the set of values specified on the last successful call to set_default_publisher_qos, or else, if the call was never made, the default values.

* **Returns:**
  Current default publisher qos. 

### ReturnCode_t get_default_publisher_qos([PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos) const

This operation retrieves the default value of the [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) QoS, that is, the QoS policies which will be used for newly created [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) entities in the case where the QoS policies are defaulted in the create_publisher operation.

The values retrieved get_default_publisher_qos will match the set of values specified on the last successful call to set_default_publisher_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) reference where the default_publisher_qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_publisher_qos_from_profile(const std::string &profile_name, [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos) const

Fills the [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) profile name. 
  * **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_publisher_qos_from_xml(const std::string &xml, [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos) const

Fills the [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) with the first publisher profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_publisher_qos_from_xml(const std::string &xml, [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos, const std::string &profile_name) const

Fills the [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) with the publisher profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) object where the qos is returned. 
  * **profile_name** – [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_publisher_qos_from_xml(const std::string &xml, [PublisherQos](../publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos) const

Fills the [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) with the default publisher profile found in the provided XML (if there is).

#### NOTE
This method does not update the default publisher qos (returned by `get_default_publisher_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [PublisherQos](../publisher/publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t set_default_subscriber_qos(const [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos)

This operation sets a default value of the [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) QoS policies that will be used for newly created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) entities in the case where the QoS policies are defaulted in the create_subscriber operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return false.

The special value SUBSCRIBER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_subscriber_qos operation had never been called.

* **Parameters:**
  **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) to be set 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &get_default_subscriber_qos() const

This operation retrieves the default value of the [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) QoS, that is, the QoS policies which will be used for newly created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) entities in the case where the QoS policies are defaulted in the create_subscriber operation.

The values retrieved get_default_subscriber_qos will match the set of values specified on the last successful call to set_default_subscriber_qos, or else, if the call was never made, the default values.

* **Returns:**
  Current default subscriber qos. 

### ReturnCode_t get_default_subscriber_qos([SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos) const

This operation retrieves the default value of the [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) QoS, that is, the QoS policies which will be used for newly created [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) entities in the case where the QoS policies are defaulted in the create_subscriber operation.

The values retrieved get_default_subscriber_qos will match the set of values specified on the last successful call to set_default_subscriber_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) reference where the default_subscriber_qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_subscriber_qos_from_profile(const std::string &profile_name, [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos) const

Fills the [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) profile name. 
  * **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_subscriber_qos_from_xml(const std::string &xml, [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos) const

Fills the [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) with the first subscriber profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_subscriber_qos_from_xml(const std::string &xml, [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos, const std::string &profile_name) const

Fills the [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) with the subscriber profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) object where the qos is returned. 
  * **profile_name** – [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_subscriber_qos_from_xml(const std::string &xml, [SubscriberQos](../subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos) const

Fills the [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) with the default subscriber profile found in the provided XML (if there is).

#### NOTE
This method does not update the default subscriber qos (returned by `get_default_subscriber_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [SubscriberQos](../subscriber/subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t set_default_topic_qos(const [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos)

This operation sets a default value of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) QoS policies which will be used for newly created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) entities in the case where the QoS policies are defaulted in the create_topic operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return INCONSISTENT_POLICY.

The special value TOPIC_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_topic_qos operation had never been called.

* **Parameters:**
  **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) to be set 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &get_default_topic_qos() const

This operation retrieves the default value of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) QoS, that is, the QoS policies that will be used for newly created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) entities in the case where the QoS policies are defaulted in the create_topic operation.

The values retrieved get_default_topic_qos will match the set of values specified on the last successful call to set_default_topic_qos, or else, TOPIC_QOS_DEFAULT if the call was never made.

* **Returns:**
  Current default topic qos. 

### ReturnCode_t get_default_topic_qos([TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos) const

This operation retrieves the default value of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) QoS, that is, the QoS policies that will be used for newly created [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) entities in the case where the QoS policies are defaulted in the create_topic operation.

The values retrieved get_default_topic_qos will match the set of values specified on the last successful call to set_default_topic_qos, or else, TOPIC_QOS_DEFAULT if the call was never made.

* **Parameters:**
  **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) reference where the default_topic_qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_topic_qos_from_profile(const std::string &profile_name, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_topic_qos_from_profile(const std::string &profile_name, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, std::string &topic_name, std::string &topic_data_type) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the values of the XML profile, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **topic_data_type** – String where the name of the topic data type associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the first topic profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, std::string &topic_name, std::string &topic_data_type) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the first topic profile found in the provided XML, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **topic_data_type** – String where the name of the topic data type associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, const std::string &profile_name) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the topic profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, std::string &topic_name, std::string &topic_data_type, const std::string &profile_name) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the topic profile with `profile_name` to be found in the provided XML, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **topic_data_type** – String where the name of the topic data type associated to this profile is returned (if specified). 
  * **profile_name** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the default topic profile found in the provided XML (if there is).

#### NOTE
This method does not update the default topic qos (returned by `get_default_topic_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_topic_qos_from_xml(const std::string &xml, [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &qos, std::string &topic_name, std::string &topic_data_type) const

Fills the [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) with the default topic profile found in the provided XML (if there is), and also its corresponding topic and data type names (if specified).

#### NOTE
This method does not update the default topic qos (returned by `get_default_topic_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **topic_data_type** – String where the name of the topic data type associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_replier_qos_from_profile(const std::string &profile_name, [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &qos) const

Fills the [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – Replier profile name. 
  * **qos** – [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_replier_qos_from_xml(const std::string &xml, [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &qos) const

Fills the [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) with the first replier profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_replier_qos_from_xml(const std::string &xml, [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &qos, const std::string &profile_name) const

Fills the [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) with the replier profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) object where the qos is returned. 
  * **profile_name** – Replier profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_replier_qos_from_xml(const std::string &xml, [ReplierQos](../rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &qos) const

Fills the [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) with the default replier profile found in the provided XML (if there is).

#### NOTE
This method does not update the default replier qos.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [ReplierQos](../rpc/replierqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReplierQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_requester_qos_from_profile(const std::string &profile_name, [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &qos) const

Fills the [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – Requester profile name. 
  * **qos** – [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_requester_qos_from_xml(const std::string &xml, [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &qos) const

Fills the [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) with the first requester profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_requester_qos_from_xml(const std::string &xml, [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &qos, const std::string &profile_name) const

Fills the [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) with the requester profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) object where the qos is returned. 
  * **profile_name** – Requester profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_requester_qos_from_xml(const std::string &xml, [RequesterQos](../rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) &qos) const

Fills the [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) with the default requester profile found in the provided XML (if there is).

#### NOTE
This method does not update the default requester qos.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [RequesterQos](../rpc/requesterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RequesterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_discovered_participants(std::vector<InstanceHandle_t> &participant_handles) const

Retrieves the list of DomainParticipants that have been discovered in the domain and are not “ignored”.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **participant_handles** – **[out]** Reference to the vector where discovered participants will be returned 
* **Returns:**
  RETCODE_OK if everything correct, error code otherwise

### ReturnCode_t get_discovered_participant_data(ParticipantBuiltinTopicData &participant_data, const InstanceHandle_t &participant_handle) const

Retrieves the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) data of a discovered not ignored participant.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **participant_data** – **[out]** Reference to the ParticipantBuiltinTopicData object to return the data 
  * **participant_handle** – InstanceHandle of [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) to retrieve the data from 
* **Returns:**
  RETCODE_OK if everything correct, PRECONDITION_NOT_MET if participant does not exist

### ReturnCode_t get_discovered_topics(std::vector<InstanceHandle_t> &topic_handles) const

Retrieves the list of topics that have been discovered in the domain and are not “ignored”.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **topic_handles** – **[out]** Reference to the vector where discovered topics will be returned 
* **Returns:**
  RETCODE_OK if everything correct, error code otherwise

### ReturnCode_t get_discovered_topic_data(builtin::TopicBuiltinTopicData &topic_data, const InstanceHandle_t &topic_handle) const

Retrieves the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) data of a discovered not ignored topic.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **topic_data** – **[out]** Reference to the TopicBuiltinTopicData object to return the data 
  * **topic_handle** – InstanceHandle of [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) to retrieve the data from 
* **Returns:**
  RETCODE_OK if everything correct, PRECONDITION_NOT_MET if topic does not exist

### bool contains_entity(const InstanceHandle_t &a_handle, bool recursive = true) const

This operation checks whether or not the given handle represents an [Entity](../core/entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) that was created from the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant).

* **Parameters:**
  * **a_handle** – InstanceHandle of the entity to look for. 
  * **recursive** – The containment applies recursively. That is, it applies both to entities ([TopicDescription](../topic/topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription), [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher), or [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber)) created directly using the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) as well as entities created using a contained [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher), or [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) as the factory, and so forth. (default: true) 
* **Returns:**
  True if entity is contained. False otherwise. 

### ReturnCode_t get_current_time(fastdds::dds::[Time_t](../core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &current_time) const

This operation returns the current value of the time that the service uses to time-stamp data-writes and to set the reception-timestamp for the data-updates it receives.

* **Parameters:**
  **current_time** – [Time_t](../core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t) reference where the current time is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t register_type([TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) type, const std::string &type_name)

Register a type in this participant.

* **Parameters:**
  * **type** – [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport). 
  * **type_name** – The name that will be used to identify the Type. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there is another [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) with the same name and RETCODE_OK if it is correctly registered. 

### ReturnCode_t register_type([TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) type)

Register a type in this participant.

* **Parameters:**
  **type** – [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport). 
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there is another [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) with the same name and RETCODE_OK if it is correctly registered. 

### ReturnCode_t unregister_type(const std::string &typeName)

Unregister a type in this participant.

* **Parameters:**
  **typeName** – Name of the type 
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there are entities using that [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) and RETCODE_OK if it is correctly unregistered. 

### [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) find_type(const std::string &type_name) const

This method gives access to a registered type based on its name.

* **Parameters:**
  **type_name** – Name of the type 
* **Returns:**
  [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) corresponding to the type_name 

### ReturnCode_t register_service_type(rpc::[ServiceTypeSupport](../rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) service_type, const std::string &service_type_name)

Register a service type in this participant.

* **Parameters:**
  * **service_type** – ServiceTypeSupport. 
  * **service_type_name** – The name that will be used to identify the service type. 
* **Returns:**
  RETCODE_OK if it is correctly registered. Error code otherwise. 

### ReturnCode_t unregister_service_type(const std::string &service_type_name)

Unregister a service type in this participant.

* **Parameters:**
  **service_type_name** – Name of the type 
* **Returns:**
  RETCODE_OK if it is correctly unregistered. Error code otherwise. 

### rpc::[ServiceTypeSupport](../rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) find_service_type(const std::string &service_type_name) const

This method gives access to a registered service type based on its name.

* **Parameters:**
  **service_type_name** – Name of the type 
* **Returns:**
  ServiceTypeSupport corresponding to the service_type_name 

### InstanceHandle_t get_instance_handle() const

Returns the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant)’s handle.

* **Returns:**
  InstanceHandle of this [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). 

### const fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid() const

Getter for the Participant GUID. 

* **Returns:**
  A reference to the GUID 

### std::vector<std::string> get_participant_names() const

Getter for the participant names. 

* **Returns:**
  Vector with the names 

### bool new_remote_endpoint_discovered(const fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &partguid, std::uint16_t userId, fastdds::rtps::[EndpointKind_t](../../rtps/common/Types/EndpointKind_t.md#_CPPv4N8eprosima7fastdds4rtps14EndpointKind_tE) kind)

This method can be used when using a StaticEndpointDiscovery mechanism different that the one included in Fast DDS, for example when communicating with other implementations. It indicates the Participant that an Endpoint from the XML has been discovered and should be activated.

* **Parameters:**
  * **partguid** – Participant GUID_t. 
  * **userId** – User defined ID as shown in the XML file. 
  * **kind** – EndpointKind (WRITER or READER) 
* **Returns:**
  True if correctly found and activated. 

### ReturnCode_t register_content_filter_factory(const char \*filter_class_name, [IContentFilterFactory](../topic/icontentfilterfactory.md#_CPPv4N8eprosima7fastdds3dds21IContentFilterFactoryE) \*const filter_factory)

Register a custom content filter factory, which can be used to create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic).

DDS specifies a SQL-like content filter to be used by content filtered topics. If this filter does not meet your filtering requirements, you can register a custom filter factory.

To use a custom filter, a factory for it must be registered in the following places:

* In any application that uses the custom filter factory to create a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) and the corresponding [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader).
* In each application that writes the data to the applications mentioned above.

For example, suppose Application A on the subscription side creates a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) named X and a [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) named filteredX (and a corresponding [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)), using a previously registered content filter factory, myFilterFactory. With only that, you will have filtering at the subscription side. If you also want to perform filtering in any application that publishes [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) X, then you also need to register the same definition of the ContentFilterFactory myFilterFactory in that application.

Each `filter_class_name` can only be used to register a content filter factory once per [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant).

* **Parameters:**
  * **filter_class_name** – Name of the filter class. Cannot be nullptr, must not exceed 255 characters, and must be unique within this [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). 
  * **filter_factory** – Factory of content filters to be registered. Cannot be nullptr.
* **Returns:**
  RETCODE_BAD_PARAMETER if any parameter is nullptr, or the filter_class_name exceeds 255 characters. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the filter_class_name has been already registered. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if filter_class_name is FASTDDS_SQLFILTER_NAME. 
* **Returns:**
  RETCODE_OK if the filter is correctly registered. 

### [IContentFilterFactory](../topic/icontentfilterfactory.md#_CPPv4N8eprosima7fastdds3dds21IContentFilterFactoryE) \*lookup_content_filter_factory(const char \*filter_class_name)

Lookup a custom content filter factory previously registered with register_content_filter_factory.

* **Parameters:**
  **filter_class_name** – Name of the filter class. Cannot be nullptr.
* **Returns:**
  nullptr if the given filter_class_name has not been previously registered on this [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). Otherwise, the content filter factory previously registered with the given filter_class_name. 

### ReturnCode_t unregister_content_filter_factory(const char \*filter_class_name)

Unregister a custom content filter factory previously registered with register_content_filter_factory.

A filter_class_name can be unregistered only if it has been previously registered to the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) with register_content_filter_factory.

The unregistration of filter is not allowed if there are any existing [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) objects that are using the filter.

If there is any existing discovered [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) with the same filter_class_name, filtering on the writer side will be stopped, but this operation will not fail.

* **Parameters:**
  **filter_class_name** – Name of the filter class. Cannot be nullptr.
* **Returns:**
  RETCODE_BAD_PARAMETER if the filter_class_name is nullptr. 
* **Returns:**
  RERCODE_PRECONDITION_NOT_MET if the filter_class_name has not been previously registered. 
* **Returns:**
  RERCODE_PRECONDITION_NOT_MET if there is any [ContentFilteredTopic](../topic/contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) referencing the filter. 
* **Returns:**
  RETCODE_OK if the filter is correctly unregistered. 

### bool has_active_entities()

Check if the Participant has any [Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher), [Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) or [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) created by the user. 

The entities that the statistics module creates for itself are not taken into account, since they do not keep the Participant from being deleted.

* **Returns:**
  true if any, false otherwise. 

### ReturnCode_t enable_statistics_datawriter(const std::string &topic_name, const [DataWriterQos](../publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &dwqos)

 **[Pro]**  This operation enables a Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

* **Parameters:**
  * **topic_name** – **[in]** Name of the topic associated to the Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **dwqos** – **[in]** [DataWriterQos](../publisher/datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) to be set 
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), RETCODE_INCONSISTENT_POLICY if the [DataWriterQos](../publisher/datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) provided are inconsistent, RETCODE_OK if the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) has been created or if it has been created previously, and RETCODE_ERROR otherwise 

### ReturnCode_t enable_statistics_datawriter_with_profile(const std::string &profile_name, const std::string &topic_name)

 **[Pro]**  This operation enables a Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) from a given profile

* **Parameters:**
  * **profile_name** – **[in]** [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) QoS profile name 
  * **topic_name** – **[in]** Name of the statistics topic to be enabled. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), RETCODE_INCONSISTENT_POLICY if the [DataWriterQos](../publisher/datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) provided in profile are inconsistent, RETCODE_OK if the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) has been created or if it has been created previously, and RETCODE_ERROR otherwise 

### ReturnCode_t disable_statistics_datawriter(const std::string &topic_name)

 **[Pro]**  This operation disables a Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

* **Parameters:**
  **topic_name** – Name of the topic associated to the Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), RETCODE_OK if the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) has been correctly deleted or does not exist, and RETCODE_ERROR otherwise 

### ReturnCode_t enable_monitor_service()

 **[Pro]**  Enables the monitor service in the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant).

* **Returns:**
  RETCODE_OK if the monitor service could be correctly enabled. 
* **Returns:**
  RETCODE_ERROR if the monitor service could not be enabled properly. 

### ReturnCode_t disable_monitor_service()

 **[Pro]**  Disables the monitor service in this [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). Does nothing if the service was not enabled before.

* **Returns:**
  RETCODE_OK if the monitor service could be correctly disabled. 
* **Returns:**
  RETCODE_NOT_ENABLED if the monitor service was not previously enabled. 
* **Returns:**
  RETCODE_ERROR if the service could not be properly disabled. 

### ReturnCode_t fill_discovery_data_from_cdr_message(ParticipantBuiltinTopicData &data, const statistics::MonitorServiceStatusData &msg)

 **[Pro]**  fills in the ParticipantBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  RETCODE_OK if the operation succeeds. 
* **Returns:**
  RETCODE_ERROR if the operation fails. 

### ReturnCode_t fill_discovery_data_from_cdr_message(PublicationBuiltinTopicData &data, const statistics::MonitorServiceStatusData &msg)

 **[Pro]**  fills in the PublicationBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill. 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  RETCODE_OK if the operation succeeds. 
* **Returns:**
  RETCODE_ERROR if the operation fails. 

### ReturnCode_t fill_discovery_data_from_cdr_message(SubscriptionBuiltinTopicData &data, const statistics::MonitorServiceStatusData &msg)

 **[Pro]**  fills in the SubscriptionBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill. 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  RETCODE_OK if the operation succeeds. 
* **Returns:**
  RETCODE_ERROR if the operation fails. 

### Public Static Functions

### static [DomainParticipant](#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*narrow([DomainParticipant](#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*domain_participant)

 **[Pro]**  Kept for backwards compatibility, as the statistics API is now part of this class.

* **Parameters:**
  **domain_participant** – Reference to the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant)
* **Returns:**
  The same [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) that was provided. 

### static const [DomainParticipant](#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*narrow(const [DomainParticipant](#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*domain_participant)

 **[Pro]**  Kept for backwards compatibility, as the statistics API is now part of this class.

* **Parameters:**
  **domain_participant** – Constant reference to the [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant)
* **Returns:**
  The same [DomainParticipant](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) that was provided.
