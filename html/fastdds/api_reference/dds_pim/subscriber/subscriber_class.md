<a id="api-pim-subscriber-class"></a>

<a id="subscriber"></a>

# Subscriber

### class Subscriber : public eprosima::fastdds::dds::[DomainEntity](../core/domainentity.md#_CPPv4N8eprosima7fastdds3dds12DomainEntityE)

Class [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber), contains the public API that allows the user to control the reception of messages. This class should not be instantiated directly. DomainRTPSParticipant class should be used to correctly create this element. 

### Public Functions

### inline virtual ~Subscriber()

Destructor. 

### virtual ReturnCode_t enable() override

This operation enables the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the participant creating this [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) is not enabled. 

### const [SubscriberQos](subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &get_qos() const

Allows accessing the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) Qos.

* **Returns:**
  [SubscriberQos](subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) reference 

### ReturnCode_t get_qos([SubscriberQos](subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos) const

Retrieves the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) Qos.

* **Parameters:**
  **qos** – [SubscriberQos](subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos) where the qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [SubscriberQos](subscriberqos.md#_CPPv4N8eprosima7fastdds3dds13SubscriberQosE) &qos)

Allows modifying the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) Qos. The given Qos must be supported by the [SubscriberQos](subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos).

* **Parameters:**
  **qos** – new value for [SubscriberQos](subscriberqos.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberQos)
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [SubscriberListener](subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*get_listener() const

Retrieves the attached [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener).

* **Returns:**
  Pointer to the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener)

### ReturnCode_t set_listener([SubscriberListener](subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener)

Modifies the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8)

* **Parameters:**
  **listener** – new value for [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener)
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_listener([SubscriberListener](subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Modifies the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener).

* **Parameters:**
  * **listener** – new value for the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to. 
* **Returns:**
  RETCODE_OK 

### [DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*create_datareader([TopicDescription](../topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) \*topic, const [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &reader_qos, [DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)(), std::shared_ptr<fastdds::rtps::[IPayloadPool](../../rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> payload_pool = nullptr)

This operation creates a [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The returned [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will be attached and belong to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber).

* **Parameters:**
  * **topic** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will be listening. 
  * **reader_qos** – QoS of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
  * **payload_pool** – IPayloadPool shared pointer that defines reader payload (default: nullptr). 
* **Returns:**
  Pointer to the created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). nullptr if failed. 

### [DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*create_datareader_with_profile([TopicDescription](../topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) \*topic, const std::string &profile_name, [DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)(), std::shared_ptr<fastdds::rtps::[IPayloadPool](../../rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> payload_pool = nullptr)

This operation creates a [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The returned [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will be attached and belongs to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber).

* **Parameters:**
  * **topic** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will be listening. 
  * **profile_name** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile name. 
  * **listener** – Pointer to the listener (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
  * **payload_pool** – IPayloadPool shared pointer that defines reader payload (default: nullptr). 
* **Returns:**
  Pointer to the created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). nullptr if failed. 

### ReturnCode_t delete_datareader(const [DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader)

This operation deletes a [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that belongs to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber).

The delete_datareader operation must be called on the same [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) object used to create the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). If delete_datareader is called on a different [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber), the operation will have no effect and it will return an error.

* **Parameters:**
  **reader** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to delete 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the datareader does not belong to this subscriber, RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise. 

### [DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*lookup_datareader(const std::string &topic_name) const

This operation retrieves a previously-created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) belonging to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) that is attached to a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) with a matching topic_name. If no such [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) exists, the operation will return nullptr.

If multiple DataReaders attached to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) satisfy this condition, then the operation will return one of them. It is not specified which one.

* **Parameters:**
  **topic_name** – Name of the topic associated to the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
* **Returns:**
  Pointer to a previously created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) created on a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) with that topic_name 

### ReturnCode_t get_datareaders(std::vector<[DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE)\*> &readers) const

This operation allows the application to access the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects.

* **Parameters:**
  **readers** – Vector of [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) where the list of existing readers is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_datareaders(std::vector<[DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE)\*> &readers, const std::vector<[SampleStateKind](samplestatekind.md#_CPPv4N8eprosima7fastdds3dds15SampleStateKindE)> &sample_states, const std::vector<[ViewStateKind](viewstatekind.md#_CPPv4N8eprosima7fastdds3dds13ViewStateKindE)> &view_states, const std::vector<[InstanceStateKind](instancestatekind.md#_CPPv4N8eprosima7fastdds3dds17InstanceStateKindE)> &instance_states) const

This operation allows the application to access the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects that contain samples with the specified sample_states, view_states, and instance_states. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **readers** – **[out]** Vector of [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) where the list of existing readers is returned 
  * **sample_states** – Vector of SampleStateKind 
  * **view_states** – Vector of ViewStateKind 
  * **instance_states** – Vector of InstanceStateKind 
* **Returns:**
  RETCODE_OK

### bool has_datareaders() const

This operation checks if the subscriber has DataReaders

* **Returns:**
  true if the subscriber has one or several DataReaders, false in other case 

### ReturnCode_t begin_access()

Indicates that the application is about to access the data samples in any of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects attached to the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK

### ReturnCode_t end_access()

Indicates that the application has finished accessing the data samples in [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects managed by the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK

### ReturnCode_t notify_datareaders() const

This operation invokes the operation on_data_available on the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener) objects attached to contained [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities.

This operation is typically invoked from the on_data_on_readers operation in the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener). That way the [SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener) can delegate to the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener) objects the handling of the data.

* **Returns:**
  RETCODE_OK 

### ReturnCode_t delete_contained_entities()

Deletes all contained DataReaders. If the DataReaders have any QueryCondition or [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition), they are deleted before the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) itself. 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise 

### ReturnCode_t set_default_datareader_qos(const [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos)

This operation sets a default value of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) QoS policies which will be used for newly created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities in the case where the QoS policies are defaulted in the create_datareader operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return false.

The special value DATAREADER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_datareader_qos operation had never been called.

* **Parameters:**
  **qos** – new value for [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) to set as default 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &get_default_datareader_qos() const

This operation returns the default value of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) QoS, that is, the QoS policies which will be used for newly created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities in the case where the QoS policies are defaulted in the create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Returns:**
  Current default [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

### [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &get_default_datareader_qos()

This operation returns the default value of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) QoS, that is, the QoS policies which will be used for newly created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities in the case where the QoS policies are defaulted in the create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Returns:**
  Current default [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

### ReturnCode_t get_default_datareader_qos([DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos) const

This operation retrieves the default value of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) QoS, that is, the QoS policies which will be used for newly created [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities in the case where the QoS policies are defaulted in the create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) where the default_qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_datareader_qos_from_profile(const std::string &profile_name, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile name. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datareader_qos_from_profile(const std::string &profile_name, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos, std::string &topic_name) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the values of the XML profile, and also its corresponding topic name (if specified).

* **Parameters:**
  * **profile_name** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile name. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the first [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos, std::string &topic_name) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the first [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos, const std::string &profile_name) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
  * **profile_name** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos, std::string &topic_name, const std::string &profile_name) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile with `profile_name` to be found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **profile_name** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the default [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile found in the provided XML (if there is).

#### NOTE
This method does not update the default datareader qos (returned by `get_default_datareader_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_datareader_qos_from_xml(const std::string &xml, [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos, std::string &topic_name) const

Fills the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) with the default [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) profile found in the provided XML (if there is), and also its corresponding topic name (if specified).

#### NOTE
This method does not update the default datareader qos (returned by `get_default_datareader_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### const [DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*get_participant() const

This operation returns the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) to which the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) belongs.

* **Returns:**
  [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) Pointer 

### const InstanceHandle_t &get_instance_handle() const

Returns the [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber)’s handle.

* **Returns:**
  InstanceHandle of this [Subscriber](#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### Public Static Functions

### static ReturnCode_t copy_from_topic_qos([DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &reader_qos, const [TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &topic_qos)

Copies [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) into the corresponding [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **reader_qos** – **[inout]** 
  * **topic_qos** – **[in]** 
* **Returns:**
  RETCODE_OK if successful, an error code otherwise
