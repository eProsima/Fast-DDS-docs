<a id="api-pim-publisher-class"></a>

<a id="publisher"></a>

# Publisher

### class Publisher : public eprosima::fastdds::dds::[DomainEntity](../core/domainentity.md#_CPPv4N8eprosima7fastdds3dds12DomainEntityE)

Class [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher), used to send data to associated subscribers. 

### Public Functions

### virtual ~Publisher()

Destructor. 

### virtual ReturnCode_t enable() override

This operation enables the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the participant creating this [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) is not enabled. 

### const [PublisherQos](publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &get_qos() const

Allows accessing the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) Qos.

* **Returns:**
  [PublisherQos](publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) reference 

### ReturnCode_t get_qos([PublisherQos](publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos) const

Retrieves the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) Qos.

* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [PublisherQos](publisherqos.md#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) &qos)

Allows modifying the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) Qos. The given Qos must be supported by the [PublisherQos](publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos).

* **Parameters:**
  **qos** – [PublisherQos](publisherqos.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos) to be set 
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [PublisherListener](publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*get_listener() const

Retrieves the attached [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener).

* **Returns:**
  [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener) pointer 

### ReturnCode_t set_listener([PublisherListener](publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener)

Modifies the [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8)

* **Parameters:**
  **listener** – new value for the [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener)
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_listener([PublisherListener](publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Modifies the [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener).

* **Parameters:**
  * **listener** – new value for the [PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to 
* **Returns:**
  RETCODE_OK 

### [DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*create_datawriter([Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*topic, const [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, [DataWriterListener](datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)(), std::shared_ptr<fastdds::rtps::[IPayloadPool](../../rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> payload_pool = nullptr)

This operation creates a [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). The returned [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) will be attached and belongs to the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher).

* **Parameters:**
  * **topic** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) will be listening 
  * **qos** – QoS of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 
  * **listener** – Pointer to the listener (default: nullptr). 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
  * **payload_pool** – IPayloadPool shared pointer that defines writer payload (default: nullptr). 
* **Returns:**
  Pointer to the created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). nullptr if failed. 

### [DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*create_datawriter_with_profile([Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*topic, const std::string &profile_name, [DataWriterListener](datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)(), std::shared_ptr<fastdds::rtps::[IPayloadPool](../../rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> payload_pool = nullptr)

This operation creates a [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). The returned [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) will be attached and belongs to the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher).

* **Parameters:**
  * **topic** – [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) will be listening 
  * **profile_name** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile name. 
  * **listener** – Pointer to the listener (default: nullptr). 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
  * **payload_pool** – IPayloadPool shared pointer that defines writer payload (default: nullptr). 
* **Returns:**
  Pointer to the created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). nullptr if failed. 

### ReturnCode_t delete_datawriter(const [DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer)

This operation deletes a [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that belongs to the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher).

The delete_datawriter operation must be called on the same [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) object used to create the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). If delete_datawriter is called on a different [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher), the operation will have no effect and it will return false.

The deletion of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) will automatically unregister all instances. Depending on the settings of the WRITER_DATA_LIFECYCLE [QosPolicy](../core/policy/qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy), the deletion of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) may also dispose all instances.

* **Parameters:**
  **writer** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) to delete 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if it does not belong to this [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher), RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise. 

### [DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*lookup_datawriter(const std::string &topic_name) const

This operation retrieves a previously created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) belonging to the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) that is attached to a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) with a matching topic_name. If no such [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) exists, the operation will return nullptr.

If multiple [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) attached to the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) satisfy this condition, then the operation will return one of them. It is not specified which one.

* **Parameters:**
  **topic_name** – Name of the [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)
* **Returns:**
  Pointer to a previously created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) associated to a [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) with the requested topic_name 

### ReturnCode_t suspend_publications()

Indicates to FastDDS that the contained DataWriters are about to be modified. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise

### ReturnCode_t resume_publications()

Indicates to FastDDS that the modifications to the DataWriters are complete. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise

### ReturnCode_t begin_coherent_changes()

Signals the beginning of a set of coherent cache changes using the Datawriters attached to the publisher. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise

### ReturnCode_t end_coherent_changes()

Signals the end of a set of coherent cache changes. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise

### ReturnCode_t wait_for_acknowledgments(const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &max_wait)

This operation blocks the calling thread until either all data written by the reliable [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities is acknowledged by all matched reliable [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities, or else the duration specified by the max_wait parameter elapses, whichever happens first. A return value of true indicates that all the samples written have been acknowledged by all reliable matched data readers; a return value of false indicates that max_wait elapsed before all the data was acknowledged.

* **Parameters:**
  **max_wait** – Maximum blocking time for this operation 
* **Returns:**
  RETCODE_TIMEOUT if the function takes more than the maximum blocking time established, RETCODE_OK if the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) receives the acknowledgments and RETCODE_ERROR otherwise. 

### const [DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*get_participant() const

This operation returns the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) to which the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher) belongs.

* **Returns:**
  Pointer to the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant)

### ReturnCode_t delete_contained_entities()

Deletes all contained DataWriters. 

* **Returns:**
  RETCODE_OK if successful, an error code otherwise 

### ReturnCode_t set_default_datawriter_qos(const [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos)

This operation sets a default value of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) QoS policies which will be used for newly created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities in the case where the QoS policies are defaulted in the create_datawriter operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return false.

The special value DATAWRITER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_datawriter_qos operation had never been called.

* **Parameters:**
  **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) to be set 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &get_default_datawriter_qos() const

This operation returns the default value of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) QoS, that is, the QoS policies which will be used for newly created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities in the case where the QoS policies are defaulted in the create_datawriter operation.

The values retrieved by get_default_datawriter_qos will match the set of values specified on the last successful call to set_default_datawriter_qos, or else, if the call was never made, the default values.

* **Returns:**
  Current default WriterQos

### ReturnCode_t get_default_datawriter_qos([DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos) const

This operation retrieves the default value of the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) QoS, that is, the QoS policies which will be used for newly created [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities in the case where the QoS policies are defaulted in the create_datawriter operation.

The values retrieved by get_default_datawriter_qos will match the set of values specified on the last successful call to set_default_datawriter_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** – Reference to the current default WriterQos. 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_datawriter_qos_from_profile(const std::string &profile_name, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile name. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datawriter_qos_from_profile(const std::string &profile_name, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, std::string &topic_name) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the values of the XML profile, and also its corresponding topic name (if specified).

* **Parameters:**
  * **profile_name** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile name. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the first [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, std::string &topic_name) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the first [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, const std::string &profile_name) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
  * **profile_name** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, std::string &topic_name, const std::string &profile_name) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile with `profile_name` to be found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
  * **profile_name** – [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the default [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile found in the provided XML (if there is).

#### NOTE
This method does not update the default datawriter qos (returned by `get_default_datawriter_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_datawriter_qos_from_xml(const std::string &xml, [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos, std::string &topic_name) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the default [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) profile found in the provided XML (if there is), and also its corresponding topic name (if specified).

#### NOTE
This method does not update the default datawriter qos (returned by `get_default_datawriter_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
  * **topic_name** – String where the name of the topic associated to this profile is returned (if specified). 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### const InstanceHandle_t &get_instance_handle() const

Returns the [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher)’s handle.

* **Returns:**
  InstanceHandle of this [Publisher](#classeprosima_1_1fastdds_1_1dds_1_1Publisher). 

### bool get_datawriters(std::vector<[DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE)\*> &writers) const

Fills the given vector with all the datawriters of this publisher.

* **Parameters:**
  **writers** – Vector where the DataWriters are returned 
* **Returns:**
  true 

### bool has_datawriters() const

This operation checks if the publisher has DataWriters

* **Returns:**
  true if the publisher has one or several DataWriters, false otherwise 

### Public Static Functions

### static ReturnCode_t copy_from_topic_qos(fastdds::dds::[DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &writer_qos, const fastdds::dds::[TopicQos](../topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) &topic_qos)

Copies [TopicQos](../topic/topicqos.md#classeprosima_1_1fastdds_1_1dds_1_1TopicQos) into the corresponding [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos). 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **writer_qos** – **[out]** 
  * **topic_qos** – **[in]** 
* **Returns:**
  RETCODE_OK if successful, an error code otherwise
