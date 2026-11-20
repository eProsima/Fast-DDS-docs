<a id="api-pim-datawriter"></a>

<a id="datawriter"></a>

# DataWriter

### class DataWriter : public eprosima::fastdds::dds::[DomainEntity](../core/domainentity.md#_CPPv4N8eprosima7fastdds3dds12DomainEntityE)

Class [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), contains the actual implementation of the behaviour of the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

### Public Types

### enum class LoanInitializationKind

How to initialize samples loaned with [loan_sample](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1aad64a30e5d44ab10acffc543e6dc9cb3)

*Values:*

### enumerator NO_LOAN_INITIALIZATION

Do not perform initialization of sample. 

This is the default initialization scheme of loaned samples. It is the fastest scheme, but implies the user should take care of writing every field on the data type before calling [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) on the loaned sample. 

### enumerator ZERO_LOAN_INITIALIZATION

Initialize all memory with zero-valued bytes. 

The contents of the loaned sample will be zero-initialized upon return of [loan_sample](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1aad64a30e5d44ab10acffc543e6dc9cb3). 

### enumerator CONSTRUCTED_LOAN_INITIALIZATION

Use in-place constructor initialization. 

This will call the constructor of the data type over the memory space being returned by [loan_sample](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1aad64a30e5d44ab10acffc543e6dc9cb3). 

### Public Functions

### virtual ReturnCode_t enable() override

This operation enables the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the [Publisher](publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) creating this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is not enabled. 

### ReturnCode_t write(const void \*const data)

Write data to the topic.

* **Parameters:**
  **data** – Pointer to the data 
* **Returns:**
  RETCODE_OK if the data is correctly sent or a ReturnCode related to the specific error otherwise. 

### ReturnCode_t write(const void \*const data, fastdds::rtps::[WriteParams](../../rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &params)

Write data with params to the topic.

* **Parameters:**
  * **data** – Pointer to the data 
  * **params** – Extra write parameters. 
* **Returns:**
  RETCODE_OK if the data is correctly sent or a ReturnCode related to the specific error otherwise. 

### ReturnCode_t write(const void \*const data, const InstanceHandle_t &handle)

Write data with handle.

The special value HANDLE_NIL can be used for the parameter handle.This indicates that the identity of the instance should be automatically deduced from the instance_data (by means of the key).

* **Parameters:**
  * **data** – Pointer to the data 
  * **handle** – InstanceHandle_t. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the handle introduced does not match with the one associated to the data, RETCODE_OK if the data is correctly sent and RETCODE_ERROR otherwise. 

### ReturnCode_t write_w_timestamp(const void \*const data, const InstanceHandle_t &handle, const fastdds::dds::[Time_t](../core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &timestamp)

This operation performs the same function as write except that it also provides the value for the [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) that is made available to [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects by means of the [eprosima::fastdds::dds::SampleInfo::source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) attribute “source_timestamp” inside the [SampleInfo](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo). The constraints on the values of the `handle` parameter and the corresponding error behavior are the same specified for the [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) operation. This operation may block and return RETCODE_TIMEOUT under the same circumstances described for the [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) operation. This operation may return RETCODE_OUT_OF_RESOURCES, RETCODE_PRECONDITION_NOT_MET or RETCODE_BAD_PARAMETER under the same circumstances described for the write operation. 

* **Parameters:**
  * **data** – Pointer to the data 
  * **handle** – InstanceHandle_t 
  * **timestamp** – [Time_t](../core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t) used to set the source_timestamp. 
* **Returns:**
  Any of the standard return codes. 

### InstanceHandle_t register_instance(const void \*const instance)

Informs that the application will be modifying a particular instance. 

It gives an opportunity to the middleware to pre-configure itself to improve performance.

* **Parameters:**
  **instance** – **[in]** Sample used to get the instance’s key. 
* **Returns:**
  Handle containing the instance’s key. This handle could be used in successive `write` or `dispose` operations. In case of error, HANDLE_NIL will be returned. 

### InstanceHandle_t register_instance_w_timestamp(const void \*const instance, const fastdds::dds::[Time_t](../core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &timestamp)

This operation performs the same function as register_instance and can be used instead of [register_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8464aa9cc60c2a53f9ea85f45f3b5069) in the cases where the application desires to specify the value for the [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7). The [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) potentially affects the relative order in which readers observe events from multiple writers. See the QoS policy [DESTINATION_ORDER](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos_1ad0bbec867eb388e8543e37251becf43c). 

This operation may block and return RETCODE_TIMEOUT under the same circumstances described for the [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) operation.

This operation may return RETCODE_OUT_OF_RESOURCES under the same circumstances described for the [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) operation.

* **Parameters:**
  * **instance** – Sample used to get the instance’s key. 
  * **timestamp** – [Time_t](../core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t) used to set the source_timestamp. 
* **Returns:**
  Handle containing the instance’s key. 

### ReturnCode_t unregister_instance(const void \*const instance, const InstanceHandle_t &handle)

This operation reverses the action of `register_instance`. 

It should only be called on an instance that is currently registered. Informs the middleware that the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is not intending to modify any more of that data instance. Also indicates that the middleware can locally remove all information regarding that instance.

* **Parameters:**
  * **instance** – **[in]** Sample used to deduce instance’s key in case of `handle` parameter is HANDLE_NIL. 
  * **handle** – **[in]** Instance’s key to be unregistered. 
* **Returns:**
  Returns the operation’s result. If the operation finishes successfully, RETCODE_OK is returned. 

### ReturnCode_t unregister_instance_w_timestamp(const void \*const instance, const InstanceHandle_t &handle, const fastdds::dds::[Time_t](../core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &timestamp)

This operation performs the same function as [unregister_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1acf1b46d769f807c1258565c2a4f34abe) and can be used instead of [unregister_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1acf1b46d769f807c1258565c2a4f34abe) in the cases where the application desires to specify the value for the [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7). The [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) potentially affects the relative order in which readers observe events from multiple writers. See the QoS policy [DESTINATION_ORDER](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos_1ad0bbec867eb388e8543e37251becf43c). 

The constraints on the values of the `handle` parameter and the corresponding error behavior are the same specified for the [unregister_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1acf1b46d769f807c1258565c2a4f34abe) operation.

This operation may block and return RETCODE_TIMEOUT under the same circumstances described for the write operation

* **Parameters:**
  * **instance** – Sample used to deduce instance’s key in case of `handle` parameter is HANDLE_NIL. 
  * **handle** – Instance’s key to be unregistered. 
  * **timestamp** – [Time_t](../core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t) used to set the source_timestamp. 
* **Returns:**
  Handle containing the instance’s key. 

### ReturnCode_t get_key_value(void \*key_holder, const InstanceHandle_t &handle)

This operation can be used to retrieve the instance key that corresponds to an instance_handle. The operation will only fill the fields that form the key inside the key_holder instance.

This operation may return BAD_PARAMETER if the InstanceHandle_t handle does not correspond to an existing data-object known to the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). If the implementation is not able to check invalid handles then the result in this situation is unspecified.

* **Parameters:**
  * **key_holder** – **[inout]** Sample where the key fields will be returned. 
  * **handle** – **[in]** Handle to the instance to retrieve the key values from.
* **Returns:**
  Any of the standard return codes. 

### InstanceHandle_t lookup_instance(const void \*const instance) const

NOT YET IMPLEMENTED

Takes as a parameter an instance and returns a handle that can be used in subsequent operations that accept an instance handle as an argument. The instance parameter is only used for the purpose of examining the fields that define the key.

* **Parameters:**
  **instance** – **[in]** Data pointer to the sample
* **Returns:**
  handle of the given instance 

### const fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid() const

Returns the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)’s GUID

* **Returns:**
  Reference to the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) GUID 

### InstanceHandle_t get_instance_handle() const

Returns the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)’s InstanceHandle

* **Returns:**
  Copy of the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) InstanceHandle 

### [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) get_type() const

Get data type associated to the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

* **Returns:**
  Copy of the [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport)

### ReturnCode_t wait_for_acknowledgments(const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &max_wait)

Waits the current thread until all writers have received their acknowledgments.

* **Parameters:**
  **max_wait** – Maximum blocking time for this operation 
* **Returns:**
  RETCODE_OK if the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) receive the acknowledgments before the time expires and RETCODE_ERROR otherwise 

### ReturnCode_t get_offered_deadline_missed_status([OfferedDeadlineMissedStatus](../core/status/offereddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds27OfferedDeadlineMissedStatusE) &status)

Returns the offered deadline missed status. 

* **Parameters:**
  **status** – **[out]** Deadline missed status struct 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_offered_incompatible_qos_status([OfferedIncompatibleQosStatus](../core/status/offeredicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds28OfferedIncompatibleQosStatusE) &status)

Returns the offered incompatible qos status. 

* **Parameters:**
  **status** – **[out]** Offered incompatible qos status struct 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_publication_matched_status([PublicationMatchedStatus](../core/status/publicationmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds24PublicationMatchedStatusE) &status) const

Returns the publication matched status. 

* **Parameters:**
  **status** – **[out]** publication matched status struct 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos)

Establishes the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) for this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

* **Parameters:**
  **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) to be set 
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &get_qos() const

Retrieves the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) for this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

* **Returns:**
  Reference to the current [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos)

### ReturnCode_t get_qos([DataWriterQos](datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) &qos) const

Fills the [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) with the values of this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

* **Parameters:**
  **qos** – [DataWriterQos](datawriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK 

### [Topic](../topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*get_topic() const

Retrieves the topic for this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

* **Returns:**
  Pointer to the associated [Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)

### const [DataWriterListener](datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) \*get_listener() const

Retrieves the listener for this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

* **Returns:**
  Pointer to the [DataWriterListener](datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener)

### ReturnCode_t set_listener([DataWriterListener](datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) \*listener)

Modifies the [DataWriterListener](datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8)

* **Parameters:**
  **listener** – new value for the [DataWriterListener](datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener)
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_listener([DataWriterListener](datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Modifies the [DataWriterListener](datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener).

* **Parameters:**
  * **listener** – new value for the [DataWriterListener](datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener)
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all). 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t dispose(const void \*const data, const InstanceHandle_t &handle)

This operation requests the middleware to delete the data (the actual deletion is postponed until there is no more use for that data in the whole system). In general, applications are made aware of the deletion by means of operations on the [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects that already knew that instance. This operation does not modify the value of the instance. The instance parameter is passed just for the purposes of identifying the instance. When this operation is used, the Service will automatically supply the value of the source_timestamp that is made available to [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects by means of the source_timestamp attribute inside the [SampleInfo](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo). The constraints on the values of the handle parameter and the corresponding error behavior are the same specified for the unregister_instance operation. 

* **Parameters:**
  * **data** – **[in]** Sample used to deduce instance’s key in case of `handle` parameter is HANDLE_NIL. 
  * **handle** – **[in]** InstanceHandle of the data 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the handle introduced does not match with the one associated to the data, RETCODE_OK if the data is correctly sent and RETCODE_ERROR otherwise. 

### ReturnCode_t dispose_w_timestamp(const void \*const instance, const InstanceHandle_t &handle, const fastdds::dds::[Time_t](../core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &timestamp)

This operation performs the same functions as [dispose](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1abb86bfcc5db5016533c00055e9a28d5e) except that the application provides the value for the [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) that is made available to [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects by means of the [source_timestamp](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa71276d2e848778a63868f8c3ac3a1b7) attribute inside the [SampleInfo](../subscriber/sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo). 

The constraints on the values of the `handle` parameter and the corresponding error behavior are the same specified for the [dispose](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1abb86bfcc5db5016533c00055e9a28d5e) operation.

This operation may return RETCODE_PRECONDITION_NOT_MET and RETCODE_BAD_PARAMETER under the same circumstances described for the [dispose](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1abb86bfcc5db5016533c00055e9a28d5e) operation.

This operation may return RETCODE_TIMEOUT and RETCODE_OUT_OF_RESOURCES under the same circumstances described for the [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6) operation.

* **Parameters:**
  * **instance** – Sample used to deduce instance’s key in case of `handle` parameter is HANDLE_NIL. 
  * **handle** – Instance’s key to be disposed. 
  * **timestamp** – [Time_t](../core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t) used to set the source_timestamp. 
* **Returns:**
  FASTDDS_EXPORTED_API 

### ReturnCode_t get_liveliness_lost_status([LivelinessLostStatus](../core/status/rtpslivelinessloststatus.md#_CPPv4N8eprosima7fastdds3dds20LivelinessLostStatusE) &status)

Returns the liveliness lost status. 

* **Parameters:**
  **status** – Liveliness lost status struct 
* **Returns:**
  RETCODE_OK 

### const [Publisher](publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) \*get_publisher() const

Getter for the [Publisher](publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) that creates this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

* **Returns:**
  Pointer to the [Publisher](publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher)

### ReturnCode_t assert_liveliness()

This operation manually asserts the liveliness of the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). This is used in combination with the [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy) to indicate to the Service that the entity remains active. This operation need only be used if the LIVELINESS setting is either MANUAL_BY_PARTICIPANT or MANUAL_BY_TOPIC. Otherwise, it has no effect. 

#### NOTE
Writing data via the write operation on a [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) asserts liveliness on the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) itself and its [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). Consequently the use of assert_liveliness is only needed if the application is not writing data regularly.

* **Returns:**
  RETCODE_OK if asserted, RETCODE_ERROR otherwise 

### ReturnCode_t get_matched_subscription_data(SubscriptionBuiltinTopicData &subscription_data, const InstanceHandle_t &subscription_handle) const

Retrieves in a subscription associated with the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

* **Parameters:**
  * **subscription_data** – **[out]** subscription data struct 
  * **subscription_handle** – InstanceHandle_t of the subscription 
* **Returns:**
  RETCODE_OK if successful, RETCODE_BAD_PARAMETER otherwise 

### ReturnCode_t get_matched_subscriptions(std::vector<InstanceHandle_t> &subscription_handles) const

Fills the given vector with the InstanceHandle_t of matched DataReaders. 

* **Parameters:**
  **subscription_handles** – **[out]** Vector where the InstanceHandle_t are returned 
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR otherwise 

### ReturnCode_t clear_history(std::size_t \*removed)

Clears the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) history. 

* **Parameters:**
  **removed** – size_t pointer to return the size of the data removed 
* **Returns:**
  RETCODE_OK if the samples are removed and RETCODE_ERROR otherwise 

### ReturnCode_t loan_sample(void \*&sample, [LoanInitializationKind](#_CPPv4N8eprosima7fastdds3dds10DataWriter22LoanInitializationKindE) initialization = [LoanInitializationKind](#_CPPv4N8eprosima7fastdds3dds10DataWriter22LoanInitializationKindE)::[NO_LOAN_INITIALIZATION](#_CPPv4N8eprosima7fastdds3dds10DataWriter22LoanInitializationKind22NO_LOAN_INITIALIZATIONE))

Get a pointer to the internal pool where the user could directly write. 

This method can only be used on a [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) for a plain data type. It will provide the user with a pointer to an internal buffer where the data type can be prepared for sending.

When using NO_LOAN_INITIALIZATION on the initialization parameter, which is the default, no assumptions should be made on the contents where the pointer points to, as it may be an old pointer being reused. See [LoanInitializationKind](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a880a5d5c543b72f75a43f67e1ffdea50) for more details.

Once the sample has been prepared, it can then be published by calling [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6). After a successful call to [write](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6), the middleware takes ownership of the loaned pointer again, and the user should not access that memory again.

If, for whatever reason, the sample is not published, the loan can be returned by calling [discard_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1ad754c7900c37468e0c038c12a23c70f2).

* **Parameters:**
  * **sample** – **[out]** Pointer to the sample on the internal pool. 
  * **initialization** – **[in]** How to initialize the loaned sample.
* **Returns:**
  RETCODE_ILLEGAL_OPERATION when the data type does not support loans. 
* **Returns:**
  RETCODE_NOT_ENABLED if the writer has not been enabled. 
* **Returns:**
  RETCODE_OUT_OF_RESOURCES if the pool has been exhausted. 
* **Returns:**
  RETCODE_OK if a pointer to a sample is successfully obtained. 

### ReturnCode_t discard_loan(void \*&sample)

Discards a loaned sample pointer. 

See the description on [loan_sample](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1aad64a30e5d44ab10acffc543e6dc9cb3) for how and when to call this method.

* **Parameters:**
  **sample** – **[inout]** Pointer to the previously loaned sample.
* **Returns:**
  RETCODE_ILLEGAL_OPERATION when the data type does not support loans. 
* **Returns:**
  RETCODE_NOT_ENABLED if the writer has not been enabled. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the pointer does not correspond to a loaned sample. 
* **Returns:**
  RETCODE_OK if the loan is successfully discarded. 

### ReturnCode_t get_sending_locators(rtps::[LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locators) const

Get the list of locators from which this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) may send data. 

* **Parameters:**
  **locators** – **[out]** LocatorList where the list of locators will be stored.
* **Returns:**
  NOT_ENABLED if the reader has not been enabled. 
* **Returns:**
  OK if a list of locators is returned. 

### ReturnCode_t wait_for_acknowledgments(const void \*const instance, const InstanceHandle_t &handle, const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &max_wait)

Block the current thread until the writer has received the acknowledgment corresponding to the given instance. Operations performed on the same instance while the current thread is waiting will not be taken into consideration, i.e. this method may return `RETCODE_OK` with those operations unacknowledged.

* **Parameters:**
  * **instance** – Sample used to deduce instance’s key in case of `handle` parameter is HANDLE_NIL. 
  * **handle** – Instance handle of the data. 
  * **max_wait** – Maximum blocking time for this operation.
* **Returns:**
  RETCODE_NOT_ENABLED if the writer has not been enabled. 
* **Returns:**
  RETCODE_BAD_PARAMETER if `instance` is not a valid pointer. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the topic does not have a key, the key is unknown to the writer, or the key is not consistent with `handle`. 
* **Returns:**
  RETCODE_OK if the [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) received the acknowledgments before the time expired. 
* **Returns:**
  RETCODE_TIMEOUT otherwise. 

### ReturnCode_t get_publication_builtin_topic_data(PublicationBuiltinTopicData &publication_data) const

Retrieve the publication data discovery information.

* **Parameters:**
  **publication_data** – **[out]** The publication data discovery information.
* **Returns:**
  NOT_ENABLED if the writer has not been enabled. 
* **Returns:**
  OK if the publication data is returned. 

### ReturnCode_t set_sample_prefilter(std::shared_ptr<[IContentFilter](../topic/icontentfilter.md#_CPPv4N8eprosima7fastdds3dds14IContentFilterE)> prefilter)

Set a sample prefilter to be used. This filter is always evaluated before sending the sample to any [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and prior to any content filtering. Passing a nullptr disables prefiltering. 

#### NOTE
Prefiltering is currently incompatible with DataSharing. 

* **Parameters:**
  **prefilter** – The prefilter to be set.
* **Returns:**
  RETCODE_OK if the prefilter is set correctly.

### ReturnCode_t set_related_datareader(const [DataReader](../subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*related_reader)

This operation sets the key of the [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that is related to this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). This is used to establish a relationship between a [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and a [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) in the context of RPC over DDS.

#### WARNING
This operation is only valid if the entity is not enabled.

* **Parameters:**
  **related_reader** – **[in]** Pointer to the [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to set as related.
* **Returns:**
  RETCODE_OK if the key is set successfully. 
* **Returns:**
  RETCODE_ILLEGAL_OPERATION if this entity is enabled. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the entity does not belong to the same participant. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the provided GUID is unknown or the pointer is not valid. 

### ReturnCode_t set_type_support_context(const std::shared_ptr<[TopicDataType](../topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)::[Context](../topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context)

Set the type support context to be used when serializing data for this [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). Cannot be called on an enabled [DataWriter](#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

* **Parameters:**
  **context** – **[in]** Shared pointer to the context to be used when serializing data.
* **Returns:**
  RETCODE_OK if the context is set successfully. RETCODE_ILLEGAL_OPERATION if this entity is enabled.
