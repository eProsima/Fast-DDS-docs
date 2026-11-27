<a id="python-api-pim-datawriter"></a>

<a id="datawriter"></a>

# DataWriter

### *class* fastdds.DataWriter(\*args, \*\*kwargs)

Class DataWriter, contains the actual implementation of the behaviour of the DataWriter.

#### LoanInitializationKind_CONSTRUCTED_LOAN_INITIALIZATION *= 2*

Use in-place constructor initialization.

This will call the constructor of the data type over the memory space being
returned by ‘loan_sample’.

#### LoanInitializationKind_NO_LOAN_INITIALIZATION *= 0*

Do not perform initialization of sample.

This is the default initialization scheme of loaned samples.
It is the fastest scheme, but implies the user should take care of writing
every field on the data type before calling ‘write’ on the loaned sample.

#### LoanInitializationKind_ZERO_LOAN_INITIALIZATION *= 1*

Initialize all memory with zero-valued bytes.

The contents of the loaned sample will be zero-initialized upon return of
‘loan_sample’.

#### assert_liveliness()

This operation manually asserts the liveliness of the DataWriter. This is used in combination with the
LivelinessQosPolicy to indicate to the Service that the entity remains active.
This operation need only be used if the LIVELINESS setting is either MANUAL_BY_PARTICIPANT or MANUAL_BY_TOPIC.
Otherwise, it has no effect.

Notes: Writing data via the write operation on a DataWriter asserts liveliness on the DataWriter itself and its
DomainParticipant. Consequently the use of assert_liveliness is only needed if the application is not writing data
regularly.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if asserted, RETCODE_ERROR otherwise

#### clear_history(\*args)

#### dispose(data, handle)

This operation requests the middleware to delete the data (the actual deletion is postponed until there is no
more use for that data in the whole system). In general, applications are made aware of the deletion by means of
operations on the DataReader objects that already knew that instance. This operation does not modify the value of
the instance. The instance parameter is passed just for the purposes of identifying the instance.
When this operation is used, the Service will automatically supply the value of the source_timestamp that is made
available to DataReader objects by means of the source_timestamp attribute inside the SampleInfo. The constraints
on the values of the handle parameter and the corresponding error behavior are the same specified for the
unregister_instance operation.

* **Parameters:**
  * **[****in****]** – data Sample used to deduce instance’s key in case of handle parameter is HANDLE_NIL.
  * **[****in****]** – handle InstanceHandle of the data
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the handle introduced does not match with the one associated to the data,
  RETCODE_OK if the data is correctly sent and RETCODE_ERROR otherwise.

#### dispose_w_timestamp(instance, handle, timestamp)

This operation performs the same functions as ‘dispose’ except that the application provides the value
for the ‘source_timestamp’ that is made available to
DataReader objects by means of the ‘source_timestamp’
attribute inside the SampleInfo.

The constraints on the values of the `handle` parameter and the corresponding error behavior are the same
specified for the ‘dispose’ operation.

This operation may return RETCODE_PRECONDITION_NOT_MET and RETCODE_BAD_PARAMETER under the same circumstances
described for the ‘dispose’ operation.

This operation may return RETCODE_TIMEOUT and RETCODE_OUT_OF_RESOURCES under the same circumstances described
for the ‘write’ operation.

* **Parameters:**
  * **instance** (*void*) – Sample used to deduce instance’s key in case of handle parameter is HANDLE_NIL.
  * **handle** (`InstanceHandle_t`) – Instance’s key to be disposed.
  * **timestamp** (`Time_t`) – Time_t used to set the source_timestamp.
* **Return type:**
  int
* **Returns:**
  FASTDDS_EXPORTED_API

#### enable()

This operation enables the DataWriter

* **Return type:**
  int
* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the Publisher creating this
  DataWriter is not enabled.

#### get_instance_handle()

Returns the DataWriter’s InstanceHandle

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  Copy of the DataWriter InstanceHandle

#### get_key_value(key_holder, handle)

This operation can be used to retrieve the instance key that corresponds to an
‘instance_handle’.
The operation will only fill the fields that form the key inside the key_holder instance.

This operation may return BAD_PARAMETER if the InstanceHandle_t handle does not correspond to an existing
data-object known to the DataWriter. If the implementation is not able to check invalid handles then the result
in this situation is unspecified.

* **Parameters:**
  * **[****in****,****out****]** – key_holder  Sample where the key fields will be returned.
  * **[****in****]** – handle          Handle to the instance to retrieve the key values from.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### get_listener()

Retrieves the listener for this DataWriter.

* **Return type:**
  [`DataWriterListener`](datawriterlistener.md#fastdds.DataWriterListener)
* **Returns:**
  Pointer to the DataWriterListener

#### get_liveliness_lost_status(status)

Returns the liveliness lost status

* **Parameters:**
  **status** (`LivelinessLostStatus`) – Liveliness lost status struct
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_matched_subscription_data(subscription_data, subscription_handle)

Retrieves in a subscription associated with the DataWriter

* **Parameters:**
  * **[****out****]** – subscription_data subscription data struct
  * **subscription_handle** (`InstanceHandle_t`) – InstanceHandle_t of the subscription
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, RETCODE_BAD_PARAMETER otherwise

#### get_matched_subscriptions(subscription_handles)

Fills the given vector with the InstanceHandle_t of matched DataReaders

* **Parameters:**
  **[****out****]** – subscription_handles Vector where the InstanceHandle_t are returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, RETCODE_ERROR otherwise

#### get_offered_deadline_missed_status(status)

Returns the offered deadline missed status

* **Parameters:**
  **[****out****]** – status Deadline missed status struct
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_offered_incompatible_qos_status(status)

Returns the offered incompatible qos status

* **Parameters:**
  **[****out****]** – status Offered incompatible qos status struct
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_publication_builtin_topic_data(publication_data)

Retrieve the publication data discovery information.

* **Parameters:**
  **[****out****]** – publication_data The publication data discovery information.
* **Return type:**
  int
* **Returns:**
  NOT_ENABLED if the writer has not been enabled.
* **Return type:**
  int
* **Returns:**
  OK if the publication data is returned.

#### get_publication_matched_status(status)

Returns the publication matched status

* **Parameters:**
  **[****out****]** – status publication matched status struct
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_publisher()

Getter for the Publisher that creates this DataWriter

* **Return type:**
  [`Publisher`](publisher_class.md#fastdds.Publisher)
* **Returns:**
  Pointer to the Publisher

#### get_qos(\*args)

*Overload 1:*

Retrieves the DataWriterQos for this DataWriter.

* **Return type:**
  [`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)
* **Returns:**
  Reference to the current DataWriterQos

<br/>

*Overload 2:*

Fills the DataWriterQos with the values of this DataWriter.

* **Parameters:**
  **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – DataWriterQos object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_sending_locators(locators)

Get the list of locators from which this DataWriter may send data.

* **Parameters:**
  **[****out****]** – locators  LocatorList where the list of locators will be stored.
* **Return type:**
  int
* **Returns:**
  NOT_ENABLED if the reader has not been enabled.
* **Return type:**
  int
* **Returns:**
  OK if a list of locators is returned.

#### get_topic()

Retrieves the topic for this DataWriter.

* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the associated Topic

#### get_type()

Get data type associated to the DataWriter

* **Return type:**
  [`TypeSupport`](../topic/typesupport.md#fastdds.TypeSupport)
* **Returns:**
  Copy of the TypeSupport

#### guid()

Returns the DataWriter’s GUID

* **Return type:**
  `GUID_t`
* **Returns:**
  Reference to the DataWriter GUID

#### loan_sample(\*args)

Get a pointer to the internal pool where the user could directly write.

This method can only be used on a DataWriter for a plain data type. It will provide the
user with a pointer to an internal buffer where the data type can be prepared for sending.

When using NO_LOAN_INITIALIZATION on the initialization parameter, which is the default,
no assumptions should be made on the contents where the pointer points to, as it may be an
old pointer being reused. See ‘LoanInitializationKind’ for more details.

Once the sample has been prepared, it can then be published by calling ‘write’.
After a successful call to ‘write’, the middleware takes ownership of the loaned pointer again,
and the user should not access that memory again.

If, for whatever reason, the sample is not published, the loan can be returned by calling
‘discard_loan’.

* **Parameters:**
  * **[****out****]** – sample          Pointer to the sample on the internal pool.
  * **[****in****]** – initialization  How to initialize the loaned sample.
* **Return type:**
  int
* **Returns:**
  RETCODE_ILLEGAL_OPERATION when the data type does not support loans.
* **Return type:**
  int
* **Returns:**
  RETCODE_NOT_ENABLED if the writer has not been enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_OUT_OF_RESOURCES if the pool has been exhausted.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if a pointer to a sample is successfully obtained.

#### lookup_instance(instance)

NOT YET IMPLEMENTED

Takes as a parameter an instance and returns a handle that can be used in subsequent operations that accept an
instance handle as an argument. The instance parameter is only used for the purpose of examining the fields that
define the key.

* **Parameters:**
  **[****in****]** – instance Data pointer to the sample
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  handle of the given instance

#### register_instance(instance)

Informs that the application will be modifying a particular instance.
It gives an opportunity to the middleware to pre-configure itself to improve performance.

* **Parameters:**
  **[****in****]** – instance Sample used to get the instance’s key.
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  Handle containing the instance’s key.
  This handle could be used in successive write or dispose operations.
  In case of error, HANDLE_NIL will be returned.

#### register_instance_w_timestamp(instance, timestamp)

This operation performs the same function as register_instance and can be used instead of
‘register_instance’ in the cases where the application desires to specify the value for the
‘source_timestamp’.
The ‘source_timestamp’ potentially affects the relative
order in which readers observe events from multiple writers. See the QoS policy
‘DESTINATION_ORDER’.

This operation may block and return RETCODE_TIMEOUT under the same circumstances described for the ‘write’
operation.

This operation may return RETCODE_OUT_OF_RESOURCES under the same circumstances described for the
‘write’ operation.

* **Parameters:**
  * **instance** (*void*) – Sample used to get the instance’s key.
  * **timestamp** (`Time_t`) – Time_t used to set the source_timestamp.
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  Handle containing the instance’s key.

#### set_listener(\*args)

#### set_qos(qos)

Establishes the DataWriterQos for this DataWriter.

* **Parameters:**
  **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – DataWriterQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not
  self consistent and RETCODE_OK if the qos is changed correctly.

#### set_related_datareader(related_reader)

This operation sets the key of the DataReader that is related to this DataWriter.
This is used to establish a relationship between a DataReader and a DataWriter
in the context of RPC over DDS.

Warning: This operation is only valid if the entity is not enabled.

* **Parameters:**
  **[****in****]** – related_reader Pointer to the DataReader to set as related.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the key is set successfully.
* **Return type:**
  int
* **Returns:**
  RETCODE_ILLEGAL_OPERATION if this entity is enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the entity does not belong to the same participant.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the provided GUID is unknown
  or the pointer is not valid.

#### set_sample_prefilter(prefilter)

> Set a sample prefilter to be used. This filter is always
> evaluated before sending the sample to any DataReader and prior to
> any content filtering.
> Passing a nullptr disables prefiltering.
* **Parameters:**
  **prefilter** (*std::shared_ptr< eprosima::fastdds::dds::IContentFilter >*) – The prefilter to be set.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the prefilter is set correctly.

Notes: Prefiltering is currently incompatible with DataSharing.

#### set_type_support_context(context)

Set the type support context to be used when serializing data for this DataWriter.
: Cannot be called on an enabled DataWriter.

* **Parameters:**
  **[****in****]** – context Shared pointer to the context to be used when serializing data.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the context is set successfully.
  RETCODE_ILLEGAL_OPERATION if this entity is enabled.

#### *property* thisown

The membership flag

#### unregister_instance(instance, handle)

This operation reverses the action of register_instance.
It should only be called on an instance that is currently registered.
Informs the middleware that the DataWriter is not intending to modify any more of that data instance.
Also indicates that the middleware can locally remove all information regarding that instance.

* **Parameters:**
  * **[****in****]** – instance Sample used to deduce instance’s key in case of handle parameter is HANDLE_NIL.
  * **[****in****]** – handle Instance’s key to be unregistered.
* **Return type:**
  int
* **Returns:**
  Returns the operation’s result.
  If the operation finishes successfully, RETCODE_OK is returned.

#### unregister_instance_w_timestamp(instance, handle, timestamp)

This operation performs the same function as ‘unregister_instance’ and can be used instead of
‘unregister_instance’ in the cases where the application desires to specify the value for the
‘source_timestamp’.
The ‘source_timestamp’ potentially affects the relative
order in which readers observe events from multiple writers. See the QoS policy
‘DESTINATION_ORDER’.

The constraints on the values of the `handle` parameter and the corresponding error behavior are the same
specified for the ‘unregister_instance’ operation.

This operation may block and return RETCODE_TIMEOUT under the same circumstances described for the write
operation

* **Parameters:**
  * **instance** (*void*) – Sample used to deduce instance’s key in case of handle parameter is HANDLE_NIL.
  * **handle** (`InstanceHandle_t`) – Instance’s key to be unregistered.
  * **timestamp** (`Time_t`) – Time_t used to set the source_timestamp.
* **Return type:**
  int
* **Returns:**
  Handle containing the instance’s key.

#### wait_for_acknowledgments(\*args)

*Overload 1:*

Waits the current thread until all writers have received their acknowledgments.

* **Parameters:**
  **max_wait** (`Duration_t`) – Maximum blocking time for this operation
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the DataWriter receive the acknowledgments before the time expires and RETCODE_ERROR otherwise

<br/>

*Overload 2:*

Block the current thread until the writer has received the acknowledgment corresponding to the given instance.
Operations performed on the same instance while the current thread is waiting will not be taken into
consideration, i.e. this method may return RETCODE_OK with those operations unacknowledged.

* **Parameters:**
  * **instance** (*void*) – Sample used to deduce instance’s key in case of handle parameter is HANDLE_NIL.
  * **handle** (`InstanceHandle_t`) – Instance handle of the data.
  * **max_wait** (`Duration_t`) – Maximum blocking time for this operation.
* **Return type:**
  int
* **Returns:**
  RETCODE_NOT_ENABLED if the writer has not been enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if instance is not a valid pointer.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the topic does not have a key, the key is unknown to the writer,
  or the key is not consistent with handle.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the DataWriter received the acknowledgments before the time expired.
* **Return type:**
  int
* **Returns:**
  RETCODE_TIMEOUT otherwise.

#### write(\*args)

*Overload 1:*

Write data to the topic.

* **Parameters:**
  **data** (*void*) – Pointer to the data
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the data is correctly sent or a ReturnCode related to the specific error otherwise.

<br/>

*Overload 2:*

Write data with params to the topic.

* **Parameters:**
  * **data** (*void*) – Pointer to the data
  * **params** (`WriteParams`) – Extra write parameters.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the data is correctly sent or a ReturnCode related to the specific error otherwise.

<br/>

*Overload 3:*

Write data with handle.

The special value HANDLE_NIL can be used for the parameter handle.This indicates that the identity of the
instance should be automatically deduced from the instance_data (by means of the key).

* **Parameters:**
  * **data** (*void*) – Pointer to the data
  * **handle** (`InstanceHandle_t`) – InstanceHandle_t.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the handle introduced does not match with the one associated to the data,
  RETCODE_OK if the data is correctly sent and RETCODE_ERROR otherwise.

#### write_w_timestamp(data, handle, timestamp)

This operation performs the same function as write except that it also provides the value for the
‘source_timestamp’ that is made available to DataReader
objects by means of the ‘eprosima::fastdds::dds::SampleInfo::source_timestamp’ attribute “source_timestamp”
inside the SampleInfo.
The constraints on the values of the `handle` parameter and the corresponding error behavior are the same
specified for the ‘write’ operation. This operation may block and return RETCODE_TIMEOUT under the same
circumstances described for the ‘write’ operation.
This operation may return RETCODE_OUT_OF_RESOURCES, RETCODE_PRECONDITION_NOT_MET or RETCODE_BAD_PARAMETER under
the same circumstances described for the write operation.

* **Parameters:**
  * **data** (*void*) – Pointer to the data
  * **handle** (`InstanceHandle_t`) – InstanceHandle_t
  * **timestamp** (`Time_t`) – Time_t used to set the source_timestamp.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.
