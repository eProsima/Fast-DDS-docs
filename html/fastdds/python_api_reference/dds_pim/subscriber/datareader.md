<a id="python-api-pim-datareader"></a>

<a id="datareader"></a>

# DataReader

### *class* fastdds.DataReader(\*args, \*\*kwargs)

Class DataReader, contains the actual implementation of the behaviour of the Subscriber.

#### create_querycondition(sample_states, view_states, instance_states, query_expression, query_parameters)

This operation creates a QueryCondition. The returned QueryCondition will be attached and belong to the
DataReader.

* **Parameters:**
  * **[****in****]** – sample_states    Only data samples with `sample_state` matching one of these will trigger the created condition.
  * **[****in****]** – view_states      Only data samples with `view_state` matching one of these will trigger the created condition.
  * **[****in****]** – instance_states  Only data samples with `instance_state` matching one of these will trigger the created condition.
  * **[****in****]** – query_expression Only data samples matching this query will trigger the created condition.
  * **[****in****]** – query_parameters Value of the parameters on the query expression.
* **Return type:**
  eprosima::fastdds::dds::QueryCondition
* **Returns:**
  pointer to the created QueryCondition, nullptr in case of error.

#### create_readcondition(sample_states, view_states, instance_states)

This operation creates a ReadCondition. The returned ReadCondition will be attached and belong to the
DataReader.

* **Parameters:**
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will trigger the created condition.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will trigger the created condition.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will trigger the created condition.
* **Return type:**
  `ReadCondition`
* **Returns:**
  pointer to the created ReadCondition, nullptr in case of error.

#### delete_contained_entities()

This operation deletes all the entities that were created by means of the “create” operations on the DataReader.
That is, it deletes all contained ReadCondition and QueryCondition objects.

The operation will return PRECONDITION_NOT_MET if the any of the contained entities is in a state where it cannot
be deleted.

* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### delete_readcondition(a_condition)

This operation deletes a ReadCondition attached to the DataReader.

* **Parameters:**
  **a_condition** (`ReadCondition`) – pointer to a ReadCondition belonging to the DataReader
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### enable()

This operation enables the DataReader.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the Subscriber creating this
  DataReader is not enabled.

#### get_first_untaken_info(info)

Returns information about the first untaken sample. This method is meant to be called prior to
a read() or take() operation as it does not modify the status condition of the entity.

* **Parameters:**
  **[****out****]** – info Pointer to a SampleInfo_t structure to store first untaken sample information.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if sample info was returned. RETCODE_NO_DATA if there is no sample to take.

#### get_instance_handle()

Getter for the associated InstanceHandle.

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  Copy of the InstanceHandle

#### get_key_value(key_holder, handle)

NOT YET IMPLEMENTED

This operation can be used to retrieve the instance key that corresponds to an `instance_handle`. The operation
will only fill the fields that form the key inside the key_holder instance.

This operation may return BAD_PARAMETER if the InstanceHandle_t a_handle does not correspond to an existing
data-object known to the DataReader. If the implementation is not able to check invalid handles then the result
in this situation is unspecified.

* **Parameters:**
  * **[****in****,****out****]** – key_holder
  * **[****in****]** – handle
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_listener()

Getter for the DataReaderListener

* **Return type:**
  [`DataReaderListener`](datareaderlistener.md#fastdds.DataReaderListener)
* **Returns:**
  Pointer to the DataReaderListener

#### get_listening_locators(locators)

Get the list of locators on which this DataReader is listening.

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

#### get_liveliness_changed_status(status)

Get the liveliness changed status.

* **Parameters:**
  **[****out****]** – status LivelinessChangedStatus object where the status is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_matched_publication_data(publication_data, publication_handle)

Retrieves in a publication associated with the DataWriter

* **Parameters:**
  * **[****out****]** – publication_data publication data struct
  * **publication_handle** (`InstanceHandle_t`) – InstanceHandle_t of the publication
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_matched_publications(publication_handles)

Fills the given vector with the InstanceHandle_t of matched DataReaders

* **Parameters:**
  **[****out****]** – publication_handles Vector where the InstanceHandle_t are returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_qos(\*args)

*Overload 1:*

Getter for the DataReaderQos.

* **Return type:**
  [`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)
* **Returns:**
  Pointer to the DataReaderQos.

<br/>

*Overload 2:*

Getter for the DataReaderQos.

* **Parameters:**
  **[****in****]** – qos DataReaderQos where the qos is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_requested_deadline_missed_status(status)

Get the requested deadline missed status.

* **Return type:**
  int
* **Returns:**
  The deadline missed status.

#### get_requested_incompatible_qos_status(status)

Get the requested incompatible qos status.

* **Parameters:**
  **[****out****]** – status Requested incompatible qos status.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_sample_lost_status(status)

Get the SAMPLE_LOST communication status

* **Parameters:**
  **[****out****]** – status SampleLostStatus object where the status is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_sample_rejected_status(status)

Get the SAMPLE_REJECTED communication status

* **Parameters:**
  **[****out****]** – status SampleRejectedStatus object where the status is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_subscriber()

Getter for the Subscriber
:rtype: [`Subscriber`](subscriber_class.md#fastdds.Subscriber)
:return: Subscriber pointer

#### get_subscription_builtin_topic_data(subscription_data)

Retrieve the subscription data discovery information.

* **Parameters:**
  **[****out****]** – subscription_data The subscription data discovery information.
* **Return type:**
  int
* **Returns:**
  NOT_ENABLED if the reader has not been enabled.
* **Return type:**
  int
* **Returns:**
  OK if the subscription data is returned.

#### get_subscription_matched_status(status)

Returns the subscription matched status

* **Parameters:**
  **[****out****]** – status subscription matched status struct
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_topicdescription()

Get TopicDescription.

* **Return type:**
  [`TopicDescription`](../topic/topicdescription.md#fastdds.TopicDescription)
* **Returns:**
  TopicDescription pointer.

#### get_unread_count(\*args)

*Overload 1:*

Get the number of samples pending to be read.
The number includes samples that may not yet be available to be read or taken by the user, due to samples
being received out of order.

* **Return type:**
  int
* **Returns:**
  the number of samples on the reader history that have never been read.

<br/>

*Overload 2:*

Get the number of samples pending to be read.

* **Parameters:**
  **mark_as_read** (*boolean*) – Whether the unread samples should be marked as read or not.
* **Return type:**
  int
* **Returns:**
  the number of samples on the reader history that have never been read.

#### guid(\*args)

*Overload 1:*

Get associated GUID.

* **Return type:**
  `GUID_t`
* **Returns:**
  Associated GUID

<br/>

*Overload 2:*

Get associated GUID.

* **Return type:**
  `GUID_t`
* **Returns:**
  Associated GUID

#### is_sample_valid(data, info)

Checks whether a loaned sample is still valid or is corrupted.
Calling this method on a sample which has not been loaned, or one for which the loan has been returned
yields undefined behavior.

* **Parameters:**
  * **data** (*void*) – Pointer to the sample data to check
  * **info** ([`SampleInfo`](sampleinfo.md#fastdds.SampleInfo)) – Pointer to the SampleInfo related to `data`
* **Return type:**
  boolean
* **Returns:**
  true if the sample is valid

#### lookup_instance(instance)

Takes as a parameter an instance and returns a handle that can be used in subsequent operations that accept an
instance handle as an argument. The instance parameter is only used for the purpose of examining the fields that
define the key.

* **Parameters:**
  **[****in****]** – instance Data pointer to the sample
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  handle of the given `instance`.
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  HANDLE_NIL if `instance` is nullptr.
* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  HANDLE_NIL if there is no instance on the DataReader’s history with the same key as `instance`.

#### read(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of Data values from the DataReader. The caller can limit the size
of the returned collection with the `max_samples` parameter.

The properties of the `data_values` collection and the setting of the ‘PresentationQosPolicy’ may
impose further limits on the size of the returned ‘list.’

1. If ‘PresentationQosPolicy::access_scope’ is ‘INSTANCE_PRESENTATION_QOS’, then the returned
   collection is a ‘list’ where samples belonging to the same data-instance are consecutive.
2. If ‘PresentationQosPolicy::access_scope’ is ‘TOPIC_PRESENTATION_QOS’ and
   ‘PresentationQosPolicy::ordered_access’ is set to `false`, then the returned collection is a
   ‘list’ where samples belonging to the same data-instance are consecutive.
3. If ‘PresentationQosPolicy::access_scope’ is ‘TOPIC_PRESENTATION_QOS’ and
   ‘PresentationQosPolicy::ordered_access’ is set to `true`, then the returned collection is a
   ‘list’ where samples belonging to the same instance may or may not be consecutive. This is because to
   preserve order it may be necessary to mix samples from different instances.
4. If ‘PresentationQosPolicy::access_scope’ is ‘GROUP_PRESENTATION_QOS’ and
   ‘PresentationQosPolicy::ordered_access’ is set to `false`, then the returned collection is a
   ‘list’ where samples belonging to the same data instance are consecutive.
5. If ‘PresentationQosPolicy::access_scope’ is ‘GROUP_PRESENTATION_QOS’ and
   ‘PresentationQosPolicy::ordered_access’ is set to `true`, then the returned collection contains at
   most one sample. The difference in this case is due to the fact that it is required that the application
   is able to read samples belonging to different DataReader objects in a specific order.

In any case, the relative order between the samples of one instance is consistent with the
‘DestinationOrderQosPolicy’:

- If ‘DestinationOrderQosPolicy::kind’ is ‘BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS’, samples
  belonging to the same instances will appear in the relative order in which there were received (FIFO,
  earlier samples ahead of the later samples).
- If ‘DestinationOrderQosPolicy::kind’ is ‘BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS’, samples
  belonging to the same instances will appear in the relative order implied by the source_timestamp (FIFO,
  smaller values of source_timestamp ahead of the larger values).

The actual number of samples returned depends on the information that has been received by the middleware
as well as the ‘HistoryQosPolicy’, ‘ResourceLimitsQosPolicy’, and
‘ReaderResourceLimitsQos’:

- In the case where the ‘HistoryQosPolicy::kind’ is KEEP_LAST_HISTORY_QOS, the call will return at most
  ‘HistoryQosPolicy::depth’ samples per instance.
- The maximum number of samples returned is limited by ‘ResourceLimitsQosPolicy::max_samples’, and by
  ‘ReaderResourceLimitsQos::max_samples_per_read’.
- For multiple instances, the number of samples returned is additionally limited by the product
  (‘ResourceLimitsQosPolicy::max_samples_per_instance’ \* ‘ResourceLimitsQosPolicy::max_instances)’.
- If ReaderResourceLimitsQos::sample_infos_allocation has a maximum limit, the number of samples returned
  may also be limited if insufficient ‘SampleInfo’ resources are available.

If the operation succeeds and the number of samples returned has been limited (by means of a maximum limit,
as listed above, or insufficient ‘SampleInfo’ resources), the call will complete successfully and provide
those samples the reader is able to return. The user may need to make additional calls, or return outstanding
loaned buffers in the case of insufficient resources, in order to access remaining samples.

In addition to the collection of samples, the read operation also uses a collection of ‘SampleInfo’
structures (`sample_infos)`.

The initial (input) properties of the `data_values` and `sample_infos` collections will determine the
precise behavior of this operation. For the purposes of this description the collections are modeled as having
three properties:

- the current length (`len`, see ‘LoanableCollection::length())’
- the maximum length (`max_len`, see ‘LoanableCollection::maximum())’
- whether the collection container owns the memory of the elements within
  (`owns`, see ‘LoanableCollection::has_ownership())’

The initial (input) values of the `len`, `max_len`, and `owns` properties for the `data_values` and
`sample_infos` collections govern the behavior of the read operation as specified by the following rules:

1. The values of `len`, `max_len`, and `owns` for the two collections must be identical. Otherwise read
   will fail with RETCODE_PRECONDITION_NOT_MET.
2. On successful output, the values of `len`, `max_len`, and `owns` will be the same for both collections.
3. If the input  max_len == 0 , then the `data_values` and `sample_infos` collections will be
   filled with elements that are ‘loaned’ by the DataReader. On output, `owns` will be `false`, `len` will
   be set to the number of values returned, and `max_len` will be set to a value
   verifying  max_len >= len . The use of this variant allows for zero-copy access to the data and the
   application will need to return the loan to the DataReader using the ‘return_loan’ operation.
4. If the input  max_len > 0  and the input  owns == false , then the read operation will
   fail with RETCODE_PRECONDITION_NOT_MET. This avoids the potential hard-to-detect memory leaks caused by an
   application forgetting to return the loan.
5. If input  max_len > 0  and the input  owns == true , then the read operation will copy
   the Data values and SampleInfo values into the elements already inside the collections. On output, `owns`
   will be `true`, `len` will be set to the number of values copied, and `max_len` will remain unchanged.
   The use of this variant forces a copy but the application can control where the copy is placed and the
   application will not need to return the loan. The number of samples copied depends on the values of
   `max_len` and `max_samples`:
   - If  max_samples == LENGTH_UNLIMITED , then at most `max_len` values will be copied. The use of
     this variant lets the application limit the number of samples returned to what the sequence can
     accommodate.
   - If  max_samples <= max_len , then at most `max_samples` values will be copied. The use of this
     variant lets the application limit the number of samples returned to fewer that what the sequence can
     accommodate.
   - If  max_samples > max_len , then the read operation will fail with RETCODE_PRECONDITION_NOT_MET.
     This avoids the potential confusion where the application expects to be able to access up to
     `max_samples`, but that number can never be returned, even if they are available in the DataReader,
     because the output sequence cannot accommodate them.

As described above, upon return the `data_values` and `sample_infos` collections may contain elements
‘loaned’ from the DataReader. If this is the case, the application will need to use the ‘return_loan’
operation to return the loan once it is no longer using the Data in the collection. Upon return from
‘return_loan’, the collection will have  max_len == 0  and  owns == false .

The application can determine whether it is necessary to return the loan or not based on the state of the
collections when the read operation was called, or by accessing the `owns` property. However, in many cases
it may be simpler to always call ‘return_loan’, as this operation is harmless (i.e., leaves all elements
unchanged) if the collection does not have a loan.

On output, the collection of Data values and the collection of SampleInfo structures are of the same length
and are in a one-to-one correspondence. Each SampleInfo provides information, such as the `source_timestamp`,
the `sample_state`, `view_state`, and `instance_state`, etc., about the corresponding sample.

Some elements in the returned collection may not have valid data. If the `instance_state` in the SampleInfo is
‘NOT_ALIVE_DISPOSED_INSTANCE_STATE’ or ‘NOT_ALIVE_NO_WRITERS_INSTANCE_STATE’, then the last sample for
that instance in the collection, that is, the one whose SampleInfo has  sample_rank == 0  does not
contain valid data. Samples that contain no data do not count towards the limits imposed by the
‘ResourceLimitsQosPolicy’.

The act of reading a sample changes its `sample_state` to ‘READ_SAMPLE_STATE’. If the sample belongs
to the most recent generation of the instance, it will also set the `view_state` of the instance to be
‘NOT_NEW_VIEW_STATE’. It will not affect the `instance_state` of the instance.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

*Important*: If the samples “returned” by this method are loaned from the middleware (see ‘take’
for more information on memory loaning), it is important that their contents not be changed. Because the
memory in which the data is stored belongs to the middleware, any modifications made to the data will be
seen the next time the same samples are read or taken; the samples will no longer reflect the state that
was received from the network.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described above.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### read_instance(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of data values from the DataReader. The behavior is identical to
‘read’, except that all samples returned belong to the single specified instance whose handle is
`a_handle`.

Upon successful completion, the data collection will contain samples all belonging to the same instance.
The corresponding ‘SampleInfo’ verifies ‘SampleInfo::instance_handle’ == `a_handle`.

This operation is semantically equivalent to the ‘read’ operation, except in building the collection.
The DataReader will check that the sample belongs to the specified instance and otherwise it will not place
the sample in the returned collection.

The behavior of this operation follows the same rules as the ‘read’ operation regarding the pre-conditions and
post-conditions for the `data_values` and `sample_infos`. Similar to ‘read’, this operation may ‘loan’
elements to the output collections, which must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – a_handle        The specified instance to return samples for. The method will fail with
    RETCODE_BAD_PARAMETER if the handle does not correspond to an existing
    data-object known to the DataReader.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### read_next_instance(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of data values from the DataReader where all the samples belong to a
single instance. The behavior is similar to ‘read_instance’, except that the actual instance is not
directly specified. Rather, the samples will all belong to the ‘next’ instance with `instance_handle`
‘greater’ than the specified ‘previous_handle’ that has available samples.

This operation implies the existence of a total order ‘greater-than’ relationship between the instance
handles. The specifics of this relationship are not all important and are implementation specific. The
important thing is that, according to the middleware, all instances are ordered relative to each other.
This ordering is between the instance handles, and should not depend on the state of the instance (e.g.
whether it has data or not) and must be defined even for instance handles that do not correspond to instances
currently managed by the DataReader. For the purposes of the ordering, it should be ‘as if’ each instance
handle was represented as an integer.

The behavior of this operation is ‘as if’ the DataReader invoked ‘read_instance’, passing the smallest
`instance_handle` among all the ones that: (a) are greater than `previous_handle`, and (b) have available
samples (i.e. samples that meet the constraints imposed by the specified states).

The special value ‘HANDLE_NIL’ is guaranteed to be ‘less than’ any valid `instance_handle`. So the use
of the parameter value `previous_handle` == ‘HANDLE_NIL’ will return the samples for the instance which
has the smallest `instance_handle` among all the instances that contain available samples.

This operation is intended to be used in an application-driven iteration, where the application starts by
passing `previous_handle` == ‘HANDLE_NIL’, examines the samples returned, and then uses the
`instance_handle` returned in the ‘SampleInfo’ as the value of the `previous_handle` argument to the
next call to ‘read_next_instance’. The iteration continues until ‘read_next_instance’ fails with
RETCODE_NO_DATA.

Note that it is possible to call the ‘read_next_instance’ operation with a `previous_handle` that does not
correspond to an instance currently managed by the DataReader. This is because as stated earlier the
‘greater-than’ relationship is defined even for handles not managed by the DataReader. One practical situation
where this may occur is when an application is iterating through all the instances, takes all the samples of a
‘NOT_ALIVE_NO_WRITERS_INSTANCE_STATE’ instance, returns the loan (at which point the instance information
may be removed, and thus the handle becomes invalid), and tries to read the next instance.

The behavior of this operation follows the same rules as the ‘read’ operation regarding the pre-conditions and
post-conditions for the `data_values` and `sample_infos`. Similar to ‘read’, this operation may ‘loan’
elements to the output collections, which must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – previous_handle The ‘next smallest’ instance with a value greater than this value that has
    available samples will be returned.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### read_next_instance_w_condition(data_values, sample_infos, max_samples, previous_handle, a_condition)

This operation accesses a collection of Data values from the DataReader. The behavior is identical to
‘read_next_instance’ except that all samples returned satisfy the specified condition. In other words, on
success all returned samples belong to the same instance, and the instance is the instance with
‘smallest’ `instance_handle` among the ones that verify (a) `instance_handle` >= `previous_handle` and (b) have samples
for which the specified ReadCondition evaluates to TRUE.

Similar to the operation ‘read_next_instance’ it is possible to call
‘read_next_instance_w_condition’ with a `previous_handle` that does not correspond to an instance currently
managed by the DataReader.

The behavior of the ‘read_next_instance_w_condition’ operation follows the same rules than the read operation
regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar
to read, the ‘read_next_instance_w_condition’ operation may ‘loan’ elements to the output collections which
must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – previous_handle The ‘next smallest’ instance with a value greater than this value that has
    available samples will be returned.
  * **[****in****]** – a_condition     A ReadCondition that returned `data_values` must pass
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### read_next_sample(data, info)

This operation copies the next, non-previously accessed Data value from the DataReader; the operation
also copies the corresponding SampleInfo. The implied order among the samples stored in the DataReader is the
same as for the read operation.

The read_next_sample operation is semantically equivalent to the read operation where the input Data sequence
has  max_length = 1 , the  sample_states = NOT_READ_SAMPLE_STATE ,
the  view_states = ANY_VIEW_STATE , and the  instance_states = ANY_INSTANCE_STATE .

The read_next_sample operation provides a simplified API to ‘read’ samples avoiding the need for the
application to manage sequences and specify states.

If there is no unread data in the DataReader, the operation will return RETCODE_NO_DATA and nothing is copied

* **Parameters:**
  * **[****out****]** – data Data pointer to store the sample
  * **[****out****]** – info SampleInfo pointer to store the sample information
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### read_w_condition(data_values, sample_infos, max_samples, a_condition)

This operation accesses via ‘read’ the samples that match the criteria specified in the ReadCondition.
This operation is especially useful in combination with QueryCondition to filter data samples based on the
content.

The specified ReadCondition must be attached to the DataReader; otherwise the operation will fail and return
RETCODE_PRECONDITION_NOT_MET.

In case the ReadCondition is a ‘plain’ ReadCondition and not the specialized QueryCondition, the
operation is equivalent to calling read and passing as `sample_states`, `view_states` and `instance_states`
the value of the corresponding attributes in `a_condition`. Using this operation the application can avoid
repeating the same parameters specified when creating the ReadCondition.

The samples are accessed with the same semantics as the read operation. If the DataReader has no samples that
meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned.
  * **[****in****]** – a_condition     A ReadCondition that returned `data_values` must pass
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### return_loan(data_values, sample_infos)

This operation indicates to the DataReader that the application is done accessing the collection of
`data_values` and `sample_infos` obtained by some earlier invocation of ‘read’ or ‘take’ on the
DataReader.

The `data_values` and `sample_infos` must belong to a single related ‘pair’; that is, they should correspond
to a pair returned from a single call to read or take. The `data_values` and `sample_infos` must also have
been obtained from the same DataReader to which they are returned. If either of these conditions is not met,
the operation will fail and return RETCODE_PRECONDITION_NOT_MET.

This operation allows implementations of the ‘read’ and ‘take’ operations to “loan” buffers from the
DataReader to the application and in this manner provide “zero-copy” access to the data. During the loan, the
DataReader will guarantee that the data and sample-information are not modified.

It is not necessary for an application to return the loans immediately after the read or take calls. However,
as these buffers correspond to internal resources inside the DataReader, the application should not retain them
indefinitely.

The use of the ‘return_loan’ operation is only necessary if the read or take calls “loaned” buffers to the
application. This only occurs if the `data_values` and `sample_infos` collections had  max_len == 0
at the time read or take was called. The application may also examine the `has_ownership` property of the
collection to determine if there is an outstanding loan. However, calling ‘return_loan’ on a collection that
does not have a loan is safe, has no side effects, and returns RETCODE_OK.

If the collections had a loan, upon return from return_loan the collections will have  max_len == 0 .

* **Parameters:**
  * **[****in****,****out****]** – data_values   A LoanableCollection object where the received data samples were obtained from
    an earlier invocation of read or take on this DataReader.
  * **[****in****,****out****]** – sample_infos  A SampleInfoSeq object where the received sample infos were obtained from
    an earlier invocation of read or take on this DataReader.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### set_listener(\*args)

#### set_qos(qos)

Setter for the DataReaderQos.

* **Parameters:**
  **[****in****]** – qos new value for the DataReaderQos.
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is
  not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_related_datawriter(related_writer)

This operation sets the key of the DataWriter that is related to this DataReader.
This is used to establish a relationship between a DataReader and a DataWriter
in the context of RPC over DDS.

Warning: This operation is only valid if the entity is not enabled.

* **Parameters:**
  **[****in****]** – related_writer Pointer to the DataWriter to set as related.
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

#### set_type_support_context(context)

Set the type support context to be used when deserializing data for this DataReader.
: Cannot be called on an enabled DataReader.

* **Parameters:**
  **[****in****]** – context Shared pointer to the context to be used when serializing data.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the context is set successfully.
  RETCODE_ILLEGAL_OPERATION if this entity is enabled.

#### take(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of data-samples from the DataReader and a corresponding collection of
SampleInfo structures, and ‘removes’ them from the DataReader. The operation will return either a ‘list’ of
samples or else a single sample. This is controlled by the ‘PresentationQosPolicy’ using the same logic
as for the ‘read’ operation.

The act of taking a sample removes it from the DataReader so it cannot be ‘read’ or ‘taken’ again. If the
sample belongs to the most recent generation of the instance, it will also set the `view_state` of the
instance to NOT_NEW. It will not affect the `instance_state` of the instance.

The behavior of the take operation follows the same rules than the ‘read’ operation regarding the
pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar to
‘read’, the take operation may ‘loan’ elements to the output collections which must then be returned by
means of ‘return_loan’. The only difference with ‘read’ is that, as stated, the samples returned by
take will no longer be accessible to successive calls to read or take.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### take_instance(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of data values from the DataReader and ‘removes’ them from the DataReader.

This operation has the same behavior as ‘read_instance’, except that the samples are ‘taken’ from the
DataReader such that they are no longer accessible via subsequent ‘read’ or ‘take’ operations.

The behavior of this operation follows the same rules as the ‘read’ operation regarding the pre-conditions and
post-conditions for the `data_values` and `sample_infos`. Similar to ‘read’, this operation may ‘loan’
elements to the output collections, which must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – a_handle        The specified instance to return samples for. The method will fail with
    RETCODE_BAD_PARAMETER if the handle does not correspond to an existing
    data-object known to the DataReader.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### take_next_instance(\*args)

Access a collection of data samples from the DataReader.

This operation accesses a collection of data values from the DataReader and ‘removes’ them from the DataReader.

This operation has the same behavior as ‘read_next_instance’, except that the samples are ‘taken’ from the
DataReader such that they are no longer accessible via subsequent ‘read’ or ‘take’ operations.

Similar to the operation ‘read_next_instance’, it is possible to call this operation with a
`previous_handle` that does not correspond to an instance currently managed by the DataReader.

The behavior of this operation follows the same rules as the ‘read’ operation regarding the pre-conditions and
post-conditions for the `data_values` and `sample_infos`. Similar to ‘read’, this operation may ‘loan’
elements to the output collections, which must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – previous_handle The ‘next smallest’ instance with a value greater than this value that has
    available samples will be returned.
  * **[****in****]** – sample_states   Only data samples with `sample_state` matching one of these will be returned.
  * **[****in****]** – view_states     Only data samples with `view_state` matching one of these will be returned.
  * **[****in****]** – instance_states Only data samples with `instance_state` matching one of these will be returned.
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### take_next_instance_w_condition(data_values, sample_infos, max_samples, previous_handle, a_condition)

This operation accesses a collection of Data values from the DataReader. The behavior is identical to
‘read_next_instance’ except that all samples returned satisfy the specified condition. In other words, on
success all returned samples belong to the same instance, and the instance is the instance with ‘smallest’
`instance_handle` among the ones that verify (a) `instance_handle` >= `previous_handle` and (b) have
samples for which the specified ReadCondition evaluates to TRUE.

Similar to the operation ‘read_next_instance’ it is possible to call ‘read_next_instance_w_condition’ with
a `previous_handle` that does not correspond to an instance currently managed by the DataReader.

The behavior of the ‘read_next_instance_w_condition’ operation follows the same rules than the read operation
regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar
to read, the ‘read_next_instance_w_condition’ operation may ‘loan’ elements to the output collections which
must then be returned by means of ‘return_loan’.

If the DataReader has no samples that meet the constraints, the return value will be RETCODE_NO_DATA

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are
    available, up to the limits described in the documentation for ‘read()’.
  * **[****in****]** – previous_handle The ‘next smallest’ instance with a value greater than this value that has
    available samples will be returned.
  * **[****in****]** – a_condition     A ReadCondition that returned `data_values` must pass
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### take_next_sample(data, info)

This operation copies the next, non-previously accessed Data value from the DataReader and ‘removes’ it
from the DataReader so it is no longer accessible. The operation also copies the corresponding SampleInfo.

This operation is analogous to ‘read_next_sample’ except for the fact that the sample is ‘removed’ from the
DataReader.

This operation is semantically equivalent to the ‘take’ operation where the input sequence has
: max_length = 1 , the  sample_states = NOT_READ_SAMPLE_STATE , the
  view_states = ANY_VIEW_STATE , and the  instance_states = ANY_INSTANCE_STATE .

This operation provides a simplified API to ’take’ samples avoiding the need for the application to manage
sequences and specify states.

If there is no unread data in the DataReader, the operation will return RETCODE_NO_DATA and nothing is copied.

* **Parameters:**
  * **[****out****]** – data Data pointer to store the sample
  * **[****out****]** – info SampleInfo pointer to store the sample information
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### take_w_condition(data_values, sample_infos, max_samples, a_condition)

This operation is analogous to ‘read_w_condition’ except it accesses samples via the ‘take’ operation.

The specified ReadCondition must be attached to the DataReader; otherwise the operation will fail and return
RETCODE_PRECONDITION_NOT_MET.

The samples are accessed with the same semantics as the ‘take’ operation.

This operation is especially useful in combination with QueryCondition to filter data samples based on the
content.

If the DataReader has no samples that meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **[****in****,****out****]** – data_values     A LoanableCollection object where the received data samples will be returned.
  * **[****in****,****out****]** – sample_infos    A SampleInfoSeq object where the received sample info will be returned.
  * **[****in****]** – max_samples     The maximum number of samples to be returned. If the special value
    ‘LENGTH_UNLIMITED’ is provided, as many samples will be returned as are.
  * **[****in****]** – a_condition     A ReadCondition that returned `data_values` must pass
* **Return type:**
  int
* **Returns:**
  Any of the standard return codes.

#### *property* thisown

The membership flag

#### type()

Getter for the data type.

* **Return type:**
  [`TypeSupport`](../topic/typesupport.md#fastdds.TypeSupport)
* **Returns:**
  Copy of the TypeSupport associated to the DataReader.

#### wait_for_historical_data(max_wait)

NOT YET IMPLEMENTED

Method to block the current thread until an unread message is available.

* **Parameters:**
  **[****in****]** – max_wait Max blocking time for this operation.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if there is new unread message, RETCODE_TIMEOUT if timeout

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### wait_for_unread_message(timeout)

Method to block the current thread until an unread message is available.

* **Parameters:**
  **[****in****]** – timeout Max blocking time for this operation.
* **Return type:**
  boolean
* **Returns:**
  true if there is new unread message, false if timeout
