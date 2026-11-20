<a id="api-pim-datareader"></a>

<a id="datareader"></a>

# DataReader

### class DataReader : public eprosima::fastdds::dds::[DomainEntity](../core/domainentity.md#_CPPv4N8eprosima7fastdds3dds12DomainEntityE)

Class [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader), contains the actual implementation of the behaviour of the [Subscriber](subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

### Read or take data methods.

Methods to read or take data from the History. 

### ReturnCode_t read([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of Data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The caller can limit the size of the returned collection with the `max_samples` parameter.

The properties of the `data_values` collection and the setting of the [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy) may impose further limits on the size of the returned ‘list.’

1. If [PresentationQosPolicy::access_scope](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1aaa5c12ffd6a13f00974026b8ce7245fa) is [INSTANCE_PRESENTATION_QOS](../core/policy/presentationqospolicyaccessscopekind.md#namespaceeprosima_1_1fastdds_1_1dds_1a6cdeae41e8d1248021485f2b5a1f9514a49cd391d79e48c306ed65bd5a3123791), then the returned collection is a ‘list’ where samples belonging to the same data-instance are consecutive.
2. If [PresentationQosPolicy::access_scope](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1aaa5c12ffd6a13f00974026b8ce7245fa) is [TOPIC_PRESENTATION_QOS](../core/policy/presentationqospolicyaccessscopekind.md#namespaceeprosima_1_1fastdds_1_1dds_1a6cdeae41e8d1248021485f2b5a1f9514aac7f7a1fc215081384b3fa1124515773) and [PresentationQosPolicy::ordered_access](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1a1cb27ff29221aa42595c94d84f22faa9) is set to `false`, then the returned collection is a ‘list’ where samples belonging to the same data-instance are consecutive.
3. If [PresentationQosPolicy::access_scope](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1aaa5c12ffd6a13f00974026b8ce7245fa) is [TOPIC_PRESENTATION_QOS](../core/policy/presentationqospolicyaccessscopekind.md#namespaceeprosima_1_1fastdds_1_1dds_1a6cdeae41e8d1248021485f2b5a1f9514aac7f7a1fc215081384b3fa1124515773) and [PresentationQosPolicy::ordered_access](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1a1cb27ff29221aa42595c94d84f22faa9) is set to `true`, then the returned collection is a ‘list’ where samples belonging to the same instance may or may not be consecutive. This is because to preserve order it may be necessary to mix samples from different instances.
4. If [PresentationQosPolicy::access_scope](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1aaa5c12ffd6a13f00974026b8ce7245fa) is [GROUP_PRESENTATION_QOS](../core/policy/presentationqospolicyaccessscopekind.md#namespaceeprosima_1_1fastdds_1_1dds_1a6cdeae41e8d1248021485f2b5a1f9514ad114a7378c3ac0e34b3843e4998e19a2) and [PresentationQosPolicy::ordered_access](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1a1cb27ff29221aa42595c94d84f22faa9) is set to `false`, then the returned collection is a ‘list’ where samples belonging to the same data instance are consecutive.
5. If [PresentationQosPolicy::access_scope](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1aaa5c12ffd6a13f00974026b8ce7245fa) is [GROUP_PRESENTATION_QOS](../core/policy/presentationqospolicyaccessscopekind.md#namespaceeprosima_1_1fastdds_1_1dds_1a6cdeae41e8d1248021485f2b5a1f9514ad114a7378c3ac0e34b3843e4998e19a2) and [PresentationQosPolicy::ordered_access](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy_1a1cb27ff29221aa42595c94d84f22faa9) is set to `true`, then the returned collection contains at most one sample. The difference in this case is due to the fact that it is required that the application is able to read samples belonging to different [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects in a specific order.

In any case, the relative order between the samples of one instance is consistent with the [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy):

* If [DestinationOrderQosPolicy::kind](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy_1a65d65a6da070efcf982a7fda2ad35aa3) is [BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS](../core/policy/destinationorderqospolicykind.md#namespaceeprosima_1_1fastdds_1_1dds_1aaf12f18308c158131e509c0dc575e73aa7cf06d167bd16ef642bcfab65a3d0ff0), samples belonging to the same instances will appear in the relative order in which there were received (FIFO, earlier samples ahead of the later samples).
* If [DestinationOrderQosPolicy::kind](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy_1a65d65a6da070efcf982a7fda2ad35aa3) is [BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS](../core/policy/destinationorderqospolicykind.md#namespaceeprosima_1_1fastdds_1_1dds_1aaf12f18308c158131e509c0dc575e73aa3744d552d8bbbb3e59afc9aacfb96863), samples belonging to the same instances will appear in the relative order implied by the source_timestamp (FIFO, smaller values of source_timestamp ahead of the larger values).

The actual number of samples returned depends on the information that has been received by the middleware as well as the [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy), [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy), and [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos):

* In the case where the [HistoryQosPolicy::kind](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy_1a1034c314902f1c40860e57a49ec01948) is KEEP_LAST_HISTORY_QOS, the call will return at most [HistoryQosPolicy::depth](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy_1adacdd3b29e172e159f217f235ecd66df) samples per instance.
* The maximum number of samples returned is limited by [ResourceLimitsQosPolicy::max_samples](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy_1ac87830cf99f3d1fdc06cfd903d60bacd), and by [ReaderResourceLimitsQos::max_samples_per_read](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos_1a1a3d7508790dc9398ac13e500e13826f).
* For multiple instances, the number of samples returned is additionally limited by the product ([ResourceLimitsQosPolicy::max_samples_per_instance](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy_1a80d1d5b65a25fffd2cd46368351f79dc) \* [ResourceLimitsQosPolicy::max_instances](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy_1af9dabd81e358b2969fc70faba4f553e2)).
* If [ReaderResourceLimitsQos::sample_infos_allocation](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos_1a759ec2b613d13a1fe2799036121d200a) has a maximum limit, the number of samples returned may also be limited if insufficient [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) resources are available.

If the operation succeeds and the number of samples returned has been limited (by means of a maximum limit, as listed above, or insufficient [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) resources), the call will complete successfully and provide those samples the reader is able to return. The user may need to make additional calls, or return outstanding loaned buffers in the case of insufficient resources, in order to access remaining samples.

In addition to the collection of samples, the read operation also uses a collection of [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) structures (`sample_infos`).

The initial (input) properties of the `data_values` and `sample_infos` collections will determine the precise behavior of this operation. For the purposes of this description the collections are modeled as having three properties:

* the current length (`len`, see [LoanableCollection::length()](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac))
* the maximum length (`max_len`, see [LoanableCollection::maximum()](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97))
* whether the collection container owns the memory of the elements within (`owns`, see [LoanableCollection::has_ownership()](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a))

The initial (input) values of the `len`, `max_len`, and `owns` properties for the `data_values` and `sample_infos` collections govern the behavior of the read operation as specified by the following rules:

1. The values of `len`, `max_len`, and `owns` for the two collections must be identical. Otherwise read will fail with RETCODE_PRECONDITION_NOT_MET.
2. On successful output, the values of `len`, `max_len`, and `owns` will be the same for both collections.
3. If the input ` max_len == 0 `, then the `data_values` and `sample_infos` collections will be filled with elements that are ‘loaned’ by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). On output, `owns` will be `false`, `len` will be set to the number of values returned, and `max_len` will be set to a value verifying ` max_len >= len `. The use of this variant allows for zero-copy access to the data and the application will need to return the loan to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) using the [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b) operation.
4. If the input ` max_len > 0 ` and the input ` owns == false `, then the read operation will fail with RETCODE_PRECONDITION_NOT_MET. This avoids the potential hard-to-detect memory leaks caused by an application forgetting to return the loan.
5. If input ` max_len > 0 ` and the input ` owns == true `, then the read operation will copy the Data values and [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) values into the elements already inside the collections. On output, `owns` will be `true`, `len` will be set to the number of values copied, and `max_len` will remain unchanged. The use of this variant forces a copy but the application can control where the copy is placed and the application will not need to return the loan. The number of samples copied depends on the values of `max_len` and `max_samples:` 
   * If ` max_samples == LENGTH_UNLIMITED `, then at most `max_len` values will be copied. The use of this variant lets the application limit the number of samples returned to what the sequence can accommodate.
   * If ` max_samples <= max_len `, then at most `max_samples` values will be copied. The use of this variant lets the application limit the number of samples returned to fewer that what the sequence can accommodate.
   * If ` max_samples > max_len `, then the read operation will fail with RETCODE_PRECONDITION_NOT_MET. This avoids the potential confusion where the application expects to be able to access up to `max_samples`, but that number can never be returned, even if they are available in the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader), because the output sequence cannot accommodate them.

As described above, upon return the `data_values` and `sample_infos` collections may contain elements ‘loaned’ from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). If this is the case, the application will need to use the [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b) operation to return the loan once it is no longer using the Data in the collection. Upon return from [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b), the collection will have ` max_len == 0 ` and ` owns == false `.

The application can determine whether it is necessary to return the loan or not based on the state of the collections when the read operation was called, or by accessing the `owns` property. However, in many cases it may be simpler to always call [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b), as this operation is harmless (i.e., leaves all elements unchanged) if the collection does not have a loan.

On output, the collection of Data values and the collection of [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) structures are of the same length and are in a one-to-one correspondence. Each [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) provides information, such as the `source_timestamp`, the `sample_state`, `view_state`, and `instance_state`, etc., about the corresponding sample.

Some elements in the returned collection may not have valid data. If the `instance_state` in the [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) is [NOT_ALIVE_DISPOSED_INSTANCE_STATE](instancestatekind.md#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a01b934811bc0b615ce3b0f09a4b73aa0) or [NOT_ALIVE_NO_WRITERS_INSTANCE_STATE](instancestatekind.md#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10af02d8319e0b7413acd28156351e25489), then the last sample for that instance in the collection, that is, the one whose [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) has ` sample_rank == 0 ` does not contain valid data. Samples that contain no data do not count towards the limits imposed by the [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy).

The act of reading a sample changes its `sample_state` to [READ_SAMPLE_STATE](samplestatekind.md#namespaceeprosima_1_1fastdds_1_1dds_1aa0466276d249ece30c7f02578b9aabc7abde4814383bf887b908abfdbca841009). If the sample belongs to the most recent generation of the instance, it will also set the `view_state` of the instance to be [NOT_NEW_VIEW_STATE](viewstatekind.md#namespaceeprosima_1_1fastdds_1_1dds_1afc3f4ffc1198e7a1e069abbd19af53c6a76ae91044fa7f3302938d41478b658d3). It will not affect the `instance_state` of the instance.

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

*Important:* If the samples “returned” by this method are loaned from the middleware (see [take](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) for more information on memory loaning), it is important that their contents not be changed. Because the memory in which the data is stored belongs to the middleware, any modifications made to the data will be seen the next time the same samples are read or taken; the samples will no longer reflect the state that was received from the network.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described above. 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t read_w_condition([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples, [ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*a_condition)

This operation accesses via ‘read’ the samples that match the criteria specified in the [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition). This operation is especially useful in combination with QueryCondition to filter data samples based on the content.

The specified [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) must be attached to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader); otherwise the operation will fail and return RETCODE_PRECONDITION_NOT_MET.

In case the [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) is a ‘plain’ [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) and not the specialized QueryCondition, the operation is equivalent to calling read and passing as `sample_states`, `view_states` and `instance_states` the value of the corresponding attributes in `a_condition`. Using this operation the application can avoid repeating the same parameters specified when creating the [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition).

The samples are accessed with the same semantics as the read operation. If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. 
  * **a_condition** – **[in]** A [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) that returned `data_values` must pass
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t read_instance([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, const InstanceHandle_t &a_handle = HANDLE_NIL, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The behavior is identical to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), except that all samples returned belong to the single specified instance whose handle is `a_handle`.

Upon successful completion, the data collection will contain samples all belonging to the same instance. The corresponding [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) verifies [SampleInfo::instance_handle](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1aa2677420acd442e077410d877b47dad7) == `a_handle`.

This operation is semantically equivalent to the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation, except in building the collection. The [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will check that the sample belongs to the specified instance and otherwise it will not place the sample in the returned collection.

The behavior of this operation follows the same rules as the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos`. Similar to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), this operation may ‘loan’ elements to the output collections, which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **a_handle** – **[in]** The specified instance to return samples for. The method will fail with RETCODE_BAD_PARAMETER if the handle does not correspond to an existing data-object known to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t read_next_instance([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, const InstanceHandle_t &previous_handle = HANDLE_NIL, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) where all the samples belong to a single instance. The behavior is similar to [read_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a0eb70402007eb89497628077920d9f6f), except that the actual instance is not directly specified. Rather, the samples will all belong to the ‘next’ instance with `instance_handle` ‘greater’ than the specified ‘previous_handle’ that has available samples.

This operation implies the existence of a total order ‘greater-than’ relationship between the instance handles. The specifics of this relationship are not all important and are implementation specific. The important thing is that, according to the middleware, all instances are ordered relative to each other. This ordering is between the instance handles, and should not depend on the state of the instance (e.g. whether it has data or not) and must be defined even for instance handles that do not correspond to instances currently managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). For the purposes of the ordering, it should be ‘as if’ each instance handle was represented as an integer.

The behavior of this operation is ‘as if’ the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) invoked [read_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a0eb70402007eb89497628077920d9f6f), passing the smallest `instance_handle` among all the ones that: (a) are greater than `previous_handle`, and (b) have available samples (i.e. samples that meet the constraints imposed by the specified states).

The special value HANDLE_NIL is guaranteed to be ‘less than’ any valid `instance_handle`. So the use of the parameter value `previous_handle` == HANDLE_NIL will return the samples for the instance which has the smallest `instance_handle` among all the instances that contain available samples.

This operation is intended to be used in an application-driven iteration, where the application starts by passing `previous_handle` == HANDLE_NIL, examines the samples returned, and then uses the `instance_handle` returned in the [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) as the value of the `previous_handle` argument to the next call to [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699). The iteration continues until [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) fails with RETCODE_NO_DATA.

Note that it is possible to call the [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) operation with a `previous_handle` that does not correspond to an instance currently managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). This is because as stated earlier the ‘greater-than’ relationship is defined even for handles not managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). One practical situation where this may occur is when an application is iterating through all the instances, takes all the samples of a [NOT_ALIVE_NO_WRITERS_INSTANCE_STATE](instancestatekind.md#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10af02d8319e0b7413acd28156351e25489) instance, returns the loan (at which point the instance information may be removed, and thus the handle becomes invalid), and tries to read the next instance.

The behavior of this operation follows the same rules as the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos`. Similar to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), this operation may ‘loan’ elements to the output collections, which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **previous_handle** – **[in]** The ‘next smallest’ instance with a value greater than this value that has available samples will be returned. 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t read_next_instance_w_condition([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples, const InstanceHandle_t &previous_handle, [ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*a_condition)

This operation accesses a collection of Data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The behavior is identical to [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) except that all samples returned satisfy the specified condition. In other words, on success all returned samples belong to the same instance, and the instance is the instance with ‘smallest’ `instance_handle` among the ones that verify (a) `instance_handle` >= `previous_handle` and (b) have samples for which the specified [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) evaluates to TRUE.

Similar to the operation [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) it is possible to call [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) with a `previous_handle` that does not correspond to an instance currently managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

The behavior of the [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) operation follows the same rules than the read operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar to read, the [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) operation may ‘loan’ elements to the output collections which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **previous_handle** – **[in]** The ‘next smallest’ instance with a value greater than this value that has available samples will be returned. 
  * **a_condition** – **[in]** A [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) that returned `data_values` must pass
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t read_next_sample(void \*data, [SampleInfo](sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) \*info)

This operation copies the next, non-previously accessed Data value from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader); the operation also copies the corresponding [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo). The implied order among the samples stored in the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) is the same as for the read operation. 

The read_next_sample operation is semantically equivalent to the read operation where the input Data sequence has ` max_length = 1 `, the ` sample_states = NOT_READ_SAMPLE_STATE `, the ` view_states = ANY_VIEW_STATE `, and the ` instance_states = ANY_INSTANCE_STATE `.

The read_next_sample operation provides a simplified API to ‘read’ samples avoiding the need for the application to manage sequences and specify states.

If there is no unread data in the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader), the operation will return RETCODE_NO_DATA and nothing is copied

* **Parameters:**
  * **data** – **[out]** Data pointer to store the sample 
  * **info** – **[out]** [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) pointer to store the sample information
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of data-samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and a corresponding collection of [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) structures, and ‘removes’ them from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The operation will return either a ‘list’ of samples or else a single sample. This is controlled by the [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy) using the same logic as for the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation.

The act of taking a sample removes it from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) so it cannot be ‘read’ or ‘taken’ again. If the sample belongs to the most recent generation of the instance, it will also set the `view_state` of the instance to NOT_NEW. It will not affect the `instance_state` of the instance.

The behavior of the take operation follows the same rules than the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), the take operation may ‘loan’ elements to the output collections which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b). The only difference with [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) is that, as stated, the samples returned by take will no longer be accessible to successive calls to read or take.

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take_w_condition([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples, [ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*a_condition)

This operation is analogous to [read_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1ac6642bc81cf9384c056c5fb303494c1e) except it accesses samples via the ‘take’ operation.

The specified [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) must be attached to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader); otherwise the operation will fail and return RETCODE_PRECONDITION_NOT_MET.

The samples are accessed with the same semantics as the [take](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) operation.

This operation is especially useful in combination with QueryCondition to filter data samples based on the content.

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the return value will be RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are. 
  * **a_condition** – **[in]** A [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) that returned `data_values` must pass
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take_instance([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, const InstanceHandle_t &a_handle = HANDLE_NIL, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and ‘removes’ them from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation has the same behavior as [read_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a0eb70402007eb89497628077920d9f6f), except that the samples are ‘taken’ from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) such that they are no longer accessible via subsequent ‘read’ or ‘take’ operations.

The behavior of this operation follows the same rules as the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos`. Similar to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), this operation may ‘loan’ elements to the output collections, which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **a_handle** – **[in]** The specified instance to return samples for. The method will fail with RETCODE_BAD_PARAMETER if the handle does not correspond to an existing data-object known to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take_next_instance([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples = LENGTH_UNLIMITED, const InstanceHandle_t &previous_handle = HANDLE_NIL, SampleStateMask sample_states = ANY_SAMPLE_STATE, ViewStateMask view_states = ANY_VIEW_STATE, InstanceStateMask instance_states = ANY_INSTANCE_STATE)

Access a collection of data samples from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation accesses a collection of data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and ‘removes’ them from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation has the same behavior as [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699), except that the samples are ‘taken’ from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) such that they are no longer accessible via subsequent ‘read’ or ‘take’ operations.

Similar to the operation [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699), it is possible to call this operation with a `previous_handle` that does not correspond to an instance currently managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

The behavior of this operation follows the same rules as the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos`. Similar to [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2), this operation may ‘loan’ elements to the output collections, which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the operations fails with RETCODE_NO_DATA.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **previous_handle** – **[in]** The ‘next smallest’ instance with a value greater than this value that has available samples will be returned. 
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will be returned. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will be returned. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will be returned.
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take_next_instance_w_condition([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos, std::int32_t max_samples, const InstanceHandle_t &previous_handle, [ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*a_condition)

This operation accesses a collection of Data values from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). The behavior is identical to [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) except that all samples returned satisfy the specified condition. In other words, on success all returned samples belong to the same instance, and the instance is the instance with ‘smallest’ `instance_handle` among the ones that verify (a) `instance_handle` >= `previous_handle` and (b) have samples for which the specified [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) evaluates to TRUE.

Similar to the operation [read_next_instance](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1adc733f22e0767ea1e5e82eea1231f699) it is possible to call [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) with a `previous_handle` that does not correspond to an instance currently managed by the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

The behavior of the [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) operation follows the same rules than the read operation regarding the pre-conditions and post-conditions for the `data_values` and `sample_infos` collections. Similar to read, the [read_next_instance_w_condition](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a5c6a831231108bf8169f8295b9399879) operation may ‘loan’ elements to the output collections which must then be returned by means of [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b).

If the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has no samples that meet the constraints, the return value will be RETCODE_NO_DATA

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples will be returned. 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample info will be returned. 
  * **max_samples** – **[in]** The maximum number of samples to be returned. If the special value LENGTH_UNLIMITED is provided, as many samples will be returned as are available, up to the limits described in the documentation for [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2). 
  * **previous_handle** – **[in]** The ‘next smallest’ instance with a value greater than this value that has available samples will be returned. 
  * **a_condition** – **[in]** A [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) that returned `data_values` must pass
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t take_next_sample(void \*data, [SampleInfo](sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) \*info)

This operation copies the next, non-previously accessed Data value from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and ‘removes’ it from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) so it is no longer accessible. The operation also copies the corresponding [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo). 

This operation is analogous to [read_next_sample](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1ae58336d79cad217f87a0028c15228bb0) except for the fact that the sample is ‘removed’ from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

This operation is semantically equivalent to the [take](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) operation where the input sequence has ` max_length = 1 `, the ` sample_states = NOT_READ_SAMPLE_STATE `, the ` view_states = ANY_VIEW_STATE `, and the ` instance_states = ANY_INSTANCE_STATE `.

This operation provides a simplified API to ’take’ samples avoiding the need for the application to manage sequences and specify states.

If there is no unread data in the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader), the operation will return RETCODE_NO_DATA and nothing is copied.

* **Parameters:**
  * **data** – **[out]** Data pointer to store the sample 
  * **info** – **[out]** [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) pointer to store the sample information
* **Returns:**
  Any of the standard return codes. 

### Public Functions

### virtual ~DataReader()

Destructor. 

### virtual ReturnCode_t enable() override

This operation enables the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the [Subscriber](subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) creating this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) is not enabled. 

### bool wait_for_unread_message(const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &timeout)

Method to block the current thread until an unread message is available.

* **Parameters:**
  **timeout** – **[in]** Max blocking time for this operation.
* **Returns:**
  true if there is new unread message, false if timeout 

### ReturnCode_t wait_for_historical_data(const fastdds::dds::[Duration_t](../core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &max_wait) const

Method to block the current thread until an unread message is available. 

NOT YET IMPLEMENTED

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **max_wait** – **[in]** Max blocking time for this operation. 
* **Returns:**
  RETCODE_OK if there is new unread message, RETCODE_TIMEOUT if timeout

### ReturnCode_t return_loan([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data_values, SampleInfoSeq &sample_infos)

This operation indicates to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that the application is done accessing the collection of `data_values` and `sample_infos` obtained by some earlier invocation of [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) or [take](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) on the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).

The `data_values` and `sample_infos` must belong to a single related ‘pair’; that is, they should correspond to a pair returned from a single call to read or take. The `data_values` and `sample_infos` must also have been obtained from the same [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to which they are returned. If either of these conditions is not met, the operation will fail and return RETCODE_PRECONDITION_NOT_MET.

This operation allows implementations of the [read](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) and [take](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) operations to “loan” buffers from the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to the application and in this manner provide “zero-copy” access to the data. During the loan, the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will guarantee that the data and sample-information are not modified.

It is not necessary for an application to return the loans immediately after the read or take calls. However, as these buffers correspond to internal resources inside the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader), the application should not retain them indefinitely.

The use of the [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b) operation is only necessary if the read or take calls “loaned” buffers to the application. This only occurs if the `data_values` and `sample_infos` collections had ` max_len == 0 ` at the time read or take was called. The application may also examine the `has_ownership` property of the collection to determine if there is an outstanding loan. However, calling [return_loan](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a084b9a068abf972e27bfd201b6c6f15b) on a collection that does not have a loan is safe, has no side effects, and returns RETCODE_OK.

If the collections had a loan, upon return from return_loan the collections will have ` max_len == 0 `.

* **Parameters:**
  * **data_values** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples were obtained from an earlier invocation of read or take on this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 
  * **sample_infos** – **[inout]** A SampleInfoSeq object where the received sample infos were obtained from an earlier invocation of read or take on this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader).
* **Returns:**
  Any of the standard return codes. 

### ReturnCode_t get_key_value(void \*key_holder, const InstanceHandle_t &handle)

NOT YET IMPLEMENTED

This operation can be used to retrieve the instance key that corresponds to an `instance_handle`. The operation will only fill the fields that form the key inside the key_holder instance.

This operation may return BAD_PARAMETER if the InstanceHandle_t a_handle does not correspond to an existing data-object known to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). If the implementation is not able to check invalid handles then the result in this situation is unspecified.

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **key_holder** – **[inout]** 
  * **handle** – 
* **Returns:**
  Any of the standard return codes.

### InstanceHandle_t lookup_instance(const void \*instance) const

Takes as a parameter an instance and returns a handle that can be used in subsequent operations that accept an instance handle as an argument. The instance parameter is only used for the purpose of examining the fields that define the key.

* **Parameters:**
  **instance** – **[in]** Data pointer to the sample
* **Returns:**
  handle of the given `instance`. 
* **Returns:**
  HANDLE_NIL if `instance` is nullptr. 
* **Returns:**
  HANDLE_NIL if there is no instance on the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader)’s history with the same key as `instance`. 

### ReturnCode_t get_first_untaken_info([SampleInfo](sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) \*info)

Returns information about the first untaken sample. This method is meant to be called prior to a [read()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1a352a0356b0ee22db067f801ddafba9e2) or [take()](#classeprosima_1_1fastdds_1_1dds_1_1DataReader_1acc1a19943ed3c25c125421f82647d059) operation as it does not modify the status condition of the entity. 

* **Parameters:**
  **info** – **[out]** Pointer to a SampleInfo_t structure to store first untaken sample information.
* **Returns:**
  RETCODE_OK if sample info was returned. RETCODE_NO_DATA if there is no sample to take. 

### std::uint64_t get_unread_count() const

Get the number of samples pending to be read. The number includes samples that may not yet be available to be read or taken by the user, due to samples being received out of order.

* **Returns:**
  the number of samples on the reader history that have never been read. 

### std::uint64_t get_unread_count(bool mark_as_read) const

Get the number of samples pending to be read.

* **Parameters:**
  **mark_as_read** – Whether the unread samples should be marked as read or not.
* **Returns:**
  the number of samples on the reader history that have never been read. 

### const fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid()

Get associated GUID.

* **Returns:**
  Associated GUID 

### const fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid() const

Get associated GUID.

* **Returns:**
  Associated GUID 

### InstanceHandle_t get_instance_handle() const

Getter for the associated InstanceHandle. 

* **Returns:**
  Copy of the InstanceHandle 

### [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) type() const

Getter for the data type.

* **Returns:**
  Copy of the [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) associated to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

### const [TopicDescription](../topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) \*get_topicdescription() const

Get [TopicDescription](../topic/topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription).

* **Returns:**
  [TopicDescription](../topic/topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription) pointer. 

### ReturnCode_t get_requested_deadline_missed_status([RequestedDeadlineMissedStatus](../core/status/requesteddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds29RequestedDeadlineMissedStatusE) &status)

Get the requested deadline missed status. 

* **Returns:**
  The deadline missed status. 

### ReturnCode_t get_requested_incompatible_qos_status([RequestedIncompatibleQosStatus](../core/status/requestedicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds30RequestedIncompatibleQosStatusE) &status)

Get the requested incompatible qos status. 

* **Parameters:**
  **status** – **[out]** Requested incompatible qos status.
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos)

Setter for the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

* **Parameters:**
  **qos** – **[in]** new value for the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos).
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### const [DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &get_qos() const

Getter for the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

* **Returns:**
  Pointer to the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

### ReturnCode_t get_qos([DataReaderQos](datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) &qos) const

Getter for the [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos). 

* **Parameters:**
  **qos** – **[in]** [DataReaderQos](datareaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos) where the qos is returned.
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_listener([DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) \*listener)

Modifies the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener), sets the mask to [StatusMask::all()](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask_1aa89f272bfb912d5a5bf940c676a1e4c8).

* **Parameters:**
  **listener** – **[in]** new value for the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener).
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_listener([DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Modifies the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener).

* **Parameters:**
  * **listener** – **[in]** new value for the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener). 
  * **mask** – **[in]** [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) that holds statuses the listener responds to (default: all).
* **Returns:**
  RETCODE_OK 

### const [DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) \*get_listener() const

Getter for the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener). 

* **Returns:**
  Pointer to the [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener)

### ReturnCode_t get_liveliness_changed_status([LivelinessChangedStatus](../core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatusE) &status) const

Get the liveliness changed status. 

* **Parameters:**
  **status** – **[out]** [LivelinessChangedStatus](../core/status/livelinesschangedstatus.md#structeprosima_1_1fastdds_1_1dds_1_1LivelinessChangedStatus) object where the status is returned.
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_sample_lost_status([SampleLostStatus](../core/status/sampleloststatus.md#_CPPv4N8eprosima7fastdds3dds16SampleLostStatusE) &status) const

Get the SAMPLE_LOST communication status. 

* **Parameters:**
  **status** – **[out]** SampleLostStatus object where the status is returned.
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_sample_rejected_status([SampleRejectedStatus](../core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatusE) &status) const

Get the SAMPLE_REJECTED communication status. 

* **Parameters:**
  **status** – **[out]** [SampleRejectedStatus](../core/status/samplerejectedstatus.md#structeprosima_1_1fastdds_1_1dds_1_1SampleRejectedStatus) object where the status is returned.
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_subscription_matched_status([SubscriptionMatchedStatus](../core/status/subscriptionmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds25SubscriptionMatchedStatusE) &status) const

Returns the subscription matched status. 

* **Parameters:**
  **status** – **[out]** subscription matched status struct 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t get_matched_publication_data(PublicationBuiltinTopicData &publication_data, const fastdds::rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &publication_handle) const

Retrieves in a publication associated with the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  * **publication_data** – **[out]** publication data struct 
  * **publication_handle** – InstanceHandle_t of the publication 
* **Returns:**
  RETCODE_OK

### ReturnCode_t get_matched_publications(std::vector<InstanceHandle_t> &publication_handles) const

Fills the given vector with the InstanceHandle_t of matched DataReaders. 

#### WARNING
Not supported yet. Currently returns RETCODE_UNSUPPORTED 

* **Parameters:**
  **publication_handles** – **[out]** Vector where the InstanceHandle_t are returned 
* **Returns:**
  RETCODE_OK

### [ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*create_readcondition(SampleStateMask sample_states, ViewStateMask view_states, InstanceStateMask instance_states)

This operation creates a [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition). The returned [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) will be attached and belong to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

* **Parameters:**
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will trigger the created condition. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will trigger the created condition. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will trigger the created condition.
* **Returns:**
  pointer to the created [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition), nullptr in case of error. 

### QueryCondition \*create_querycondition(SampleStateMask sample_states, ViewStateMask view_states, InstanceStateMask instance_states, const std::string &query_expression, const std::vector<std::string> &query_parameters)

This operation creates a QueryCondition. The returned QueryCondition will be attached and belong to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

* **Parameters:**
  * **sample_states** – **[in]** Only data samples with `sample_state` matching one of these will trigger the created condition. 
  * **view_states** – **[in]** Only data samples with `view_state` matching one of these will trigger the created condition. 
  * **instance_states** – **[in]** Only data samples with `instance_state` matching one of these will trigger the created condition. 
  * **query_expression** – **[in]** Only data samples matching this query will trigger the created condition. 
  * **query_parameters** – **[in]** Value of the parameters on the query expression.
* **Returns:**
  pointer to the created QueryCondition, nullptr in case of error. 

### ReturnCode_t delete_readcondition([ReadCondition](readcondition.md#_CPPv4N8eprosima7fastdds3dds13ReadConditionE) \*a_condition)

This operation deletes a [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) attached to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

* **Parameters:**
  **a_condition** – pointer to a [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) belonging to the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
* **Returns:**
  RETCODE_OK 

### const [Subscriber](subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*get_subscriber() const

Getter for the [Subscriber](subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). 

* **Returns:**
  [Subscriber](subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) pointer 

### ReturnCode_t delete_contained_entities()

This operation deletes all the entities that were created by means of the “create” operations on the [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). That is, it deletes all contained [ReadCondition](readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) and QueryCondition objects.

The operation will return PRECONDITION_NOT_MET if the any of the contained entities is in a state where it cannot be deleted.

* **Returns:**
  Any of the standard return codes. 

### bool is_sample_valid(const void \*data, const [SampleInfo](sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) \*info) const

Checks whether a loaned sample is still valid or is corrupted. Calling this method on a sample which has not been loaned, or one for which the loan has been returned yields undefined behavior.

* **Parameters:**
  * **data** – Pointer to the sample data to check 
  * **info** – Pointer to the [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) related to `data`
* **Returns:**
  true if the sample is valid 

### ReturnCode_t get_listening_locators(rtps::[LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locators) const

Get the list of locators on which this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) is listening.

* **Parameters:**
  **locators** – **[out]** LocatorList where the list of locators will be stored.
* **Returns:**
  NOT_ENABLED if the reader has not been enabled. 
* **Returns:**
  OK if a list of locators is returned. 

### ReturnCode_t get_subscription_builtin_topic_data(SubscriptionBuiltinTopicData &subscription_data) const

Retrieve the subscription data discovery information.

* **Parameters:**
  **subscription_data** – **[out]** The subscription data discovery information.
* **Returns:**
  NOT_ENABLED if the reader has not been enabled. 
* **Returns:**
  OK if the subscription data is returned. 

### ReturnCode_t set_related_datawriter(const [DataWriter](../publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*related_writer)

This operation sets the key of the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that is related to this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). This is used to establish a relationship between a [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and a [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) in the context of RPC over DDS.

#### WARNING
This operation is only valid if the entity is not enabled.

* **Parameters:**
  **related_writer** – **[in]** Pointer to the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) to set as related.
* **Returns:**
  RETCODE_OK if the key is set successfully. 
* **Returns:**
  RETCODE_ILLEGAL_OPERATION if this entity is enabled. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the entity does not belong to the same participant. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the provided GUID is unknown or the pointer is not valid. 

### ReturnCode_t set_type_support_context(const std::shared_ptr<[TopicDataType](../topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)::[Context](../topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context)

Set the type support context to be used when deserializing data for this [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). Cannot be called on an enabled [DataReader](#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

* **Parameters:**
  **context** – **[in]** Shared pointer to the context to be used when serializing data.
* **Returns:**
  RETCODE_OK if the context is set successfully. RETCODE_ILLEGAL_OPERATION if this entity is enabled.
