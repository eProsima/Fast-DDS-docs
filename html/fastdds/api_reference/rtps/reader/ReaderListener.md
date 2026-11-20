<a id="readerlistener"></a>

# ReaderListener

### class ReaderListener

Class [ReaderListener](#classeprosima_1_1fastdds_1_1rtps_1_1ReaderListener), to be used by the user to override some of is virtual method to program actions to certain events. 

### Public Functions

### inline virtual void on_reader_matched([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, const [MatchingInfo](../common/MatchingInfo/MatchingInfo.md#_CPPv4N8eprosima7fastdds4rtps12MatchingInfoE) &info)

This method is invoked when a new reader matches 

* **Parameters:**
  * **reader** – Matching reader 
  * **info** – Matching information of the reader 

### inline virtual void on_new_cache_change_added([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, const [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*const change)

This method is called when a new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) is added to the [ReaderHistory](../history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). 

* **Parameters:**
  * **reader** – Pointer to the reader. 
  * **change** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). This is a const pointer to const data to indicate that the user should not dispose of this data himself. To remove the data call the remove_change method of the [ReaderHistory](../history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). reader->get_history()->remove_change((CacheChange_t\*)change). 

### inline virtual void on_liveliness_changed([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, const eprosima::fastdds::dds::[LivelinessChangedStatus](../../dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatusE) &status)

Method called when the liveliness of a reader changes. 

* **Parameters:**
  * **reader** – The reader 
  * **status** – The liveliness changed status 

### inline virtual void on_requested_incompatible_qos([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, eprosima::fastdds::dds::PolicyMask qos)

This method is called when a new Writer is discovered, with a Topic that matches that of a local reader, but with an offered QoS that is incompatible with the one requested by the local reader 

* **Parameters:**
  * **reader** – Pointer to the [RTPSReader](RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 
  * **qos** – A mask with the bits of all incompatible Qos activated. 

### inline virtual void on_sample_lost([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, std::int32_t sample_lost_since_last_update)

This method is called when the reader detects that one or more samples have been lost.

* **Parameters:**
  * **reader** – Pointer to the [RTPSReader](RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 
  * **sample_lost_since_last_update** – The number of samples that were lost since the last time this method was called for the same reader. 

### inline virtual void on_writer_discovery([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, [WriterDiscoveryStatus](../participant/participantdiscoveryinfo/WriterDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21WriterDiscoveryStatusE) reason, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, const [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) \*writer_info)

Method called when the discovery information of a writer regarding a reader changes. 

* **Parameters:**
  * **reader** – The reader. 
  * **reason** – The reason motivating this method to be called. 
  * **writer_guid** – The GUID of the writer for which the discovery information changed. 
  * **writer_info** – Discovery information about the writer. Will be `nullptr` for reason `REMOVED_WRITER`. 

### inline virtual void on_sample_rejected([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, eprosima::fastdds::dds::[SampleRejectedStatusKind](../../dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) reason, const [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*const change)

This method is called when the reader rejects a samples.

* **Parameters:**
  * **reader** – Pointer to the [RTPSReader](RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 
  * **reason** – Indicates reason for sample rejection. 
  * **change** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). This is a const pointer to const data to indicate that the user should not dispose of this data himself. 

### inline virtual void on_data_available([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &first_sequence, const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &last_sequence, bool &should_notify_individual_changes)

This method is called when new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) objects are made available to the user. 

#### NOTE
This method is currently never called. Implementation will be added in future releases.

* **Parameters:**
  * **reader** – **[in]** Pointer to the reader performing the notification. 
  * **writer_guid** – **[in]** GUID of the writer from which the changes were received. 
  * **first_sequence** – **[in]** Sequence number of the first change made available. 
  * **last_sequence** – **[in]** Sequence number of the last change made available. It will always be greater or equal than `first_sequence`. 
  * **should_notify_individual_changes** – **[out]** Whether the individual changes should be notified by means of `on_new_cache_change_added`. 

### inline virtual void on_incompatible_type([RTPSReader](RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader)

This method is called when a new Writer is discovered, with a Topic that matches the name of a local reader, but with an incompatible type

* **Parameters:**
  **reader** – Pointer to the [RTPSReader](RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader).
