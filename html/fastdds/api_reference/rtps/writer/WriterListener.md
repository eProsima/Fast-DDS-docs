<a id="writerlistener"></a>

# WriterListener

### class WriterListener

Class [WriterListener](#classeprosima_1_1fastdds_1_1rtps_1_1WriterListener) with virtual method so the user can implement callbacks to certain events. 

### Public Functions

### inline virtual void on_writer_matched([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer, const [MatchingInfo](../common/MatchingInfo/MatchingInfo.md#_CPPv4N8eprosima7fastdds4rtps12MatchingInfoE) &info)

This method is called when a new Reader is matched with this Writer by the builtin protocols 

* **Parameters:**
  * **writer** – Pointer to the [RTPSWriter](RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 
  * **info** – Matching Information. 

### inline virtual void on_offered_incompatible_qos([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer, eprosima::fastdds::dds::PolicyMask qos)

This method is called when a new Reader is discovered, with a Topic that matches that of a local writer, but with a requested QoS that is incompatible with the one offered by the local writer 

* **Parameters:**
  * **writer** – Pointer to the [RTPSWriter](RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 
  * **qos** – A mask with the bits of all incompatible Qos activated. 

### inline virtual void on_writer_change_received_by_all([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer, [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*change)

This method is called when all the readers matched with this Writer acknowledge that a cache change has been received. 

* **Parameters:**
  * **writer** – Pointer to the [RTPSWriter](RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 
  * **change** – Pointer to the affected [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 

### inline virtual void on_liveliness_lost([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer, const LivelinessLostStatus &status)

Method called when the liveliness of a writer is lost. 

* **Parameters:**
  * **writer** – The writer 
  * **status** – The liveliness lost status 

### inline virtual void on_reader_discovery([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer, [ReaderDiscoveryStatus](../participant/participantdiscoveryinfo/ReaderDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21ReaderDiscoveryStatusE) reason, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &reader_guid, const [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) \*reader_info)

Method called when the discovery information of a reader regarding a writer changes. 

* **Parameters:**
  * **writer** – The writer. 
  * **reason** – The reason motivating this method to be called. 
  * **reader_guid** – The GUID of the reader for which the discovery information changed. 
  * **reader_info** – Discovery information about the reader. Will be `nullptr` for reason `REMOVED_READER`. 

### inline virtual void on_incompatible_type([RTPSWriter](RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer)

This method is called when a new Reader is discovered, with a Topic that matches the name of a local writer, but with an incompatible type

* **Parameters:**
  **writer** – Pointer to the [RTPSWriter](RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter).
