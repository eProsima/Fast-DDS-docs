<a id="rtpsreader"></a>

# RTPSReader

### class RTPSReader : public eprosima::fastdds::rtps::[Endpoint](../Endpoint.md#_CPPv4N8eprosima7fastdds4rtps8EndpointE)

Class [RTPSReader](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader), manages the reception of data from its matched writers. Needs to be constructed using the createRTPSReader method from the [RTPSDomain](../RTPSDomain.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain). 

### Public Functions

### virtual bool matched_writer_add(const [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &info) = 0

Add a matched writer represented by its publication info. 

* **Parameters:**
  **info** – Publication info of the writer being matched.
* **Returns:**
  True if correctly added. 

### virtual bool matched_writer_remove(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, bool removed_by_lease = false) = 0

Remove a writer from the matched writers. 

* **Parameters:**
  * **writer_guid** – GUID of the writer to remove. 
  * **removed_by_lease** – Whether the writer is being unmatched due to a participant drop.
* **Returns:**
  True if correctly removed. 

### virtual bool matched_writer_is_matched(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid) = 0

Check if a specific writer is matched against this reader. 

* **Parameters:**
  **writer_guid** – GUID of the writer to check.
* **Returns:**
  True if the specified writer is matched with this reader. 

### virtual void assert_writer_liveliness(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer) = 0

Assert the liveliness of a matched writer. 

* **Parameters:**
  **writer** – GUID of the writer on which to assert liveliness. 

### virtual bool is_in_clean_state() = 0

Check if this reader is in a clean state with all its matched writers. This will happen when the reader has received all samples announced by all its matched writers. 

* **Returns:**
  Whether the reader is in a clean state with all its matched writers. 

### virtual [ReaderListener](ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) \*get_listener() const = 0

Get the associated listener. 

* **Returns:**
  Pointer to the associated reader listener. 

### virtual void set_listener([ReaderListener](ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) \*listener) = 0

Change the listener associated to this reader. 

* **Parameters:**
  **listener** – The new listener to associate to this reader. 

### virtual bool expects_inline_qos() const = 0

* **Returns:**
  True if the reader expects Inline QoS. 

### virtual [ReaderHistory](../history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE) \*get_history() const = 0

* **Returns:**
  a pointer to the associated [History](../history/History_class.md#classeprosima_1_1fastdds_1_1rtps_1_1History). 

### virtual eprosima::fastdds::rtps::IReaderDataFilter \*get_content_filter() const = 0

* **Returns:**
  The content filter associated to this reader. 

### virtual void set_content_filter(eprosima::fastdds::rtps::IReaderDataFilter \*filter) = 0

Set the content filter associated to this reader.

* **Parameters:**
  **filter** – Pointer to the content filter to associate to this reader. 

### virtual bool matched_writers_guids(std::vector<[GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE)> &guids) const = 0

Fills the provided vector with the GUIDs of the matched writers. 

* **Parameters:**
  **guids** – **[out]** Vector to be filled with the GUIDs of the matched writers. 
* **Returns:**
  True if the operation was successful. 

### virtual [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*next_unread_cache() = 0

Read the next unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) from the history. 

* **Returns:**
  A pointer to the first unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) from the history. 

### virtual [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*next_untaken_cache() = 0

Get the next [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) from the history to take. 

* **Returns:**
  A pointer to the first [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history. 

### virtual bool wait_for_unread_cache(const eprosima::fastdds::dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &timeout) = 0

Wait until there is an unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history.

* **Parameters:**
  **timeout** – Maximum time to wait.
* **Returns:**
  true if there is an unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history. 

### virtual std::uint64_t get_unread_count() const = 0

Get the number of unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history.

* **Returns:**
  The number of unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history. 

### virtual std::uint64_t get_unread_count(bool mark_as_read) = 0

Get the number of unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history and optionally mark them as read.

* **Parameters:**
  **mark_as_read** – Whether to mark the unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) as read.
* **Returns:**
  The number of previously unread [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) in the history. 

### virtual bool is_sample_valid(const void \*data, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer, const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &sn) const = 0

Checks whether the sample is still valid or is corrupted.

* **Parameters:**
  * **data** – Pointer to the sample data to check. If it does not belong to the payload pool passed to the reader on construction, it yields undefined behavior. 
  * **writer** – GUID of the writer that sent `data`. 
  * **sn** – Sequence number related to `data`.
* **Returns:**
  true if the sample is valid 

### virtual bool add_statistics_listener(std::shared_ptr<fastdds::statistics::IListener> listener) = 0

Add a listener to receive statistics backend callbacks 

* **Parameters:**
  **listener** – 
* **Returns:**
  true if successfully added 

### virtual bool remove_statistics_listener(std::shared_ptr<fastdds::statistics::IListener> listener) = 0

Remove a listener from receiving statistics backend callbacks 

* **Parameters:**
  **listener** – 
* **Returns:**
  true if successfully removed 

### virtual void set_enabled_statistics_writers_mask(std::uint32_t enabled_writers) = 0

Set the enabled statistics writers mask. 

* **Parameters:**
  **enabled_writers** – The new mask to set 

### virtual bool get_connections(fastdds::statistics::rtps::ConnectionList &connection_list) = 0

Get the connection list of this reader. 

* **Parameters:**
  **connection_list** – **[out]** of the reader 
* **Returns:**
  True if could be retrieved
