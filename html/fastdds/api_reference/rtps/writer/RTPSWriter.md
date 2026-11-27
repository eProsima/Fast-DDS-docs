<a id="rtpswriter"></a>

# RTPSWriter

### class RTPSWriter : public eprosima::fastdds::rtps::[Endpoint](../Endpoint.md#_CPPv4N8eprosima7fastdds4rtps8EndpointE)

Class [RTPSWriter](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter), manages the sending of data to the readers. Is always associated with a HistoryCache. 

### Public Functions

### virtual bool matched_reader_add(const [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &info) = 0

Add a matched reader represented by its attributes. 

* **Parameters:**
  **info** – Subscription info of the reader being matched.
* **Returns:**
  True if added. 

### virtual bool matched_reader_remove(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &reader_guid) = 0

Remove a matched reader. 

* **Parameters:**
  **reader_guid** – GUID of the reader to remove. 
* **Returns:**
  True if removed. 

### virtual bool matched_reader_is_matched(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &reader_guid) = 0

Tells us if a specific Reader is matched against this writer. 

* **Parameters:**
  **reader_guid** – GUID of the reader to check. 
* **Returns:**
  True if it was matched. 

### virtual void reader_data_filter(IReaderDataFilter \*filter) = 0

Set a content filter to perform content filtering on this writer. 

This method sets a content filter that will be used to check whether a cache change is relevant for a reader or not.

* **Parameters:**
  **filter** – The content filter to use on this writer. May be `nullptr` to remove the content filter (i.e. treat all samples as relevant). 

### virtual const IReaderDataFilter \*reader_data_filter() const = 0

Get the content filter used to perform content filtering on this writer. 

* **Returns:**
  The content filter used on this writer. 

### virtual bool has_been_fully_delivered(const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &seq_num) const = 0

Check if a specific change has been delivered to the transport layer of every matched remote [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader) at least once. 

* **Parameters:**
  **seq_num** – Sequence number of the change to check. 
* **Returns:**
  true if delivered. False otherwise. 

### virtual bool is_acked_by_all(const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &seq_num) const = 0

Check if a specific change has been acknowledged by all Readers. Is only useful in reliable Writer. In BE Writers returns false when pending to be sent.

* **Parameters:**
  **seq_num** – Sequence number to check.
* **Returns:**
  True if acknowledged by all. 

### virtual bool wait_for_all_acked(const dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) &max_wait) = 0

Waits until all changes were acknowledged or max_wait.

* **Parameters:**
  **max_wait** – Maximum time to wait.
* **Returns:**
  True if all were acknowledged. 

### virtual void update_attributes(const [WriterAttributes](../attributes/WriterAttributes.md#_CPPv4N8eprosima7fastdds4rtps16WriterAttributesE) &att) = 0

Update the Attributes of the Writer. 

* **Parameters:**
  **att** – New attributes 

### virtual [WriterListener](WriterListener.md#_CPPv4N8eprosima7fastdds4rtps14WriterListenerE) \*get_listener() const = 0

Get listener 

* **Returns:**
  Listener 

### virtual bool set_listener([WriterListener](WriterListener.md#_CPPv4N8eprosima7fastdds4rtps14WriterListenerE) \*listener) = 0

Set the listener.

* **Parameters:**
  **listener** – Pointer to the listener. 
* **Returns:**
  True if correctly set. 

### virtual bool is_async() const = 0

Get the publication mode 

* **Returns:**
  publication mode 

### virtual bool get_disable_positive_acks() const = 0

Returns if disable positive ACKs QoS is enabled. 

* **Returns:**
  Best effort writers always return false. Reliable writers override this method. 

### virtual std::int32_t get_transport_priority() const = 0

Get the transport priority of this writer. 

* **Returns:**
  Transport priority of this writer. 

### virtual bool matched_readers_guids(std::vector<[GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE)> &guids) const = 0

Fills the provided vector with the GUIDs of the matched readers. 

* **Parameters:**
  **guids** – **[out]** Vector to be filled with the GUIDs of the matched readers. 
* **Returns:**
  True if the operation was successful. 

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

Get the connection list of this writer. 

* **Parameters:**
  **connection_list** – **[out]** of the writer 
* **Returns:**
  True if could be retrieved
