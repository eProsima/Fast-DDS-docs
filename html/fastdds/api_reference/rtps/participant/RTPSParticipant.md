<a id="rtpsparticipant"></a>

# RTPSParticipant

### class RTPSParticipant

Class [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant), contains the public API for a [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

### Public Functions

### const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &getGuid() const

Get the [GUID_t](../common/Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) of the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

### void announceRTPSParticipantState()

Force the announcement of the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) state. 

### void stopRTPSParticipantAnnouncement()

Stop the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) announcement period. //TODO remove this method because is only for testing. 

### void resetRTPSParticipantAnnouncement()

Reset the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) announcement period. //TODO remove this method because is only for testing. 

### bool newRemoteWriterDiscovered(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &pguid, std::int16_t userDefinedId)

Indicate the Participant that you have discovered a new Remote Writer. This method can be used by the user to implements its own Static [Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) Discovery Protocol 

* **Parameters:**
  * **pguid** – [GUID_t](../common/Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) of the discovered Writer. 
  * **userDefinedId** – ID of the discovered Writer. 
* **Returns:**
  True if correctly added. 

### bool newRemoteReaderDiscovered(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &pguid, std::int16_t userDefinedId)

Indicate the Participant that you have discovered a new Remote Reader. This method can be used by the user to implements its own Static [Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) Discovery Protocol 

* **Parameters:**
  * **pguid** – [GUID_t](../common/Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) of the discovered Reader. 
  * **userDefinedId** – ID of the discovered Reader. 
* **Returns:**
  True if correctly added. 

### std::uint32_t getRTPSParticipantID() const

Get the Participant ID. 

* **Returns:**
  Participant ID. 

### bool register_writer([RTPSWriter](../writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*rtps_writer, const TopicDescription &topic, const fastdds::dds::WriterQos &qos)

Register a Writer in the BuiltinProtocols.

* **Parameters:**
  * **rtps_writer** – Pointer to the [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 
  * **topic** – Information regarding the topic where the writer is registering. 
  * **qos** – Qos policies of the writer.
* **Returns:**
  True if correctly registered. 

### dds::ReturnCode_t register_writer([RTPSWriter](../writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*rtps_writer, const TopicDescription &topic, const [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &pub_builtin_topic_data)

Register a Writer in the BuiltinProtocols.

* **Parameters:**
  * **rtps_writer** – Pointer to the [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 
  * **topic** – Information regarding the topic where the writer is registering. 
  * **pub_builtin_topic_data** – Information on the publication endpoint.
* **Returns:**
  True if correctly registered. 

### bool register_reader([RTPSReader](../reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*rtps_reader, const TopicDescription &topic, const fastdds::dds::ReaderQos &qos, const [ContentFilterProperty](../builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterPropertyE) \*content_filter = nullptr)

Register a Reader in the BuiltinProtocols.

* **Parameters:**
  * **rtps_reader** – Pointer to the [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 
  * **topic** – Information regarding the topic where the reader is registering. 
  * **qos** – Qos policies of the reader. 
  * **content_filter** – Optional content filtering information.
* **Returns:**
  True if correctly registered. 

### dds::ReturnCode_t register_reader([RTPSReader](../reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*rtps_reader, const TopicDescription &topic, const [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &sub_builtin_topic_data, const [ContentFilterProperty](../builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterPropertyE) \*content_filter = nullptr)

Register a Reader in the BuiltinProtocols.

* **Parameters:**
  * **rtps_reader** – Pointer to the [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 
  * **topic** – Information regarding the topic where the reader is registering. 
  * **sub_builtin_topic_data** – Information on the subscription endpoint. 
  * **content_filter** – Optional content filtering information.
* **Returns:**
  True if correctly registered. 

### void update_attributes(const [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) &patt)

Update participant attributes. 

* **Parameters:**
  **patt** – New participant attributes. 

### bool update_writer([RTPSWriter](../writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*rtps_writer, const fastdds::dds::WriterQos &wqos)

Update local writer QoS 

* **Parameters:**
  * **rtps_writer** – Writer to update. 
  * **wqos** – New QoS for the writer. 
* **Returns:**
  True on success 

### bool update_reader([RTPSReader](../reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*rtps_reader, const fastdds::dds::ReaderQos &rqos, const [ContentFilterProperty](../builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterPropertyE) \*content_filter = nullptr)

Update local reader QoS 

* **Parameters:**
  * **rtps_reader** – Reader to update. 
  * **rqos** – New QoS for the reader. 
  * **content_filter** – Optional content filtering information. 
* **Returns:**
  True on success 

### std::vector<std::string> getParticipantNames() const

Returns a list with the participant names. 

* **Returns:**
  list of participant names. 

### const [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) &get_attributes() const

Get a reference of the current state of the [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantAttributes). 

#### WARNING
The returned reference is not thread safe. It is recommended to use [copy_attributes()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant_1a973c6356ff2bd0d8b3dd2b3417ff3a81) instead to get a thread safe copy of the attributes. 

* **Returns:**
  [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantAttributes) reference. 

### const RTPSParticipantConstantAttributes &get_const_attributes() const

Get a const reference of RTPSParticipantConstantAttributes of this RTPSParticipantImpl. This method is thread safe because it returns a const reference to the internal constant attributes. 

* **Returns:**
  A const reference to the RTPSParticipantConstantAttributes of this RTPSParticipantImpl. 

### const RTPSParticipantMutableAttributes get_mutable_attributes() const

Get a const copy of RTPSParticipantMutableAttributes of this RTPSParticipantImpl. This method is thread safe because it returns a const copy of the internal mutable attributes. 

* **Returns:**
  A const copy of the RTPSParticipantMutableAttributes of this RTPSParticipantImpl. 

### [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) copy_attributes() const

Get a copy of the current state of the [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantAttributes). 

* **Returns:**
  [RTPSParticipantAttributes](../attributes/RTPSParticipantAttributes.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantAttributes) copy. 

### std::uint32_t getMaxMessageSize() const

Retrieves the maximum message size. 

### std::uint32_t getMaxDataSize() const

Retrieves the maximum data size. 

### WLP \*wlp() const

A method to retrieve the built-in writer liveliness protocol. 

* **Returns:**
  Writer liveliness protocol 

### bool get_new_entity_id([EntityId_t](../common/EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entityId)

Fills a new entityId if set to unknown, or checks if a entity already exists with that entityId in other case. 

* **Parameters:**
  **entityId** – to check of fill. If filled, EntityKind will be “vendor-specific” (0x01) 
* **Returns:**
  True if filled or the entityId is available. 

### void set_check_type_function(std::function<bool(const std::string&)> &&check_type)

Allows setting a function to check if a type is already known by the top level API participant. 

### void set_listener([RTPSParticipantListener](RTPSParticipantListener.md#_CPPv4N8eprosima7fastdds4rtps23RTPSParticipantListenerE) \*listener)

Modifies the participant listener. 

* **Parameters:**
  **listener** – 

### void set_congestion_control_listener(CongestionControlListener \*listener)

 **[Pro]**  Sets the user listener that observes congestion-control meta-information.

Congestion control is participant-scoped; this listener observes all reliable writers of the participant. The listener object must outlive the participant.

* **Parameters:**
  **listener** – Listener to notify, or nullptr to detach. 

### std::uint32_t get_domain_id() const

Retrieves the DomainId. 

### void enable()

This operation enables the RTPSParticipantImpl. 

### bool ignore_participant(const [GuidPrefix_t](../common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &participant_guid)

Ignore all messages coming from the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

* **Parameters:**
  **participant_guid** – **[in]** [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) GUID to be ignored 
* **Returns:**
  True if correctly included into the ignore collection. False otherwise. 

### bool ignore_writer(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid)

Ignore all messages coming from the [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 

* **Parameters:**
  **writer_guid** – **[in]** [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter) GUID to be ignored 
* **Returns:**
  True if correctly included into the ignore collection. False otherwise. 

### bool ignore_reader(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &reader_guid)

Ignore all messages coming from the [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 

* **Parameters:**
  **reader_guid** – **[in]** [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader) GUID to be ignored 
* **Returns:**
  True if correctly included into the ignore collection. False otherwise. 

### std::vector<TransportNetmaskFilterInfo> get_netmask_filter_info() const

Returns registered transports’ netmask filter information (transport’s netmask filter kind and allowlist). 

* **Returns:**
  A vector with all registered transports’ netmask filter information. 

### bool get_publication_info(fastdds::rtps::[PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &data, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid) const

Fills the provided publication discovery data with the information of the writer identified by writer_guid. 

* **Parameters:**
  * **data** – **[out]** publication discovery data to fill. 
  * **writer_guid** – **[in]** GUID of the writer to get the information from. 
* **Returns:**
  True if the writer was found and the data was filled. 

### bool get_subscription_info(fastdds::rtps::[SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &data, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &reader_guid) const

Fills the provided subscription discovery data with the information of the reader identified by reader_guid. 

* **Parameters:**
  * **data** – **[out]** subscription discovery data to fill. 
  * **reader_guid** – **[in]** GUID of the reader to get the information from. 
* **Returns:**
  True if the reader was found and the data was filled. 

### bool is_security_enabled_for_writer(const [WriterAttributes](../attributes/WriterAttributes.md#_CPPv4N8eprosima7fastdds4rtps16WriterAttributesE) &writer_attributes)

Checks whether the writer has security attributes enabled. 

* **Parameters:**
  **writer_attributes** – Attributes of the writer as given to the RTPSParticipantImpl::create_writer 

### bool is_security_enabled_for_reader(const [ReaderAttributes](../attributes/ReaderAttributes.md#_CPPv4N8eprosima7fastdds4rtps16ReaderAttributesE) &reader_attributes)

Checks whether the reader has security attributes enabled. 

* **Parameters:**
  **reader_attributes** – Attributes of the reader as given to the RTPSParticipantImpl::create_reader 

### bool add_statistics_listener(std::shared_ptr<fastdds::statistics::IListener> listener, std::uint32_t kind)

Add a listener to receive statistics backend callbacks 

* **Parameters:**
  * **listener** – 
  * **kind** – combination of fastdds::statistics::EventKind flags used as a mask. Events to notify. 
* **Returns:**
  true if successfully added 

### bool remove_statistics_listener(std::shared_ptr<fastdds::statistics::IListener> listener, std::uint32_t kind)

Remove a listener from receiving statistics backend callbacks 

* **Parameters:**
  * **listener** – 
  * **kind** – combination of fastdds::statistics::EventKind flags used as a mask. Events to ignore. 
* **Returns:**
  true if successfully removed 

### void set_enabled_statistics_writers_mask(std::uint32_t enabled_writers)

Set the enabled statistics writers mask. 

* **Parameters:**
  **enabled_writers** – The new mask to set 

### const fastdds::statistics::rtps::IStatusObserver \*create_monitor_service(fastdds::statistics::rtps::IStatusQueryable &status_queryable)

Creates the monitor service in this [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) with the provided interfaces.

#### NOTE
Not supported yet. Currently always returns nullptr 

* **Parameters:**
  **status_queryable** – reference to the object implementing the StatusQueryable interface. It will usually be the DDS DomainParticipant
* **Returns:**
  A const pointer to the listener (implemented within the [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant))

### bool create_monitor_service()

Creates the monitor service in this [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) with a simple default implementation of the IStatusQueryable.

#### NOTE
Not supported yet. Currently always returns false 

* **Returns:**
  true if the monitor service could be correctly created.

### bool is_monitor_service_created() const

Returns whether the monitor service in created in this [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant).

#### NOTE
Not supported yet. Currently always returns false 

* **Returns:**
  true if the monitor service is created. 
* **Returns:**
  false otherwise.

### bool enable_monitor_service() const

Enables the monitor service in this [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant).

#### NOTE
Not supported yet. Currently always returns false 

* **Returns:**
  true if the monitor service could be correctly enabled.

### bool disable_monitor_service() const

Disables the monitor service in this [RTPSParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). Does nothing if the service was not enabled before.

#### NOTE
Not supported yet. Currently always returns false 

* **Returns:**
  true if the monitor service could be correctly disabled. 
* **Returns:**
  false if the service could not be properly disabled or if the monitor service was not previously enabled.

### bool fill_discovery_data_from_cdr_message([ParticipantBuiltinTopicData](../builtin/data/ParticipantBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27ParticipantBuiltinTopicDataE) &data, const fastdds::statistics::MonitorServiceStatusData &msg)

fills in the [ParticipantBuiltinTopicData](../builtin/data/ParticipantBuiltinTopicData.md#structeprosima_1_1fastdds_1_1rtps_1_1ParticipantBuiltinTopicData) from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  true if the operation succeeds. 

### bool fill_discovery_data_from_cdr_message([PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &data, const fastdds::statistics::MonitorServiceStatusData &msg)

fills in the [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#structeprosima_1_1fastdds_1_1rtps_1_1PublicationBuiltinTopicData) from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill. 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  true if the operation succeeds. 

### bool fill_discovery_data_from_cdr_message([SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &data, const fastdds::statistics::MonitorServiceStatusData &msg)

fills in the [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#structeprosima_1_1fastdds_1_1rtps_1_1SubscriptionBuiltinTopicData) from a MonitorService Message

* **Parameters:**
  * **data** – **[out]** Proxy to fill. 
  * **msg** – **[in]** MonitorService Message to get the proxy information from.
* **Returns:**
  true if the operation succeeds.
