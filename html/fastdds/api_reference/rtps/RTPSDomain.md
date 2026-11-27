<a id="rtpsdomain"></a>

# RTPSDomain

### class RTPSDomain

Class [RTPSDomain](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain),it manages the creation and destruction of [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter) and [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). It stores a list of all created [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). It has only static methods. 

### Public Static Functions

### static void set_filewatch_thread_config(const fastdds::rtps::[ThreadSettings](attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &watch_thread, const fastdds::rtps::[ThreadSettings](attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &callback_thread)

Method to set the configuration of the threads created by the file watcher for the environment file. In order for these settings to take effect, this method must be called before the first call to [createParticipant](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1addd6e5a621a4935d42d25e5016aad1d6).

* **Parameters:**
  * **watch_thread** – Settings for the thread watching the environment file. 
  * **callback_thread** – Settings for the thread executing the callback when the environment file changed. 

### static void stopAll()

Method to shut down all RTPSParticipants, readers, writers, etc. It must be called at the end of the process to avoid memory leaks. It also shut downs the DomainRTPSParticipant.

* **Post:**
  After this call, all the pointers to RTPS entities are invalidated and their use may result in undefined behaviour. 

### static [RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*createParticipant(std::uint32_t domain_id, const [RTPSParticipantAttributes](attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) &attrs, [RTPSParticipantListener](participant/RTPSParticipantListener.md#_CPPv4N8eprosima7fastdds4rtps23RTPSParticipantListenerE) \*plisten = nullptr)

Create a [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSParticipant()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ab511d587f31a0c54f67ad6d65c65f7bf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **domain_id** – DomainId to be used by the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) (80 by default). 
  * **attrs** – [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) Attributes. 
  * **plisten** – Pointer to the ParticipantListener. 
* **Returns:**
  Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant).

### static [RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*createParticipant(std::uint32_t domain_id, bool enabled, const [RTPSParticipantAttributes](attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) &attrs, [RTPSParticipantListener](participant/RTPSParticipantListener.md#_CPPv4N8eprosima7fastdds4rtps23RTPSParticipantListenerE) \*plisten = nullptr)

Create a [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSParticipant()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ab511d587f31a0c54f67ad6d65c65f7bf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **domain_id** – DomainId to be used by the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) (80 by default). 
  * **enabled** – True if the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) should be enabled on creation. False if it will be enabled later with [RTPSParticipant::enable()](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant_1a78b66a8d8a6005a5fca305af1417cf77)
  * **attrs** – [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) Attributes. 
  * **plisten** – Pointer to the ParticipantListener. 
* **Returns:**
  Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant).

### static [RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*create_client_server_participant(std::uint32_t domain_id, bool enabled, const [RTPSParticipantAttributes](attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) &attrs, [RTPSParticipantListener](participant/RTPSParticipantListener.md#_CPPv4N8eprosima7fastdds4rtps23RTPSParticipantListenerE) \*plisten = nullptr)

Create a [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) as default server or client if ROS_MASTER_URI environment variable is set. It also configures ROS 2 Easy Mode IP if ROS2_EASY_MODE_URI environment variable is set and it was empty in the input attributes. 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSParticipant()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ab511d587f31a0c54f67ad6d65c65f7bf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **domain_id** – DomainId to be used by the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **enabled** – True if the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) should be enabled on creation. False if it will be enabled later with [RTPSParticipant::enable()](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant_1a78b66a8d8a6005a5fca305af1417cf77)
  * **attrs** – [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) Attributes. 
  * **plisten** – Pointer to the ParticipantListener. 
* **Returns:**
  Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant).

### static [RTPSWriter](writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*createRTPSWriter([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p, [WriterAttributes](attributes/WriterAttributes.md#_CPPv4N8eprosima7fastdds4rtps16WriterAttributesE) &watt, [WriterHistory](history/WriterHistory.md#_CPPv4N8eprosima7fastdds4rtps13WriterHistoryE) \*hist, [WriterListener](writer/WriterListener.md#_CPPv4N8eprosima7fastdds4rtps14WriterListenerE) \*listen = nullptr)

Create a [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter) in a participant.

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSWriter()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1addee9b6ce47ffbae1142ca4503370bc6) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **p** – Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **watt** – Writer Attributes. 
  * **hist** – Pointer to the [WriterHistory](history/WriterHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 
  * **listen** – Pointer to the [WriterListener](writer/WriterListener.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterListener).
* **Returns:**
  Pointer to the created [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter).

### static [RTPSWriter](writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*createRTPSWriter([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p, const [EntityId_t](common/EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entity_id, [WriterAttributes](attributes/WriterAttributes.md#_CPPv4N8eprosima7fastdds4rtps16WriterAttributesE) &watt, [WriterHistory](history/WriterHistory.md#_CPPv4N8eprosima7fastdds4rtps13WriterHistoryE) \*hist, [WriterListener](writer/WriterListener.md#_CPPv4N8eprosima7fastdds4rtps14WriterListenerE) \*listen = nullptr)

Create a [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter) in a participant.

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSWriter()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1addee9b6ce47ffbae1142ca4503370bc6) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **p** – Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **entity_id** – Specific entity id to use for the created writer. 
  * **watt** – Writer Attributes. 
  * **hist** – Pointer to the [WriterHistory](history/WriterHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 
  * **listen** – Pointer to the [WriterListener](writer/WriterListener.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterListener).
* **Returns:**
  Pointer to the created [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter).

### static bool removeRTPSWriter([RTPSWriter](writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) \*writer)

Remove a [RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 

* **Parameters:**
  **writer** – Pointer to the writer you want to remove. 
* **Returns:**
  True if correctly removed. 

### static [RTPSReader](reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*createRTPSReader([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p, [ReaderAttributes](attributes/ReaderAttributes.md#_CPPv4N8eprosima7fastdds4rtps16ReaderAttributesE) &ratt, [ReaderHistory](history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE) \*hist, [ReaderListener](reader/ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) \*listen = nullptr)

Create a [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader) in a participant. 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSReader()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ac274fc82b8a493e23c5a1e8a9779a2cf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **p** – Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **ratt** – Reader Attributes. 
  * **hist** – Pointer to the [ReaderHistory](history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). 
  * **listen** – Pointer to the [ReaderListener](reader/ReaderListener.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderListener). 
* **Returns:**
  Pointer to the created [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader).

### static [RTPSReader](reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*createRTPSReader([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p, [ReaderAttributes](attributes/ReaderAttributes.md#_CPPv4N8eprosima7fastdds4rtps16ReaderAttributesE) &ratt, const std::shared_ptr<[IPayloadPool](history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> &payload_pool, [ReaderHistory](history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE) \*hist, [ReaderListener](reader/ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) \*listen = nullptr)

Create a RTPReader in a participant using a custom payload pool. 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSReader()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ac274fc82b8a493e23c5a1e8a9779a2cf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **p** – Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **ratt** – Reader Attributes. 
  * **payload_pool** – Shared pointer to the [IPayloadPool](history/IPayloadPool.md#classeprosima_1_1fastdds_1_1rtps_1_1IPayloadPool)
  * **hist** – Pointer to the [ReaderHistory](history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). 
  * **listen** – Pointer to the [ReaderListener](reader/ReaderListener.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderListener). 
* **Returns:**
  Pointer to the created [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader).

### static [RTPSReader](reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*createRTPSReader([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p, const [EntityId_t](common/EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entity_id, [ReaderAttributes](attributes/ReaderAttributes.md#_CPPv4N8eprosima7fastdds4rtps16ReaderAttributesE) &ratt, const std::shared_ptr<[IPayloadPool](history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> &payload_pool, [ReaderHistory](history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE) \*hist, [ReaderListener](reader/ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) \*listen = nullptr)

Create a [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader) in a participant using a custom payload pool. 

#### WARNING
The returned pointer is invalidated after a call to [removeRTPSReader()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1ac274fc82b8a493e23c5a1e8a9779a2cf) or [stopAll()](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSDomain_1a1f4c6ac705642bec982f5e36334a4042), so its use may result in undefined behaviour. 

* **Parameters:**
  * **p** – Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 
  * **entity_id** – Specific entity id to use for the created reader. 
  * **ratt** – Reader Attributes. 
  * **payload_pool** – Shared pointer to the [IPayloadPool](history/IPayloadPool.md#classeprosima_1_1fastdds_1_1rtps_1_1IPayloadPool)
  * **hist** – Pointer to the [ReaderHistory](history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). 
  * **listen** – Pointer to the [ReaderListener](reader/ReaderListener.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderListener). 
* **Returns:**
  Pointer to the created [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader).

### static bool removeRTPSReader([RTPSReader](reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) \*reader)

Remove a [RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 

* **Parameters:**
  **reader** – Pointer to the reader you want to remove. 
* **Returns:**
  True if correctly removed. 

### static bool removeRTPSParticipant([RTPSParticipant](participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*p)

Remove a [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) and delete all its associated Writers, Readers, resources, etc. 

* **Parameters:**
  **p** – **[in]** Pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant); 
* **Returns:**
  True if correct. 

### static bool get_library_settings(fastdds::LibrarySettings &library_settings)

Get the library settings. 

* **Parameters:**
  **library_settings** – LibrarySettings reference where the settings are returned. 
* **Returns:**
  True. 

### static bool set_library_settings(const fastdds::LibrarySettings &library_settings)

Set the library settings-. 

* **Parameters:**
  **library_settings** – LibrarySettings to be set. 
* **Returns:**
  False if there is any [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) already created. True if correctly set.
