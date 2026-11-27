<a id="rtpsparticipantlistener"></a>

# RTPSParticipantListener

### class RTPSParticipantListener

Class [RTPSParticipantListener](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantListener) with virtual method that the user can overload to respond to certain events. 

### Public Functions

### inline virtual void on_participant_discovery([RTPSParticipant](RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*participant, [ParticipantDiscoveryStatus](participantdiscoveryinfo/ParticipantDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps26ParticipantDiscoveryStatusE) reason, const [ParticipantBuiltinTopicData](../builtin/data/ParticipantBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27ParticipantBuiltinTopicDataE) &info, bool &should_be_ignored)

This method is called when a new Participant is discovered, or a previously discovered participant changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[out]** Pointer to the Participant which discovered the remote participant. 
  * **reason** – **[out]** Reason of the change in the status of the discovered participant. 
  * **info** – **[out]** Remote participant information. User can take ownership of the object. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered Participant. 

### inline virtual void on_reader_discovery([RTPSParticipant](RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*participant, [ReaderDiscoveryStatus](participantdiscoveryinfo/ReaderDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21ReaderDiscoveryStatusE) reason, const [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &info, bool &should_be_ignored)

This method is called when a new Reader is discovered, or a previously discovered reader changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote reader. 
  * **reason** – **[in]** The reason motivating this method to be called. 
  * **info** – **[in]** Remote reader information. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered reader. 

### inline virtual void on_writer_discovery([RTPSParticipant](RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*participant, [WriterDiscoveryStatus](participantdiscoveryinfo/WriterDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21WriterDiscoveryStatusE) reason, const [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &info, bool &should_be_ignored)

This method is called when a new Writer is discovered, or a previously discovered writer changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote writer. 
  * **reason** – **[in]** The reason motivating this method to be called. 
  * **info** – **[in]** Remote writer information. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered writer. 

### inline virtual bool should_endpoints_match(const [RTPSParticipant](RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) \*participant, const [SubscriptionBuiltinTopicData](../builtin/data/SubscriptionBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps28SubscriptionBuiltinTopicDataE) &reader_info, const [PublicationBuiltinTopicData](../builtin/data/PublicationBuiltinTopicData.md#_CPPv4N8eprosima7fastdds4rtps27PublicationBuiltinTopicDataE) &writer_info)

This method is called to check whether a discovered DataReader and DataWriter should be matched or not. It is called after successful validation of the standard DDS matching rules. By default, they will match, but the user can override this method to implement custom matching logic.

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote endpoints. 
  * **reader_info** – **[in]** Remote reader information. 
  * **writer_info** – **[in]** Remote writer information.
* **Returns:**
  true if the endpoints should be matched, false otherwise.
