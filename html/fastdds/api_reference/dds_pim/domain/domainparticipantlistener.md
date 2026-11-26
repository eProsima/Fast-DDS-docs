<a id="api-pim-domainparticipantlistener"></a>

<a id="domainparticipantlistener"></a>

# DomainParticipantListener

### class DomainParticipantListener : public eprosima::fastdds::dds::[PublisherListener](../publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE), public eprosima::fastdds::dds::[SubscriberListener](../subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE), public eprosima::fastdds::dds::[TopicListener](../topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE)

Class [DomainParticipantListener](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener), overrides behaviour towards certain events. 

### Public Functions

### inline DomainParticipantListener()

Constructor. 

### inline virtual ~DomainParticipantListener()

Destructor. 

### inline virtual void on_participant_discovery([DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, fastdds::rtps::[ParticipantDiscoveryStatus](../../rtps/participant/participantdiscoveryinfo/ParticipantDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps26ParticipantDiscoveryStatusE) reason, const ParticipantBuiltinTopicData &info, bool &should_be_ignored)

This method is called when a new Participant is discovered, or a previously discovered participant changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[out]** Pointer to the Participant which discovered the remote participant. 
  * **reason** – **[out]** Reason of the change in the status of the discovered participant. 
  * **info** – **[out]** Remote participant information. User can take ownership of the object. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered Participant. 

### inline virtual void onParticipantAuthentication([DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, rtps::[ParticipantAuthenticationInfo](../../rtps/participant/participantdiscoveryinfo/ParticipantAuthenticationInfo.md#_CPPv4N8eprosima7fastdds4rtps29ParticipantAuthenticationInfoE) &&info)

This method is called when a new Participant is authenticated. 

* **Parameters:**
  * **participant** – **[out]** Pointer to the authenticated Participant. 
  * **info** – **[out]** Remote participant authentication information. User can take ownership of the object. 

### inline virtual void on_data_reader_discovery([DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, rtps::[ReaderDiscoveryStatus](../../rtps/participant/participantdiscoveryinfo/ReaderDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21ReaderDiscoveryStatusE) reason, const SubscriptionBuiltinTopicData &info, bool &should_be_ignored)

This method is called when a new [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) is discovered, or a previously discovered [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote reader. 
  * **reason** – **[in]** The reason motivating this method to be called. 
  * **info** – **[in]** Remote reader information. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered reader. 

### inline virtual void on_data_writer_discovery([DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, rtps::[WriterDiscoveryStatus](../../rtps/participant/participantdiscoveryinfo/WriterDiscoveryStatus.md#_CPPv4N8eprosima7fastdds4rtps21WriterDiscoveryStatusE) reason, const PublicationBuiltinTopicData &info, bool &should_be_ignored)

This method is called when a new [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is discovered, or a previously discovered [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) changes its QOS or is removed. 

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote writer. 
  * **reason** – **[in]** The reason motivating this method to be called. 
  * **info** – **[in]** Remote writer information. 
  * **should_be_ignored** – **[out]** Flag to indicate the library to automatically ignore the discovered writer. 

### inline virtual bool should_endpoints_match(const [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, const SubscriptionBuiltinTopicData &reader_info, const PublicationBuiltinTopicData &writer_info)

This method is called to check whether a discovered [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) should be matched or not. It is called after successful validation of the standard DDS matching rules. By default, they will match, but the user can override this method to implement custom matching logic.

* **Parameters:**
  * **participant** – **[in]** Pointer to the Participant which discovered the remote endpoints. 
  * **reader_info** – **[in]** Remote reader information. 
  * **writer_info** – **[in]** Remote writer information.
* **Returns:**
  true if the endpoints should be matched, false otherwise.
