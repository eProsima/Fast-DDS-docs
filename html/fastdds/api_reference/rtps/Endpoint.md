<a id="endpoint"></a>

# Endpoint

### class Endpoint

Class [Endpoint](#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint), all entities of the RTPS network derive from this class. Although the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) is also defined as an endpoint in the RTPS specification, in this implementation the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) class **does not** inherit from the endpoint class. Each [Endpoint](#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) object owns a pointer to the [RTPSParticipant](participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) it belongs to. 

Subclassed by [eprosima::fastdds::rtps::RTPSReader](reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader), [eprosima::fastdds::rtps::RTPSWriter](writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter)

### Public Functions

### inline const [GUID_t](common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &getGuid() const

Get associated GUID 

* **Returns:**
  Associated GUID 

### inline RecursiveTimedMutex &getMutex()

Get mutex 

* **Returns:**
  Associated Mutex 

### inline [EndpointAttributes](attributes/EndpointAttributes.md#_CPPv4N8eprosima7fastdds4rtps18EndpointAttributesE) &getAttributes()

Get associated attributes 

* **Returns:**
  [Endpoint](#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) attributes
