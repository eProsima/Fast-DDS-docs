<a id="rtpsparticipantallocationattributes"></a>

# RTPSParticipantAllocationAttributes

### struct RTPSParticipantAllocationAttributes

Holds allocation limits affecting collections managed by a participant. 

### Public Functions

### inline ResourceLimitedContainerConfig total_readers() const

* **Returns:**
  the allocation config for the total of readers in the system (participants \* readers) 

### inline ResourceLimitedContainerConfig total_writers() const

* **Returns:**
  the allocation config for the total of writers in the system (participants \* writers) 

### Public Members

### [RemoteLocatorsAllocationAttributes](RemoteLocatorsAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps34RemoteLocatorsAllocationAttributesE) locators

Holds limits for collections of remote locators. 

### ResourceLimitedContainerConfig participants

Defines the allocation behaviour for collections dependent on the total number of participants. 

### ResourceLimitedContainerConfig readers

Defines the allocation behaviour for collections dependent on the total number of readers per participant. 

### ResourceLimitedContainerConfig writers

Defines the allocation behaviour for collections dependent on the total number of writers per participant. 

### [SendBuffersAllocationAttributes](SendBuffersAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps31SendBuffersAllocationAttributesE) send_buffers

Defines the allocation behaviour for the send buffer manager. 

### [VariableLengthDataLimits](VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimitsE) data_limits

Holds limits for variable-length data. 

### fastdds::rtps::[ContentFilterProperty](../builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterPropertyE)::[AllocationConfiguration](../builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfigurationE) content_filter

Defines the allocation behavior of content filter discovery information.
