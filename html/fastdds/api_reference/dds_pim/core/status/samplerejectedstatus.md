<a id="api-pim-samplerejectedstatus"></a>

<a id="samplerejectedstatus"></a>

# SampleRejectedStatus

### struct SampleRejectedStatus

A struct storing the sample rejected status. 

### Public Members

### std::uint32_t total_count = 0

Total cumulative count of samples rejected by the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

### std::uint32_t total_count_change = 0

The incremental number of samples rejected since the last time the listener was called or the status was read. 

### [SampleRejectedStatusKind](samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) last_reason = NOT_REJECTED

Reason for rejecting the last sample rejected. If no samples have been rejected, the reason is the special value NOT_REJECTED. 

### InstanceHandle_t last_instance_handle

Handle to the instance being updated by the last sample that was rejected.
