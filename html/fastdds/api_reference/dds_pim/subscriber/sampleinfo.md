<a id="api-pim-sampleinfo"></a>

<a id="sampleinfo"></a>

# SampleInfo

### struct SampleInfo

[SampleInfo](#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) is the information that accompanies each sample that is ‘read’ or ‘taken. 

’ 

### Public Members

### [SampleStateKind](samplestatekind.md#_CPPv4N8eprosima7fastdds3dds15SampleStateKindE) sample_state

indicates whether or not the corresponding data sample has already been read 

### [ViewStateKind](viewstatekind.md#_CPPv4N8eprosima7fastdds3dds13ViewStateKindE) view_state

indicates whether the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has already seen samples for the most-current generation of the related instance. 

### [InstanceStateKind](instancestatekind.md#_CPPv4N8eprosima7fastdds3dds17InstanceStateKindE) instance_state

indicates whether the instance is currently in existence or, if it has been disposed, the reason why it was disposed. 

### std::int32_t disposed_generation_count

number of times the instance had become alive after it was disposed 

### std::int32_t no_writers_generation_count

number of times the instance had become alive after it was disposed because no writers 

### std::int32_t sample_rank

number of samples related to the same instance that follow in the collection 

### std::int32_t generation_rank

the generation difference between the time the sample was received, and the time the most recent sample in the collection was received. 

### std::int32_t absolute_generation_rank

the generation difference between the time the sample was received, and the time the most recent sample was received. The most recent sample used for the calculation may or may not be in the returned collection 

### rtps::[Time_t](../../rtps/common/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) source_timestamp

time provided by the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) when the sample was written 

### rtps::[Time_t](../../rtps/common/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) reception_timestamp

time provided by the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) when the sample was added to its history 

### InstanceHandle_t instance_handle

identifies locally the corresponding instance 

### InstanceHandle_t publication_handle

identifies locally the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that modified the instance

Is the same InstanceHandle_t that is returned by the operation get_matched_publications on the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

### rtps::OriginalWriterInfo original_writer_info

Original writer info. 

### bool valid_data

whether the DataSample contains data or is only used to communicate of a change in the instance 

### rtps::[SampleIdentity](../../rtps/common/SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) sample_identity

Sample Identity (Extension for RPC) 

### rtps::[SampleIdentity](../../rtps/common/SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) related_sample_identity

Related Sample Identity (Extension for RPC) 

### bool has_more_replies = false

Flag to indicate if there are more replies (Extension for RPC)
