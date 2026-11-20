<a id="python-api-pim-sampleinfo"></a>

<a id="sampleinfo"></a>

# SampleInfo

### *class* fastdds.SampleInfo

SampleInfo is the information that accompanies each sample that is ‘read’ or ‘taken.’

#### *property* absolute_generation_rank

the generation difference between the time the sample was received, and the time the most recent sample was received. The most recent sample used for the calculation may or may not be in the returned collection

#### *property* disposed_generation_count

number of times the instance had become alive after it was disposed

#### *property* generation_rank

the generation difference between the time the sample was received, and the time the most recent sample in the collection was received.

#### *property* has_more_replies

Flag to indicate if there are more replies (Extension for RPC)

#### *property* instance_handle

identifies locally the corresponding instance

#### *property* instance_state

indicates whether the instance is currently in existence or, if it has been disposed, the reason why it was disposed.

#### *property* no_writers_generation_count

number of times the instance had become alive after it was disposed because no writers

#### *property* original_writer_info

Original writer info

#### *property* publication_handle

identifies locally the DataWriter that modified the instance Is the same InstanceHandle_t that is returned by the operation get_matched_publications on the DataReader

#### *property* reception_timestamp

time provided by the DataReader when the sample was added to its history

#### *property* related_sample_identity

Related Sample Identity (Extension for RPC)

#### *property* sample_identity

Sample Identity (Extension for RPC)

#### *property* sample_rank

number of samples related to the same instance that follow in the collection

#### *property* sample_state

indicates whether or not the corresponding data sample has already been read

#### *property* source_timestamp

time provided by the DataWriter when the sample was written

#### *property* thisown

The membership flag

#### *property* valid_data

whether the DataSample contains data or is only used to communicate of a change in the instance

#### *property* view_state

indicates whether the DataReader has already seen samples for the most-current generation of the related instance.
