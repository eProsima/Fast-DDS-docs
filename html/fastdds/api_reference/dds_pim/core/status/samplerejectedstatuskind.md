<a id="api-pim-samplerejectedstatuskind"></a>

<a id="samplerejectedstatuskind"></a>

# SampleRejectedStatusKind

### enum eprosima::fastdds::dds::SampleRejectedStatusKind

An enum with the possible values for the sample rejected reason. 

*Values:*

### enumerator NOT_REJECTED

Default value. 

### enumerator REJECTED_BY_INSTANCES_LIMIT

Exceeds the max_instance limit. 

### enumerator REJECTED_BY_SAMPLES_LIMIT

Exceeds the max_samples limit. 

### enumerator REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT

Exceeds the max_samples_per_instance limit. 

### enumerator REJECTED_BY_UNKNOWN_INSTANCE

Instance could not be determined.
