<a id="api-pim-readerresourcelimitsqos"></a>

<a id="readerresourcelimitsqos"></a>

# ReaderResourceLimitsQos

### class ReaderResourceLimitsQos

Qos Policy to configure the limit of the reader resources. 

### Public Functions

### ReaderResourceLimitsQos() = default

Constructor. 

### virtual ~ReaderResourceLimitsQos() = default

Destructor. 

### Public Members

### fastdds::ResourceLimitedContainerConfig matched_publisher_allocation

Matched publishers allocation limits. 

### fastdds::ResourceLimitedContainerConfig sample_infos_allocation = {32u}

[SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) allocation limits. 

### fastdds::ResourceLimitedContainerConfig outstanding_reads_allocation = {2u}

Loaned collections allocation limits. 

### std::int32_t max_samples_per_read = 32

Maximum number of samples to return on a single call to read / take.

This attribute is a signed integer to be consistent with the `max_samples` argument of [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) methods, but should always have a strict positive value. Bear in mind that a big number here may cause the creation of the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to fail due to pre-allocation of internal resources.

Default value: 32.
