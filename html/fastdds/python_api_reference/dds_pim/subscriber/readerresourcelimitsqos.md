<a id="python-api-pim-readerresourcelimitsqos"></a>

<a id="readerresourcelimitsqos"></a>

# ReaderResourceLimitsQos

### *class* fastdds.ReaderResourceLimitsQos

Qos Policy to configure the limit of the reader resources

#### clear()

#### *property* matched_publisher_allocation

Matched publishers allocation limits.

#### *property* max_samples_per_read

Maximum number of samples to return on a single call to read / take.

This attribute is a signed integer to be consistent with the `max_samples` argument of
‘DataReader’ methods, but should always have a strict positive value. Bear in mind that
a big number here may cause the creation of the DataReader to fail due to pre-allocation of
internal resources.

Default value: 32.

#### *property* outstanding_reads_allocation

Loaned collections allocation limits.

#### *property* sample_infos_allocation

SampleInfo allocation limits.

#### *property* thisown

The membership flag
