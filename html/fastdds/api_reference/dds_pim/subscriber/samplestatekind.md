<a id="api-pim-samplestatekind"></a>

<a id="samplestatekind"></a>

# SampleStateKind

### enum eprosima::fastdds::dds::SampleStateKind

Indicates whether or not a sample has ever been read.

For each sample received, the middleware internally maintains a sample state relative to each [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). This sample state can have the following values:

* [READ_SAMPLE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1aa0466276d249ece30c7f02578b9aabc7abde4814383bf887b908abfdbca841009) indicates that the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has already accessed that sample by means of a read or take operation

* [NOT_READ_SAMPLE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1aa0466276d249ece30c7f02578b9aabc7a55a2fa3348710eed94e4b65d040d3268) indicates that the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has not accessed that sample before.


The sample state will, in general, be different for each sample in the collection returned by read or take. 

*Values:*

### enumerator READ_SAMPLE_STATE

Sample has been read. 

### enumerator NOT_READ_SAMPLE_STATE

Sample has not been read.
