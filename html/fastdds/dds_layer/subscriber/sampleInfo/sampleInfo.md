<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-sampleinfo"></a>

<a id="sampleinfo"></a>

# SampleInfo

When a sample is retrieved from the [DataReader](../dataReader/dataReader.md#dds-layer-subscriber-datareader), in addition to the sample data,
a [`SampleInfo`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) instance is returned.
This object contains additional information that complements the returned data value and helps on its interpretation.
For example, if the [valid_data](#dds-layer-subscriber-sampleinfo-validdata) value is `false`, the
DataReader is not informing the application about a new value in the data instance,
but a change on its status, and the returned data value must be discarded.

Please, refer to the section [Accessing received data](../dataReader/readingData.md#dds-layer-subscriber-accessreceived) for more information regarding how received
data can be accessed on the DataReader.

The following sections describe the data members of [`SampleInfo`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE)
and the meaning of each one in relation to the returned sample data.

* [sample_state](#dds-layer-subscriber-sampleinfo-samplestate)
* [view_state](#dds-layer-subscriber-sampleinfo-viewstate)
* [instance_state](#dds-layer-subscriber-sampleinfo-instancestate)
* [disposed_generation_count](#dds-layer-subscriber-sampleinfo-disposedgenerationcount)
* [no_writers_generation_count](#dds-layer-subscriber-sampleinfo-nowritersgenerationcount)
* [sample_rank](#dds-layer-subscriber-sampleinfo-samplerank)
* [generation_rank](#dds-layer-subscriber-sampleinfo-generationrank)
* [absolute_generation_rank](#dds-layer-subscriber-sampleinfo-absolutegenerationrank)
* [source_timestamp](#dds-layer-subscriber-sampleinfo-sourcetimestamp)
* [instance_handle](#dds-layer-subscriber-sampleinfo-instancehandle)
* [publication_handle](#dds-layer-subscriber-sampleinfo-publicationhandle)
* [valid_data](#dds-layer-subscriber-sampleinfo-validdata)
* [sample_identity](#dds-layer-subscriber-sampleinfo-sampleidentity)
* [related_sample_identity](#dds-layer-subscriber-sampleinfo-relatedsampleidentity)

<a id="dds-layer-subscriber-sampleinfo-samplestate"></a>

<a id="sample-state"></a>

## sample_state

[`sample_state`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo12sample_stateE) indicates whether or not the corresponding data sample has already
been read previously.
It can take one of these values:

* **READ**: The data sample has already been *read* or *taken* previously.
* **NOT_READ**: This is the first time this data sample has been retrieved.

<a id="dds-layer-subscriber-sampleinfo-viewstate"></a>

<a id="view-state"></a>

## view_state

[`view_state`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo10view_stateE) indicates whether or not this is the very first sample
of this data instance that the DataReader retrieves.
It can take one of these values:

* **NEW**: This is the first time a sample of this instance is retrieved.
* **NOT_NEW**: Other samples of this instance have been retrieved previously.

<a id="dds-layer-subscriber-sampleinfo-instancestate"></a>

<a id="instance-state"></a>

## instance_state

[`instance_state`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo14instance_stateE) indicates whether the instance is currently in existence
or it has been disposed.
In the latter case, it also provides information about the reason for the disposal.
It can take one of these values:

* **ALIVE**: The instance is currently in existence.
* **NOT_ALIVE_DISPOSED**: A remote [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) disposed the instance.
* **NOT_ALIVE_NO_WRITERS**: The DataReader disposed the instance because no remote
  DataWriter that was publishing the instance is *alive*.

<a id="dds-layer-subscriber-sampleinfo-disposedgenerationcount"></a>

<a id="disposed-generation-count"></a>

## disposed_generation_count

[`disposed_generation_count`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo25disposed_generation_countE) indicates the number of times the instance had become alive after it was
disposed.

<a id="dds-layer-subscriber-sampleinfo-nowritersgenerationcount"></a>

<a id="no-writers-generation-count"></a>

## no_writers_generation_count

[`no_writers_generation_count`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo27no_writers_generation_countE) indicates the number of times the instance had become alive after it was
disposed as `NOT_ALIVE_NO_WRITERS`.

<a id="dds-layer-subscriber-sampleinfo-samplerank"></a>

<a id="sample-rank"></a>

## sample_rank

[`sample_rank`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo11sample_rankE) indicates the number of samples of the same instance that have been received after
this one.
For example, a value of `5` means that there are 5 newer samples available on the DataReader.

#### NOTE
Currently the [`sample_rank`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo11sample_rankE) is not implemented, and its value is always set to `0`.
It will be implemented on a future release of *Fast DDS*.

<a id="dds-layer-subscriber-sampleinfo-generationrank"></a>

<a id="generation-rank"></a>

## generation_rank

[`generation_rank`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo15generation_rankE) indicates the number of times the instance was disposed and become alive again
between the time the sample was received and the time the most recent sample of the same instance
that is still held in the collection was received.

#### NOTE
Currently the [`generation_rank`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo15generation_rankE) is not implemented, and its value is always set to `0`.
It will be implemented on a future release of *Fast DDS*.

<a id="dds-layer-subscriber-sampleinfo-absolutegenerationrank"></a>

<a id="absolute-generation-rank"></a>

## absolute_generation_rank

`absolute_generation_rank` indicates the number of times the instance was disposed and become alive
again between the time the sample was received and the time the most recent sample of the same instance
(which may not be in the collection) was received.

#### NOTE
Currently the `absolute_generation_rank` is not implemented, and its value is always set to `0`.
It will be implemented on a future release of *Fast DDS*.

<a id="dds-layer-subscriber-sampleinfo-sourcetimestamp"></a>

<a id="source-timestamp"></a>

## source_timestamp

[`source_timestamp`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo16source_timestampE) holds the time stamp provided by the DataWriter when
the sample was published.

<a id="dds-layer-subscriber-sampleinfo-instancehandle"></a>

<a id="instance-handle"></a>

## instance_handle

[`instance_handle`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo15instance_handleE) handles of the local instance.

<a id="dds-layer-subscriber-sampleinfo-publicationhandle"></a>

<a id="publication-handle"></a>

## publication_handle

[`publication_handle`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo18publication_handleE) handles of the DataWriter that published the data change.

<a id="dds-layer-subscriber-sampleinfo-validdata"></a>

<a id="valid-data"></a>

## valid_data

[`valid_data`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo10valid_dataE) is a boolean that indicates whether the data sample contains a change in the value or not.
Samples with this value set to false are used to communicate a change in the instance status, e.g.,
a change in the liveliness of the instance.
In this case, the data sample should be dismissed as all the relevant information is in the
data members of SampleInfo.

<a id="dds-layer-subscriber-sampleinfo-sampleidentity"></a>

<a id="sample-identity"></a>

## sample_identity

[`sample_identity`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo15sample_identityE) is an extension for requester-replier configuration.
It contains the DataWriter and the sequence number of the current message, and it is used
by the replier to fill the [related_sample_identity](#dds-layer-subscriber-sampleinfo-relatedsampleidentity) when it sends the reply.

<a id="dds-layer-subscriber-sampleinfo-relatedsampleidentity"></a>

<a id="related-sample-identity"></a>

## related_sample_identity

[`related_sample_identity`](../../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo23related_sample_identityE) is an extension for requester-replier configuration.
On reply messages, it contains the [sample_identity](#dds-layer-subscriber-sampleinfo-sampleidentity) of the related request message.
It is used by the requester to be able to link each reply to the appropriate request.
