<a id="api-pim-icontentfilter"></a>

<a id="icontentfilter"></a>

# IContentFilter

### struct IContentFilter

The interface that content filter objects should implement. 

### Public Functions

### virtual bool evaluate(const SerializedPayload &payload, const [FilterSampleInfo](#_CPPv4N8eprosima7fastdds3dds14IContentFilter16FilterSampleInfoE) &sample_info, const GUID_t &reader_guid) const = 0

Evaluate if a serialized payload should be accepted by certain reader.

* **Parameters:**
  * **payload** – **[in]** The serialized payload of the sample being evaluated. 
  * **sample_info** – **[in]** The accompanying sample information. 
  * **reader_guid** – **[in]** The GUID of the reader for which the filter is being evaluated.
* **Returns:**
  whether the sample should be accepted for the specified reader. 

### struct FilterSampleInfo

Selected information from the cache change that is passed to the content filter object on payload evaluation. 

### Public Members

### SampleIdentity sample_identity

Identity of the sample being filtered. 

### SampleIdentity related_sample_identity

Identity of a sample related to the one being filtered. 

### std::shared_ptr<fastdds::rtps::[WriteParams](../../rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE)::[UserWriteData](../../rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParams13UserWriteDataE)> user_write_data

Extra write information that can be used by the prefilter.
