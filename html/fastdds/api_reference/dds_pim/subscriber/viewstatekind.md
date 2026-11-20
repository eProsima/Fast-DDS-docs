<a id="api-pim-viewstatekind"></a>

<a id="viewstatekind"></a>

# ViewStateKind

### enum eprosima::fastdds::dds::ViewStateKind

Indicates whether or not an instance is new.

For each instance (identified by the key), the middleware internally maintains a view state relative to each [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). This view state can have the following values:

* [NEW_VIEW_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1afc3f4ffc1198e7a1e069abbd19af53c6a44cd6e932095889cd8e5ee6fdd805eb2) indicates that either this is the first time that the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has ever accessed samples of that instance, or else that the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has accessed previous samples of the instance, but the instance has since been reborn (i.e. become not-alive and then alive again). These two cases are distinguished by examining the [SampleInfo::disposed_generation_count](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1af5710e4f5893ad7409c3958fbf50f92e) and the [SampleInfo::no_writers_generation_count](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo_1af891c5f506667d4b8974fa8d41a618dc).

* [NOT_NEW_VIEW_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1afc3f4ffc1198e7a1e069abbd19af53c6a76ae91044fa7f3302938d41478b658d3) indicates that the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) has already accessed samples of the same instance and that the instance has not been reborn since.


The view_state available in the [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) is a snapshot of the view state of the instance relative to the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) used to access the samples at the time the collection was obtained (i.e. at the time read or take was called). The view_state is therefore the same for all samples in the returned collection that refer to the same instance.

Once an instance has been detected as not having any “live” writers and all the samples associated with the instance are “taken” from the DDSDataReader, the middleware can reclaim all local resources regarding the instance. Future samples will be treated as “never seen.” 

*Values:*

### enumerator NEW_VIEW_STATE

New instance.This latest generation of the instance has not previously been accessed. 

### enumerator NOT_NEW_VIEW_STATE

Not a new instance. This latest generation of the instance has previously been accessed.
