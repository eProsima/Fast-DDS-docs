<a id="api-pim-instancestatekind"></a>

<a id="instancestatekind"></a>

# InstanceStateKind

### enum eprosima::fastdds::dds::InstanceStateKind

Indicates if the samples are from an alive [eprosima::fastdds::dds::DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) or not.

For each instance, the middleware internally maintains an instance state. The instance state can be:

* [ALIVE_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a1385da9106927a8e5e6248b3dd2b9b40) indicates that (a) samples have been received for the instance, (b) there are alive [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities writing the instance, and (c) the instance has not been explicitly disposed (or else more samples have been received after it was disposed).

* [NOT_ALIVE_DISPOSED_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a01b934811bc0b615ce3b0f09a4b73aa0) indicates the instance was explicitly disposed by a [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) by means of the dispose operation.

* [NOT_ALIVE_NO_WRITERS_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10af02d8319e0b7413acd28156351e25489) indicates the instance has been declared as not-alive by the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) because it detected that there are no alive [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities writing that instance.


The precise behavior events that cause the instance state to change depends on the setting of the OWNERSHIP QoS:

* If OWNERSHIP is set to EXCLUSIVE_OWNERSHIP_QOS, then the instance state becomes [NOT_ALIVE_DISPOSED_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a01b934811bc0b615ce3b0f09a4b73aa0) only if the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that “owns” the instance explicitly disposes it. The instance state becomes [ALIVE_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a1385da9106927a8e5e6248b3dd2b9b40) again only if the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that owns the instance writes it.

* If OWNERSHIP is set to SHARED_OWNERSHIP_QOS, then the instance state becomes [NOT_ALIVE_DISPOSED_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a01b934811bc0b615ce3b0f09a4b73aa0) if any [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) explicitly disposes the instance. The instance state becomes [ALIVE_INSTANCE_STATE](#namespaceeprosima_1_1fastdds_1_1dds_1ab3b35291bbccae0044af423dda98dc10a1385da9106927a8e5e6248b3dd2b9b40) as soon as any [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) writes the instance again.


The instance state available in the [SampleInfo](sampleinfo.md#structeprosima_1_1fastdds_1_1dds_1_1SampleInfo) is a snapshot of the instance state of the instance at the time the collection was obtained (i.e. at the time read or take was called). The instance state is therefore the same for all samples in the returned collection that refer to the same instance. 

*Values:*

### enumerator ALIVE_INSTANCE_STATE

Instance is currently in existence. 

### enumerator NOT_ALIVE_DISPOSED_INSTANCE_STATE

Not alive disposed instance. The instance has been disposed by a [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

### enumerator NOT_ALIVE_NO_WRITERS_INSTANCE_STATE

Not alive no writers for instance. None of the [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) objects that are currently alive (according to the LIVELINESS QoS) are writing the instance.
