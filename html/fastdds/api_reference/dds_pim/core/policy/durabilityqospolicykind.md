<a id="api-pim-durabilityqospolicykind"></a>

<a id="durabilityqospolicykind"></a>

# DurabilityQosPolicyKind

### enum eprosima::fastdds::dds::DurabilityQosPolicyKind

Enum DurabilityQosPolicyKind_t, different kinds of durability for [DurabilityQosPolicy](durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy). 

*Values:*

### enumerator VOLATILE_DURABILITY_QOS

The Service does not need to keep any samples of data-instances on behalf of any [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that is not known by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) at the time the instance is written. In other words the Service will only attempt to provide the data to existing subscribers 

### enumerator TRANSIENT_LOCAL_DURABILITY_QOS

For TRANSIENT_LOCAL, the service is only required to keep the data in the memory of the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) that wrote the data and the data is not required to survive the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

### enumerator TRANSIENT_DURABILITY_QOS

For TRANSIENT, the service is only required to keep the data in memory and not in permanent storage; but the data is not tied to the lifecycle of the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) and will, in general, survive it. 

### enumerator PERSISTENT_DURABILITY_QOS

Data is kept on permanent storage, so that they can outlive a system session.

#### WARNING
Not Supported
