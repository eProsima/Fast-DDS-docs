<a id="api-pim-groupdataqospolicy"></a>

<a id="groupdataqospolicy"></a>

# GroupDataQosPolicy

### class GroupDataQosPolicy : public eprosima::fastdds::dds::[GenericDataQosPolicy](genericdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE)

Class derived from [GenericDataQosPolicy](genericdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy). 

The purpose of this QoS is to allow the application to attach additional information to the created [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) or [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). The value of the GROUP_DATA is available to the application on the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) entities and is propagated by means of the built-in topics.

This QoS can be used by an application combination with the [DataReaderListener](../../subscriber/datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener) and [DataWriterListener](../../publisher/datawriterlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener) to implement matching policies similar to those of the PARTITION QoS except the decision can be made based on an application-defined policy.
