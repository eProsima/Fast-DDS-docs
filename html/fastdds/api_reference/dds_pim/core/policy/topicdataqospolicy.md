<a id="api-pim-topicdataqospolicy"></a>

<a id="topicdataqospolicy"></a>

# TopicDataQosPolicy

### class TopicDataQosPolicy : public eprosima::fastdds::dds::[GenericDataQosPolicy](genericdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE)

Class derived from [GenericDataQosPolicy](genericdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy). 

The purpose of this QoS is to allow the application to attach additional information to the created [Topic](../../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) such that when a remote application discovers their existence it can examine the information and use it in an application-defined way.

In combination with the listeners on the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) and [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) as well as by means of operations such as ignore_topic,these QoS can assist an application to extend the provided QoS.
