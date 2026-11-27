<a id="api-pim-reliabilityqospolicy"></a>

<a id="reliabilityqospolicy"></a>

# ReliabilityQosPolicy

### class ReliabilityQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Indicates the reliability of the endpoint.

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline ReliabilityQosPolicy()

Constructor. 

### virtual ~ReliabilityQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [ReliabilityQosPolicyKind](reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE) kind

Defines the reliability kind of the endpoint. 



 By default, BEST_EFFORT_RELIABILITY_QOS for DataReaders and RELIABLE_RELIABILITY_QOS for DataWriters. 

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) max_blocking_time

Defines the maximum period of time certain methods will be blocked. 

Methods affected by this property are:
* [DataWriter::write](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter_1a8e44c93541265fd4a049bf2e72838cc6)
* DataReader::takeNextData
* DataReader::readNextData

  By default, 100 ms.
