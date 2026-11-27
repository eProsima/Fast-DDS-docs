<a id="api-pim-timebasedfilterqospolicy"></a>

<a id="timebasedfilterqospolicy"></a>

# TimeBasedFilterQosPolicy

### class TimeBasedFilterQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Filter that allows a [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to specify that it is interested only in (potentially) a subset of the values of the data. The filter states that the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) does not want to receive more than one value each minimum_separation, regardless of how fast the changes occur. It is inconsistent for a [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to have a minimum_separation longer than its Deadline period.

#### NOTE
Mutable Qos Policy 

#### WARNING
This [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) can be defined and is transmitted to the rest of the network but is not implemented in this version.

### Public Functions

### inline TimeBasedFilterQosPolicy()

Constructor. 

### virtual ~TimeBasedFilterQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) minimum_separation

Minimum interval between samples. By default, [dds::c_TimeZero](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1aee681a69cd352a1a7aa6ec2791a7a484) (the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) is interested in all values)
