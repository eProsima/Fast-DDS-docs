<a id="api-pim-deadlineqospolicy"></a>

<a id="deadlineqospolicy"></a>

# DeadlineQosPolicy

### class DeadlineQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

[DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) expects a new sample updating the value of each instance at least once every deadline period. [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) indicates that the application commits to write a new value (using the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)) for each instance managed by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) at least once every deadline period. 

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline DeadlineQosPolicy()

Constructor. 

### virtual ~DeadlineQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) period

Maximum time expected between samples. It is inconsistent for a [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) to have a DEADLINE period less than its [TimeBasedFilterQosPolicy](timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy)

minimum_separation.

By default, c_TimeInifinite.
