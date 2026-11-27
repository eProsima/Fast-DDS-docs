<a id="api-pim-transportpriorityqospolicy"></a>

<a id="transportpriorityqospolicy"></a>

# TransportPriorityQosPolicy

### class TransportPriorityQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

This policy is a hint to the infrastructure as to how to set the priority of the underlying transport used to send the data.

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline TransportPriorityQosPolicy()

Constructor. 

### virtual ~TransportPriorityQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### std::int32_t value

Priority

By default, 0.
