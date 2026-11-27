<a id="api-pim-lifespanqospolicy"></a>

<a id="lifespanqospolicy"></a>

# LifespanQosPolicy

### class LifespanQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the maximum duration of validity of the data written by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter).

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline LifespanQosPolicy()

Constructor. 

### virtual ~LifespanQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) duration

Period of validity.

By default,

[dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef).
