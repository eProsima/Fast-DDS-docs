<a id="api-pim-disablepositiveacksqospolicy"></a>

<a id="disablepositiveacksqospolicy"></a>

# DisablePositiveACKsQosPolicy

### class DisablePositiveACKsQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Class [DisablePositiveACKsQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1DisablePositiveACKsQosPolicy) to disable sending of positive ACKs

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline DisablePositiveACKsQosPolicy()

Constructor. 

### virtual ~DisablePositiveACKsQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### bool enabled

True if this QoS is enabled.

By default, false.

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) duration

The duration to keep samples for (not serialized as not needed by reader).

By default,

[dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef).
