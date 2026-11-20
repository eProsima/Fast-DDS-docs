<a id="api-pim-latencybudgetqospolicy"></a>

<a id="latencybudgetqospolicy"></a>

# LatencyBudgetQosPolicy

### class LatencyBudgetQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the maximum acceptable delay from the time the data is written until the data is inserted in the receiver’s application-cache and the receiving application is notified of the fact.This policy is a hint to the Service, not something that must be monitored or enforced. The Service is not required to track or alert the user of any violation.

#### NOTE
Mutable Qos Policy 

#### WARNING
This [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) can be defined and is transmitted to the rest of the network but is not implemented in this version.

### Public Functions

### inline LatencyBudgetQosPolicy()

Constructor. 

### virtual ~LatencyBudgetQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) duration

Maximum acceptable delay from the time data is written until it is received.

By default,

[dds::c_TimeZero](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1aee681a69cd352a1a7aa6ec2791a7a484).
