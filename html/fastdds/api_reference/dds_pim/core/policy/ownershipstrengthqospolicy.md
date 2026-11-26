<a id="api-pim-ownershipstrengthqospolicy"></a>

<a id="ownershipstrengthqospolicy"></a>

# OwnershipStrengthQosPolicy

### class OwnershipStrengthQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the value of the “strength” used to arbitrate among multiple [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) objects that attempt to modify the same instance of a data-object (identified by [Topic](../../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) + key).This policy only applies if the OWNERSHIP QoS policy is of kind EXCLUSIVE.

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline OwnershipStrengthQosPolicy()

Constructor. 

### virtual ~OwnershipStrengthQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### std::uint32_t value

Strength

By default, 0.
