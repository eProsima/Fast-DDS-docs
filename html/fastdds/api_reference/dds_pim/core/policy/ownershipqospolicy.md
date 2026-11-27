<a id="api-pim-ownershipqospolicy"></a>

<a id="ownershipqospolicy"></a>

# OwnershipQosPolicy

### class OwnershipQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies whether it is allowed for multiple DataWriters to write the same instance of the data and if so, how these modifications should be arbitrated

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline OwnershipQosPolicy()

Constructor. 

### virtual ~OwnershipQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [OwnershipQosPolicyKind](ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKindE) kind

OwnershipQosPolicyKind.
