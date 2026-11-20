<a id="api-pim-qospolicycount"></a>

<a id="qospolicycount"></a>

# QosPolicyCount

### struct QosPolicyCount

A struct storing the id of the incompatible QoS Policy and the number of times it fails. 

### Public Functions

### QosPolicyCount() = default

Constructor. 

### inline QosPolicyCount([QosPolicyId_t](../policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) id, std::int32_t c)

Constructor. 

### Public Members

### [QosPolicyId_t](../policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) policy_id = INVALID_QOS_POLICY_ID

The id of the policy. 

### std::uint32_t count = 0

Total number of times that the concerned writer discovered a reader for the same topic. 

The requested QoS is incompatible with the one offered by the writer
