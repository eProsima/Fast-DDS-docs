<a id="api-pim-incompatibleqosstatus"></a>

<a id="incompatibleqosstatus"></a>

# IncompatibleQosStatus

### struct IncompatibleQosStatus

A struct storing the requested incompatible QoS status. 

### Public Members

### std::uint32_t total_count = 0

Total cumulative number of times the concerned writer discovered a reader for the same topic. 

The requested QoS is incompatible with the one offered by the writer 

### std::uint32_t total_count_change = 0

The change in total_count since the last time the listener was called or the status was read. 

### [QosPolicyId_t](../policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) last_policy_id = INVALID_QOS_POLICY_ID

The id of the policy that was found to be incompatible the last time an incompatibility is detected. 

### [QosPolicyCountSeq](qospolicycountseq.md#_CPPv4N8eprosima7fastdds3dds17QosPolicyCountSeqE) policies

A list of [QosPolicyCount](qospolicycount.md#structeprosima_1_1fastdds_1_1dds_1_1QosPolicyCount).
