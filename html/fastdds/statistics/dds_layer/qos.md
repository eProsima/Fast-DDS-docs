<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics-qos"></a>

<a id="statistics-recommended-qos"></a>

# Statistics recommended QoS

Although the statistics DataWriters can be enabled using any valid QoS profile, the recommended profile is presented
below.
Also, the DataReaders created by the user to receive the data being published by the statistics DataWriters can use any
compatible QoS profile.
However, a recommended DataReader QoS profile is also provided.

<a id="statistics-datawriter-qos"></a>

<a id="statistics-datawriter-recommended-qos"></a>

## Statistics DataWriter recommended QoS

The following table shows the recommended [`DataWriterQos`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) profile for enabling the statistics DataWriters.
This profile enables the `pull mode` [operating mode](../../property_policies/non_consolidated_qos.md#push-mode-qos-policy) on the statistics DataWriters.
This entails that the DataWriters will only send information upon the reception of acknack submessages sent by the
monitoring DataReader.
This QoS profile is always used when the statistics DataWriters are
[auto-enabled](domainParticipant.md#auto-enabling-statistics-datawriters).
The recommended profile can be accessed through the constant [`STATISTICS_DATAWRITER_QOS`](../../api_reference/statistics/datawriter_qos.md#_CPPv4N8eprosima7fastdds10statistics3dds25STATISTICS_DATAWRITER_QOSE).

| Qos Policy                                                                                                                                                          | Value                                                                                                                                                                                         |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`ReliabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE)           | [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)           |
| [`DurabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE)              | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) |
| [`PublishModeQosPolicyKind`](../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKindE)           | [`ASYNCHRONOUS_PUBLISH_MODE`](../../api_reference/dds_pim/core/policy/publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKind25ASYNCHRONOUS_PUBLISH_MODEE)         |
| [`flow_controller_name`](../../api_reference/dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicy20flow_controller_nameE) | [`FASTDDS_STATISTICS_FLOW_CONTROLLER_DEFAULT`](../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps42FASTDDS_STATISTICS_FLOW_CONTROLLER_DEFAULTE)   |
| [`HistoryQosPolicyKind`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE)                       | [`KEEP_LAST_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE)                         |
| [`history_depth`](../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13history_depthE)   | 10                                                                                                                                                                                            |
| [`PropertyPolicyQos`](../../api_reference/dds_pim/core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) name = value                   | `"fastdds.push_mode"` = `"false"`                                                                                                                                                             |

<a id="statistics-datareader-qos"></a>

<a id="statistics-datareader-recommended-qos"></a>

## Statistics DataReader recommended QoS

The following table shows the recommended [`DataReaderQos`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) profile for creating the monitoring DataReaders.
The recommended profile can be accessed through constant [`STATISTICS_DATAREADER_QOS`](../../api_reference/statistics/datareader_qos.md#_CPPv4N8eprosima7fastdds10statistics3dds25STATISTICS_DATAREADER_QOSE).

| Qos Policy                                                                                                                                                        | Value                                                                                                                                                                                                 |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`ReliabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE)         | [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE)                   |
| [`DurabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE)            | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE)         |
| [`HistoryQosPolicyKind`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE)                     | [`KEEP_LAST_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE)                                 |
| [`history_depth`](../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13history_depthE) | 100                                                                                                                                                                                                   |
| [`MemoryManagementPolicy`](../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicyE)                   | [`PREALLOCATED_WITH_REALLOC_MEMORY_MODE`](../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE) |
