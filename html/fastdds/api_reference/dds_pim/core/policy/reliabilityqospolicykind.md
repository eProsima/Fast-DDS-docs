<a id="api-pim-reliabilityqospolicykind"></a>

<a id="reliabilityqospolicykind"></a>

# ReliabilityQosPolicyKind

### enum eprosima::fastdds::dds::ReliabilityQosPolicyKind

Enum ReliabilityQosPolicyKind, different kinds of reliability for [ReliabilityQosPolicy](reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy). 

*Values:*

### enumerator BEST_EFFORT_RELIABILITY_QOS

Indicates that it is acceptable to not retry propagation of any samples. Presumably new values for the samples are generated often enough that it is not necessary to re-send or acknowledge any samples 

### enumerator RELIABLE_RELIABILITY_QOS

Specifies the Service will attempt to deliver all samples in its history. Missed samples may be retried. In steady-state (no modifications communicated via the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)) the middleware guarantees that all samples in the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) history will eventually be delivered to all the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) objects. Outside steady state the [HistoryQosPolicy](historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) and [ResourceLimitsQosPolicy](resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy) will determine how samples become part of the history and whether samples can be discarded from it.
