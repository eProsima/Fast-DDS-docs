<a id="python-api-pim-reliabilityqospolicykind"></a>

<a id="reliabilityqospolicykind"></a>

# ReliabilityQosPolicyKind

### fastdds.BEST_EFFORT_RELIABILITY_QOS

Indicates that it is acceptable to not retry propagation of any samples. Presumably new values for the samples
are generated often enough that it is not necessary to re-send or acknowledge any samples

### fastdds.RELIABLE_RELIABILITY_QOS

Specifies the Service will attempt to deliver all samples in its history. Missed samples may be retried.
In steady-state (no modifications communicated via the DataWriter) the middleware guarantees that all samples
in the DataWriter history will eventually be delivered to all the DataReader objects. Outside steady state the
HistoryQosPolicy and ResourceLimitsQosPolicy will determine how samples become part of the history and whether
samples can be discarded from it.
