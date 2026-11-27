<a id="python-api-pim-historyqospolicy"></a>

<a id="historyqospolicy"></a>

# HistoryQosPolicy

### *class* fastdds.HistoryQosPolicy

Specifies the behavior of the Service in the case where the value of a sample changes (one or more times) before it
can be successfully communicated to one or more existing subscribers. This QoS policy controls whether the Service
should deliver only the most recent value, attempt to deliver all intermediate values, or do something in between.
On the publishing side this policy controls the samples that should be maintained by the DataWriter on behalf of
existing DataReader entities. The behavior with regards to a DataReaderentities discovered after a sample is written
is controlled by the DURABILITY QoS policy. On the subscribing side it controls the samples that should be maintained
until the application “takes” them from the Service.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* depth

History depth. 
By default, 1. If a value other than 1 is specified, it should
be consistent with the settings of the ResourceLimitsQosPolicy.

Warning: Only takes effect if the kind is KEEP_LAST_HISTORY_QOS.

#### *property* kind

HistoryQosPolicyKind. 
: By default, KEEP_LAST_HISTORY_QOS.

#### *property* thisown

The membership flag
