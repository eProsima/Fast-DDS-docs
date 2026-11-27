<a id="python-api-pim-latencybudgetqospolicy"></a>

<a id="latencybudgetqospolicy"></a>

# LatencyBudgetQosPolicy

### *class* fastdds.LatencyBudgetQosPolicy

Specifies the maximum acceptable delay from the time the data is written until the data is inserted in the receiver’s
application-cache and the receiving application is notified of the fact.This policy is a hint to the Service, not something
that must be monitored or enforced. The Service is not required to track or alert the user of any violation.

Warning: This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.

Notes: Mutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* duration

Maximum acceptable delay from the time data is written until it is received. 
: By default, dds::c_TimeZero.

#### *property* thisown

The membership flag
