<a id="python-api-pim-topicqos"></a>

<a id="topicqos"></a>

# TopicQos

### *class* fastdds.TopicQos

Class TopicQos, containing all the possible Qos that can be set for a determined Topic.
Although these values can be set and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.

#### deadline(\*args)

*Overload 1:*

Getter for DeadlineQosPolicy

* **Return type:**
  [`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)
* **Returns:**
  DeadlineQos reference

<br/>

*Overload 2:*

Getter for DeadlineQosPolicy

* **Return type:**
  [`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)
* **Returns:**
  DeadlineQos reference

<br/>

*Overload 3:*

Setter for DeadlineQosPolicy

* **Parameters:**
  **deadline** ([`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)) – new value for the DeadlineQosPolicy

#### destination_order(\*args)

*Overload 1:*

Getter for DestinationOrderQosPolicy

* **Return type:**
  [`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)
* **Returns:**
  DestinationOrderQos reference

<br/>

*Overload 2:*

Getter for DestinationOrderQosPolicy

* **Return type:**
  [`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)
* **Returns:**
  DestinationOrderQos reference

<br/>

*Overload 3:*

Setter for DestinationOrderQosPolicy

* **Parameters:**
  **destination_order** ([`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)) – new value for the DestinationOrderQosPolicy

#### durability(\*args)

*Overload 1:*

Getter for DurabilityQosPolicy

* **Return type:**
  [`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)
* **Returns:**
  DurabilityQos reference

<br/>

*Overload 2:*

Getter for DurabilityQosPolicy

* **Return type:**
  [`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)
* **Returns:**
  DurabilityQos reference

<br/>

*Overload 3:*

Setter for DurabilityQosPolicy

* **Parameters:**
  **durability** ([`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)) – new value for the DurabilityQosPolicy

#### durability_service(\*args)

*Overload 1:*

Getter for DurabilityServiceQosPolicy

* **Return type:**
  [`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)
* **Returns:**
  DurabilityServiceQos reference

<br/>

*Overload 2:*

Getter for DurabilityServiceQosPolicy

* **Return type:**
  [`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)
* **Returns:**
  DurabilityServiceQos reference

<br/>

*Overload 3:*

Setter for DurabilityServiceQosPolicy

* **Parameters:**
  **durability_service** ([`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)) – new value for the DurabilityServiceQosPolicy

#### history(\*args)

*Overload 1:*

Getter for HistoryQosPolicy

* **Return type:**
  [`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)
* **Returns:**
  HistoryQos reference

<br/>

*Overload 2:*

Getter for HistoryQosPolicy

* **Return type:**
  [`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)
* **Returns:**
  HistoryQos reference

<br/>

*Overload 3:*

Setter for HistoryQosPolicy

* **Parameters:**
  **history** ([`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)) – new value for the HistoryQosPolicy

#### latency_budget(\*args)

*Overload 1:*

Getter for LatencyBudgetQosPolicy

* **Return type:**
  [`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)
* **Returns:**
  LatencyBudgetQos reference

<br/>

*Overload 2:*

Getter for LatencyBudgetQosPolicy

* **Return type:**
  [`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)
* **Returns:**
  LatencyBudgetQos reference

<br/>

*Overload 3:*

Setter for LatencyBudgetQosPolicy

* **Parameters:**
  **latency_budget** ([`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)) – new value for the LatencyBudgetQosPolicy

#### lifespan(\*args)

*Overload 1:*

Getter for LifespanQosPolicy

* **Return type:**
  [`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)
* **Returns:**
  LifespanQos reference

<br/>

*Overload 2:*

Getter for LifespanQosPolicy

* **Return type:**
  [`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)
* **Returns:**
  LifespanQos reference

<br/>

*Overload 3:*

Setter for LifespanQosPolicy

* **Parameters:**
  **lifespan** ([`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)) – new value for the LifespanQosPolicy

#### liveliness(\*args)

*Overload 1:*

Getter for LivelinessQosPolicy

* **Return type:**
  [`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)
* **Returns:**
  LivelinessQos reference

<br/>

*Overload 2:*

Getter for LivelinessQosPolicy

* **Return type:**
  [`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)
* **Returns:**
  LivelinessQos reference

<br/>

*Overload 3:*

Setter for LivelinessQosPolicy

* **Parameters:**
  **liveliness** ([`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)) – new value for the LivelinessQosPolicy

#### ownership(\*args)

*Overload 1:*

Getter for OwnershipQosPolicy

* **Return type:**
  [`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)
* **Returns:**
  OwnershipQos reference

<br/>

*Overload 2:*

Getter for OwnershipQosPolicy

* **Return type:**
  [`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)
* **Returns:**
  OwnershipQos reference

<br/>

*Overload 3:*

Setter for OwnershipQosPolicy

* **Parameters:**
  **ownership** ([`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)) – new value for the OwnershipQosPolicy

#### reliability(\*args)

*Overload 1:*

Getter for ReliabilityQosPolicy

* **Return type:**
  [`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)
* **Returns:**
  ReliabilityQos reference

<br/>

*Overload 2:*

Getter for ReliabilityQosPolicy

* **Return type:**
  [`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)
* **Returns:**
  ReliabilityQos reference

<br/>

*Overload 3:*

Setter for ReliabilityQosPolicy

* **Parameters:**
  **reliability** ([`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)) – new value for the ReliabilityQosPolicy

#### resource_limits(\*args)

*Overload 1:*

Getter for ResourceLimitsQosPolicy

* **Return type:**
  [`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)
* **Returns:**
  ResourceLimitsQos reference

<br/>

*Overload 2:*

Getter for ResourceLimitsQosPolicy

* **Return type:**
  [`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)
* **Returns:**
  ResourceLimitsQos reference

<br/>

*Overload 3:*

Setter for ResourceLimitsQosPolicy

* **Parameters:**
  **resource_limits** ([`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)) – new value for the ResourceLimitsQosPolicy

#### *property* thisown

The membership flag

#### topic_data(\*args)

*Overload 1:*

Getter for TopicDataQosPolicy

* **Return type:**
  [`TopicDataQosPolicy`](../core/policy/topicdataqospolicy.md#fastdds.TopicDataQosPolicy)
* **Returns:**
  TopicDataQos reference

<br/>

*Overload 2:*

Getter for TopicDataQosPolicy

* **Return type:**
  [`TopicDataQosPolicy`](../core/policy/topicdataqospolicy.md#fastdds.TopicDataQosPolicy)
* **Returns:**
  TopicDataQos reference

<br/>

*Overload 3:*

Setter for TopicDataQosPolicy

* **Parameters:**
  **value** ([`TopicDataQosPolicy`](../core/policy/topicdataqospolicy.md#fastdds.TopicDataQosPolicy)) – new value for the TopicDataQosPolicy

#### transport_priority(\*args)

*Overload 1:*

Getter for TransportPriorityQosPolicy

* **Return type:**
  [`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)
* **Returns:**
  TransportPriorityQos reference

<br/>

*Overload 2:*

Getter for TransportPriorityQosPolicy

* **Return type:**
  [`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)
* **Returns:**
  TransportPriorityQos reference

<br/>

*Overload 3:*

Setter for TransportPriorityQosPolicy

* **Parameters:**
  **transport_priority** ([`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)) – new value for the TransportPriorityQosPolicy

### fastdds.TOPIC_QOS_DEFAULT *= <fastdds.TopicQos>*

Class TopicQos, containing all the possible Qos that can be set for a determined Topic.
Although these values can be set and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.
