<a id="api-pim-topicqos"></a>

<a id="topicqos"></a>

# TopicQos

### class TopicQos

Class [TopicQos](#classeprosima_1_1fastdds_1_1dds_1_1TopicQos), containing all the possible Qos that can be set for a determined [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic). Although these values can be set and are transmitted during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library. Please consult each of them to check for implementation details and default values. 

### Public Functions

### TopicQos()

Constructor. 

### inline const [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18TopicDataQosPolicyE) &topic_data() const

Getter for [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataQosPolicy)

* **Returns:**
  TopicDataQos reference 

### inline [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18TopicDataQosPolicyE) &topic_data()

Getter for [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataQosPolicy)

* **Returns:**
  TopicDataQos reference 

### inline void topic_data(const [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18TopicDataQosPolicyE) &value)

Setter for [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataQosPolicy)

* **Parameters:**
  **value** – new value for the [TopicDataQosPolicy](../core/policy/topicdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataQosPolicy)

### inline const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability() const

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  DurabilityQos reference 

### inline [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability()

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  DurabilityQos reference 

### inline void durability(const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability)

Setter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Parameters:**
  **durability** – new value for the [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

### inline const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service() const

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  DurabilityServiceQos reference 

### inline [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service()

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  DurabilityServiceQos reference 

### inline void durability_service(const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service)

Setter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Parameters:**
  **durability_service** – new value for the [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

### inline const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline() const

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  DeadlineQos reference 

### inline [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline()

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  DeadlineQos reference 

### inline void deadline(const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline)

Setter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Parameters:**
  **deadline** – new value for the [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

### inline const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget() const

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  LatencyBudgetQos reference 

### inline [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget()

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  LatencyBudgetQos reference 

### inline void latency_budget(const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget)

Setter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Parameters:**
  **latency_budget** – new value for the [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

### inline const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness() const

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  LivelinessQos reference 

### inline [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness()

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  LivelinessQos reference 

### inline void liveliness(const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness)

Setter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Parameters:**
  **liveliness** – new value for the [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

### inline const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability() const

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  ReliabilityQos reference 

### inline [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability()

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  ReliabilityQos reference 

### inline void reliability(const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability)

Setter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Parameters:**
  **reliability** – new value for the [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

### inline const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order() const

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  DestinationOrderQos reference 

### inline [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order()

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  DestinationOrderQos reference 

### inline void destination_order(const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order)

Setter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Parameters:**
  **destination_order** – new value for the [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

### inline const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history() const

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  HistoryQos reference 

### inline [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history()

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  HistoryQos reference 

### inline void history(const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history)

Setter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Parameters:**
  **history** – new value for the [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

### inline const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits() const

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  ResourceLimitsQos reference 

### inline [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits()

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  ResourceLimitsQos reference 

### inline void resource_limits(const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits)

Setter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Parameters:**
  **resource_limits** – new value for the [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

### inline const [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority() const

Getter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Returns:**
  TransportPriorityQos reference 

### inline [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority()

Getter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Returns:**
  TransportPriorityQos reference 

### inline void transport_priority(const [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority)

Setter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Parameters:**
  **transport_priority** – new value for the [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

### inline const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan() const

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  LifespanQos reference 

### inline [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan()

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  LifespanQos reference 

### inline void lifespan(const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan)

Setter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Parameters:**
  **lifespan** – new value for the [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

### inline const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership() const

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  OwnershipQos reference 

### inline [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership()

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  OwnershipQos reference 

### inline void ownership(const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership)

Setter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Parameters:**
  **ownership** – new value for the [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

### inline const [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation() const

Getter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Returns:**
  [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy) reference 

### inline [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation()

Getter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Returns:**
  [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy) reference 

### inline void representation(const [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation)

Setter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Parameters:**
  **representation** – new value for the [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

### const [TopicQos](#_CPPv4N8eprosima7fastdds3dds8TopicQosE) eprosima::fastdds::dds::TOPIC_QOS_DEFAULT
