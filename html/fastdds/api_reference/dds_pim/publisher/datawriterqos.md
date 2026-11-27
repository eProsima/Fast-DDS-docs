<a id="api-pim-datawriterqos"></a>

<a id="datawriterqos"></a>

# DataWriterQos

### class DataWriterQos

Class [DataWriterQos](#classeprosima_1_1fastdds_1_1dds_1_1DataWriterQos), containing all the possible Qos that can be set for a determined [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). Although these values can be and are transmitted during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library. Please consult each of them to check for implementation details and default values. 

Subclassed by [eprosima::fastdds::statistics::dds::DataWriterQos](../../statistics/datawriter_qos.md#classeprosima_1_1fastdds_1_1statistics_1_1dds_1_1DataWriterQos)

### Public Functions

### DataWriterQos()

Constructor. 

### ~DataWriterQos() = default

Destructor. 

### inline [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability()

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy) reference 

### inline const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability() const

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy) reference 

### inline void durability(const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability)

Setter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Parameters:**
  **durability** – new value for the [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

### inline [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service()

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy) reference 

### inline const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service() const

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy) reference 

### inline void durability_service(const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service)

Setter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Parameters:**
  **durability_service** – new value for the [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

### inline [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline()

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy) reference 

### inline const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline() const

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy) reference 

### inline void deadline(const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline)

Setter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Parameters:**
  **deadline** – new value for the [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

### inline [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget()

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy) reference 

### inline const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget() const

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy) reference 

### inline void latency_budget(const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget)

Setter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Parameters:**
  **latency_budget** – new value for the [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

### inline [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness()

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy) reference 

### inline const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness() const

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy) reference 

### inline void liveliness(const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness)

Setter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Parameters:**
  **liveliness** – new value for the [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

### inline [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability()

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy) reference 

### inline const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability() const

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy) reference 

### inline void reliability(const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability)

Setter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Parameters:**
  **reliability** – new value for the [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

### inline [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order()

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy) reference 

### inline const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order() const

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy) reference 

### inline void destination_order(const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order)

Setter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Parameters:**
  **destination_order** – new value for the [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

### inline [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history()

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) reference 

### inline const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history() const

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) reference 

### inline void history(const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history)

Setter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Parameters:**
  **history** – new value for the [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

### inline [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits()

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy) reference 

### inline const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits() const

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy) reference 

### inline void resource_limits(const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits)

Setter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Parameters:**
  **resource_limits** – new value for the [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

### inline [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority()

Getter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Returns:**
  [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy) reference 

### inline const [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority() const

Getter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Returns:**
  [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy) reference 

### inline void transport_priority(const [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE) &transport_priority)

Setter for [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

* **Parameters:**
  **transport_priority** – new value for the [TransportPriorityQosPolicy](../core/policy/transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy)

### inline [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan()

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy) reference 

### inline const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan() const

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy) reference 

### inline void lifespan(const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan)

Setter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Parameters:**
  **lifespan** – new value for the [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

### inline [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data()

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) reference 

### inline const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data() const

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) reference 

### inline void user_data(const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data)

Setter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Parameters:**
  **user_data** – new value for the [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

### inline [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership()

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy) reference 

### inline const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership() const

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy) reference 

### inline void ownership(const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership)

Setter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Parameters:**
  **ownership** – new value for the [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

### inline [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicyE) &ownership_strength()

Getter for [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy)

* **Returns:**
  [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy) reference 

### inline const [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicyE) &ownership_strength() const

Getter for [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy)

* **Returns:**
  [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy) reference 

### inline void ownership_strength(const [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicyE) &ownership_strength)

Setter for [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy)

* **Parameters:**
  **ownership_strength** – new value for the [OwnershipStrengthQosPolicy](../core/policy/ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy)

### inline [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicyE) &writer_data_lifecycle()

Getter for [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy)

* **Returns:**
  [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy) reference 

### inline const [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicyE) &writer_data_lifecycle() const

Getter for [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy)

* **Returns:**
  [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy) reference 

### inline void writer_data_lifecycle(const [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicyE) &writer_data_lifecycle)

Setter for [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy)

* **Parameters:**
  **writer_data_lifecycle** – new value for the [WriterDataLifecycleQosPolicy](../core/policy/writerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1WriterDataLifecycleQosPolicy)

### inline [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE) &publish_mode()

Getter for [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy)

* **Returns:**
  [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy) reference 

### inline const [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE) &publish_mode() const

Getter for [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy)

* **Returns:**
  [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy) reference 

### inline void publish_mode(const [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE) &publish_mode)

Setter for [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy)

* **Parameters:**
  **publish_mode** – new value for the [PublishModeQosPolicy](../core/policy/publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy)

### inline [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation()

Getter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Returns:**
  [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy) reference 

### inline const [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation() const

Getter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Returns:**
  [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy) reference 

### inline void representation(const [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &representation)

Setter for [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

* **Parameters:**
  **representation** – new value for the [DataRepresentationQosPolicy](../core/policy/datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy)

### inline [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties()

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos reference 

### inline const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties() const

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos reference 

### inline void properties(const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties)

Setter for PropertyPolicyQos

* **Parameters:**
  **properties** – new value for the PropertyPolicyQos 

### inline [RTPSReliableWriterQos](rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE) &reliable_writer_qos()

Getter for [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos)

* **Returns:**
  [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos) reference 

### inline const [RTPSReliableWriterQos](rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE) &reliable_writer_qos() const

Getter for [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos)

* **Returns:**
  [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos) reference 

### inline void reliable_writer_qos(const [RTPSReliableWriterQos](rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE) &reliable_writer_qos)

Setter for [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos)

* **Parameters:**
  **reliable_writer_qos** – new value for the [RTPSReliableWriterQos](rtpsreliablewriterqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableWriterQos)

### inline [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &endpoint()

Getter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

* **Returns:**
  [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos) reference 

### inline const [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &endpoint() const

Getter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos) 

* **Returns:**
  [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos) reference 

### inline void endpoint(const [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &endpoint)

Setter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

* **Parameters:**
  **endpoint** – new value for the [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

### inline [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE) &writer_resource_limits()

Getter for [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos)

* **Returns:**
  [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos) reference 

### inline const [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE) &writer_resource_limits() const

Getter for [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos)

* **Returns:**
  [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos) reference 

### inline void writer_resource_limits(const [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE) &writer_resource_limits)

Setter for [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos)

* **Parameters:**
  **writer_resource_limits** – new value for the [WriterResourceLimitsQos](../core/policy/writerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1WriterResourceLimitsQos)

### inline [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) &data_sharing()

Getter for [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy)

* **Returns:**
  [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy) reference 

### inline const [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) &data_sharing() const

Getter for [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy)

* **Returns:**
  [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy) reference 

### inline void data_sharing(const [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) &data_sharing)

Setter for [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy)

* **Parameters:**
  **data_sharing** – new value for the [DataSharingQosPolicy](../core/policy/datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy)

### const [DataWriterQos](#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT

### const [DataWriterQos](#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) eprosima::fastdds::dds::DATAWRITER_QOS_USE_TOPIC_QOS
