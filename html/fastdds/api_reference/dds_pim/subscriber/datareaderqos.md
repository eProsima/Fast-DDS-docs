<a id="api-pim-datareaderqos"></a>

<a id="datareaderqos"></a>

# DataReaderQos

### class DataReaderQos

Class [DataReaderQos](#classeprosima_1_1fastdds_1_1dds_1_1DataReaderQos), containing all the possible Qos that can be set for a determined [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). Although these values can be set and are transmitted during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library. Please consult each of them to check for implementation details and default values. 

Subclassed by [eprosima::fastdds::statistics::dds::DataReaderQos](../../statistics/datareader_qos.md#classeprosima_1_1fastdds_1_1statistics_1_1dds_1_1DataReaderQos), [eprosima::fastdds::statistics::dds::MonitorServiceDataReaderQos](../../statistics/datareader_qos.md#classeprosima_1_1fastdds_1_1statistics_1_1dds_1_1MonitorServiceDataReaderQos)

### Public Functions

### inline DataReaderQos()

Constructor. 

### inline [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability()

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy) reference 

### inline const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &durability() const

Getter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Returns:**
  [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy) const reference 

### inline void durability(const [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) &new_value)

Setter for [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

* **Parameters:**
  **new_value** – new value for the [DurabilityQosPolicy](../core/policy/durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy)

### inline [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline()

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy) reference 

### inline const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &deadline() const

Getter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Returns:**
  [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy) const reference 

### inline void deadline(const [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) &new_value)

Setter for [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

* **Parameters:**
  **new_value** – new value for the [DeadlineQosPolicy](../core/policy/deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy)

### inline [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget()

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy) reference 

### inline const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &latency_budget() const

Getter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Returns:**
  [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy) const reference 

### inline void latency_budget(const [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) &new_value)

Setter for [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

* **Parameters:**
  **new_value** – new value for the [LatencyBudgetQosPolicy](../core/policy/latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy)

### inline [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness()

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy) reference 

### inline const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &liveliness() const

Getter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Returns:**
  [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy) const reference 

### inline void liveliness(const [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) &new_value)

Setter for [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

* **Parameters:**
  **new_value** – new value for the [LivelinessQosPolicy](../core/policy/livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

### inline [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability()

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy) reference 

### inline const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &reliability() const

Getter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Returns:**
  [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy) const reference 

### inline void reliability(const [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) &new_value)

Setter for [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

* **Parameters:**
  **new_value** – new value for the [ReliabilityQosPolicy](../core/policy/reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy)

### inline [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order()

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy) reference 

### inline const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &destination_order() const

Getter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Returns:**
  [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy) const reference 

### inline void destination_order(const [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) &new_value)

Setter for [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

* **Parameters:**
  **new_value** – new value for the [DestinationOrderQosPolicy](../core/policy/destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy)

### inline [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history()

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) reference 

### inline const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &history() const

Getter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Returns:**
  [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) const reference 

### inline void history(const [HistoryQosPolicy](../core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE) &new_value)

Setter for [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

* **Parameters:**
  **new_value** – new value for the [HistoryQosPolicy](../core/policy/historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy)

### inline [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits()

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy) reference 

### inline const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &resource_limits() const

Getter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Returns:**
  [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy) const reference 

### inline void resource_limits(const [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE) &new_value)

Setter for [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

* **Parameters:**
  **new_value** – new value for the [ResourceLimitsQosPolicy](../core/policy/resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy)

### inline [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data()

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) reference 

### inline const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data() const

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) const reference 

### inline void user_data(const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &new_value)

Setter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Parameters:**
  **new_value** – new value for the [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

### inline [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership()

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy) reference 

### inline const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &ownership() const

Getter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Returns:**
  [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy) const reference 

### inline void ownership(const [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) &new_value)

Setter for [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

* **Parameters:**
  **new_value** – new value for the [OwnershipQosPolicy](../core/policy/ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy)

### inline [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicyE) &time_based_filter()

Getter for [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy)

* **Returns:**
  [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy) reference 

### inline const [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicyE) &time_based_filter() const

Getter for [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy)

* **Returns:**
  [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy) const reference 

### inline void time_based_filter(const [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicyE) &new_value)

Setter for [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy)

* **Parameters:**
  **new_value** – new value for the [TimeBasedFilterQosPolicy](../core/policy/timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy)

### inline [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicyE) &reader_data_lifecycle()

Getter for [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy)

* **Returns:**
  [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy) reference 

### inline const [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicyE) &reader_data_lifecycle() const

Getter for [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy)

* **Returns:**
  [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy) const reference 

### inline void reader_data_lifecycle(const [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28ReaderDataLifecycleQosPolicyE) &new_value)

Setter for [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy)

* **Parameters:**
  **new_value** – new value for the [ReaderDataLifecycleQosPolicy](../core/policy/readerdatalifecycleqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderDataLifecycleQosPolicy)

### inline [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan()

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy) reference 

### inline const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &lifespan() const

Getter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Returns:**
  [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy) const reference 

### inline void lifespan(const [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) &new_value)

Setter for [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

* **Parameters:**
  **new_value** – new value for the [LifespanQosPolicy](../core/policy/lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy)

### inline [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service()

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy) reference 

### inline const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &durability_service() const

Getter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Returns:**
  [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy) const reference 

### inline void durability_service(const [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) &new_value)

Setter for [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

* **Parameters:**
  **new_value** – new value for the [DurabilityServiceQosPolicy](../core/policy/durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy)

### inline [RTPSReliableReaderQos](rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQosE) &reliable_reader_qos()

Getter for [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos)

* **Returns:**
  [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos) reference 

### inline const [RTPSReliableReaderQos](rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQosE) &reliable_reader_qos() const

Getter for [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos)

* **Returns:**
  [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos) const reference 

### inline void reliable_reader_qos(const [RTPSReliableReaderQos](rtpsreliablereaderqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableReaderQosE) &new_value)

Setter for [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos)

* **Parameters:**
  **new_value** – new value for the [RTPSReliableReaderQos](rtpsreliablereaderqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSReliableReaderQos)

### inline [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicyE) &type_consistency()

Getter for [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy)

* **Returns:**
  [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy) reference 

### inline const [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicyE) &type_consistency() const

Getter for [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy)

* **Returns:**
  [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy) const reference 

### inline void type_consistency(const [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicyE) &new_value)

Setter for [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy)

* **Parameters:**
  **new_value** – new value for the [TypeConsistencyEnforcementQosPolicy](../core/policy/typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy)

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

### inline bool expects_inline_qos() const

Getter for expects_inline_qos

* **Returns:**
  expects_inline_qos 

### inline void expects_inline_qos(bool new_value)

Setter for expects_inline_qos

* **Parameters:**
  **new_value** – new value for the expects_inline_qos 

### inline [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties()

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos reference 

### inline const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties() const

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos const reference 

### inline void properties(const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &new_value)

Setter for PropertyPolicyQos

* **Parameters:**
  **new_value** – new value for the PropertyPolicyQos 

### inline [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &endpoint()

Getter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

* **Returns:**
  [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos) reference 

### inline const [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &endpoint() const

Getter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

* **Returns:**
  [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos) const reference 

### inline void endpoint(const [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE) &new_value)

Setter for [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

* **Parameters:**
  **new_value** – new value for the [RTPSEndpointQos](../core/policy/rtpsendpointqos.md#classeprosima_1_1fastdds_1_1dds_1_1RTPSEndpointQos)

### inline [ReaderResourceLimitsQos](readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQosE) &reader_resource_limits()

Getter for [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos)

* **Returns:**
  [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos) reference 

### inline const [ReaderResourceLimitsQos](readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQosE) &reader_resource_limits() const

Getter for [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos)

* **Returns:**
  [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos) const reference 

### inline void reader_resource_limits(const [ReaderResourceLimitsQos](readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQosE) &new_value)

Setter for [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos)

* **Parameters:**
  **new_value** – new value for the [ReaderResourceLimitsQos](readerresourcelimitsqos.md#classeprosima_1_1fastdds_1_1dds_1_1ReaderResourceLimitsQos)

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

### const [DataReaderQos](#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT

### const [DataReaderQos](#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) eprosima::fastdds::dds::DATAREADER_QOS_USE_TOPIC_QOS
