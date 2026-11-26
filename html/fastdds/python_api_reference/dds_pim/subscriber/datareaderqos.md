<a id="python-api-pim-datareaderqos"></a>

<a id="datareaderqos"></a>

# DataReaderQos

### *class* fastdds.DataReaderQos

Class DataReaderQos, containing all the possible Qos that can be set for a determined DataReader.
Although these values can be set and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.

#### data_sharing(\*args)

*Overload 1:*

Getter for DataSharingQosPolicy

* **Return type:**
  [`DataSharingQosPolicy`](../core/policy/datasharingqospolicy.md#fastdds.DataSharingQosPolicy)
* **Returns:**
  DataSharingQosPolicy reference

<br/>

*Overload 2:*

Getter for DataSharingQosPolicy

* **Return type:**
  [`DataSharingQosPolicy`](../core/policy/datasharingqospolicy.md#fastdds.DataSharingQosPolicy)
* **Returns:**
  DataSharingQosPolicy reference

<br/>

*Overload 3:*

Setter for DataSharingQosPolicy

* **Parameters:**
  **data_sharing** ([`DataSharingQosPolicy`](../core/policy/datasharingqospolicy.md#fastdds.DataSharingQosPolicy)) – new value for the DataSharingQosPolicy

#### deadline(\*args)

*Overload 1:*

Getter for DeadlineQosPolicy

* **Return type:**
  [`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)
* **Returns:**
  DeadlineQosPolicy reference

<br/>

*Overload 2:*

Getter for DeadlineQosPolicy

* **Return type:**
  [`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)
* **Returns:**
  DeadlineQosPolicy const reference

<br/>

*Overload 3:*

Setter for DeadlineQosPolicy

* **Parameters:**
  **new_value** ([`DeadlineQosPolicy`](../core/policy/deadlineqospolicy.md#fastdds.DeadlineQosPolicy)) – new value for the DeadlineQosPolicy

#### destination_order(\*args)

*Overload 1:*

Getter for DestinationOrderQosPolicy

* **Return type:**
  [`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)
* **Returns:**
  DestinationOrderQosPolicy reference

<br/>

*Overload 2:*

Getter for DestinationOrderQosPolicy

* **Return type:**
  [`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)
* **Returns:**
  DestinationOrderQosPolicy const reference

<br/>

*Overload 3:*

Setter for DestinationOrderQosPolicy

* **Parameters:**
  **new_value** ([`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)) – new value for the DestinationOrderQosPolicy

#### durability(\*args)

*Overload 1:*

Getter for DurabilityQosPolicy

* **Return type:**
  [`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)
* **Returns:**
  DurabilityQosPolicy reference

<br/>

*Overload 2:*

Getter for DurabilityQosPolicy

* **Return type:**
  [`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)
* **Returns:**
  DurabilityQosPolicy const reference

<br/>

*Overload 3:*

Setter for DurabilityQosPolicy

* **Parameters:**
  **new_value** ([`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)) – new value for the DurabilityQosPolicy

#### durability_service(\*args)

*Overload 1:*

Getter for DurabilityServiceQosPolicy

* **Return type:**
  [`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)
* **Returns:**
  DurabilityServiceQosPolicy reference

<br/>

*Overload 2:*

Getter for DurabilityServiceQosPolicy

* **Return type:**
  [`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)
* **Returns:**
  DurabilityServiceQosPolicy const reference

<br/>

*Overload 3:*

Setter for DurabilityServiceQosPolicy

* **Parameters:**
  **new_value** ([`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)) – new value for the DurabilityServiceQosPolicy

#### endpoint(\*args)

*Overload 1:*

Getter for RTPSEndpointQos

* **Return type:**
  [`RTPSEndpointQos`](../core/policy/rtpsendpointqos.md#fastdds.RTPSEndpointQos)
* **Returns:**
  RTPSEndpointQos reference

<br/>

*Overload 2:*

Getter for RTPSEndpointQos

* **Return type:**
  [`RTPSEndpointQos`](../core/policy/rtpsendpointqos.md#fastdds.RTPSEndpointQos)
* **Returns:**
  RTPSEndpointQos const reference

<br/>

*Overload 3:*

Setter for RTPSEndpointQos

* **Parameters:**
  **new_value** ([`RTPSEndpointQos`](../core/policy/rtpsendpointqos.md#fastdds.RTPSEndpointQos)) – new value for the RTPSEndpointQos

#### expects_inline_qos(\*args)

*Overload 1:*

Getter for expects_inline_qos

* **Return type:**
  boolean
* **Returns:**
  expects_inline_qos

<br/>

*Overload 2:*

Setter for expects_inline_qos

* **Parameters:**
  **new_value** (*boolean*) – new value for the expects_inline_qos

#### get_readerqos(sqos)

#### history(\*args)

*Overload 1:*

Getter for HistoryQosPolicy

* **Return type:**
  [`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)
* **Returns:**
  HistoryQosPolicy reference

<br/>

*Overload 2:*

Getter for HistoryQosPolicy

* **Return type:**
  [`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)
* **Returns:**
  HistoryQosPolicy const reference

<br/>

*Overload 3:*

Setter for HistoryQosPolicy

* **Parameters:**
  **new_value** ([`HistoryQosPolicy`](../core/policy/historyqospolicy.md#fastdds.HistoryQosPolicy)) – new value for the HistoryQosPolicy

#### latency_budget(\*args)

*Overload 1:*

Getter for LatencyBudgetQosPolicy

* **Return type:**
  [`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)
* **Returns:**
  LatencyBudgetQosPolicy reference

<br/>

*Overload 2:*

Getter for LatencyBudgetQosPolicy

* **Return type:**
  [`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)
* **Returns:**
  LatencyBudgetQosPolicy const reference

<br/>

*Overload 3:*

Setter for LatencyBudgetQosPolicy

* **Parameters:**
  **new_value** ([`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)) – new value for the LatencyBudgetQosPolicy

#### lifespan(\*args)

*Overload 1:*

Getter for LifespanQosPolicy

* **Return type:**
  [`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)
* **Returns:**
  LifespanQosPolicy reference

<br/>

*Overload 2:*

Getter for LifespanQosPolicy

* **Return type:**
  [`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)
* **Returns:**
  LifespanQosPolicy const reference

<br/>

*Overload 3:*

Setter for LifespanQosPolicy

* **Parameters:**
  **new_value** ([`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)) – new value for the LifespanQosPolicy

#### liveliness(\*args)

*Overload 1:*

Getter for LivelinessQosPolicy

* **Return type:**
  [`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)
* **Returns:**
  LivelinessQosPolicy reference

<br/>

*Overload 2:*

Getter for LivelinessQosPolicy

* **Return type:**
  [`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)
* **Returns:**
  LivelinessQosPolicy const reference

<br/>

*Overload 3:*

Setter for LivelinessQosPolicy

* **Parameters:**
  **new_value** ([`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)) – new value for the LivelinessQosPolicy

#### ownership(\*args)

*Overload 1:*

Getter for OwnershipQosPolicy

* **Return type:**
  [`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)
* **Returns:**
  OwnershipQosPolicy reference

<br/>

*Overload 2:*

Getter for OwnershipQosPolicy

* **Return type:**
  [`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)
* **Returns:**
  OwnershipQosPolicy const reference

<br/>

*Overload 3:*

Setter for OwnershipQosPolicy

* **Parameters:**
  **new_value** ([`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)) – new value for the OwnershipQosPolicy

#### properties(\*args)

*Overload 1:*

Getter for PropertyPolicyQos

* **Return type:**
  `PropertyPolicyQos`
* **Returns:**
  PropertyPolicyQos reference

<br/>

*Overload 2:*

Getter for PropertyPolicyQos

* **Return type:**
  `PropertyPolicyQos`
* **Returns:**
  PropertyPolicyQos const reference

<br/>

*Overload 3:*

Setter for PropertyPolicyQos

* **Parameters:**
  **new_value** (`PropertyPolicyQos`) – new value for the PropertyPolicyQos

#### reader_data_lifecycle(\*args)

*Overload 1:*

Getter for ReaderDataLifecycleQosPolicy

* **Return type:**
  [`ReaderDataLifecycleQosPolicy`](../core/policy/readerdatalifecycleqospolicy.md#fastdds.ReaderDataLifecycleQosPolicy)
* **Returns:**
  ReaderDataLifecycleQosPolicy reference

<br/>

*Overload 2:*

Getter for ReaderDataLifecycleQosPolicy

* **Return type:**
  [`ReaderDataLifecycleQosPolicy`](../core/policy/readerdatalifecycleqospolicy.md#fastdds.ReaderDataLifecycleQosPolicy)
* **Returns:**
  ReaderDataLifecycleQosPolicy const reference

<br/>

*Overload 3:*

Setter for ReaderDataLifecycleQosPolicy

* **Parameters:**
  **new_value** ([`ReaderDataLifecycleQosPolicy`](../core/policy/readerdatalifecycleqospolicy.md#fastdds.ReaderDataLifecycleQosPolicy)) – new value for the ReaderDataLifecycleQosPolicy

#### reader_resource_limits(\*args)

*Overload 1:*

Getter for ReaderResourceLimitsQos

* **Return type:**
  [`ReaderResourceLimitsQos`](readerresourcelimitsqos.md#fastdds.ReaderResourceLimitsQos)
* **Returns:**
  ReaderResourceLimitsQos reference

<br/>

*Overload 2:*

Getter for ReaderResourceLimitsQos

* **Return type:**
  [`ReaderResourceLimitsQos`](readerresourcelimitsqos.md#fastdds.ReaderResourceLimitsQos)
* **Returns:**
  ReaderResourceLimitsQos const reference

<br/>

*Overload 3:*

Setter for ReaderResourceLimitsQos

* **Parameters:**
  **new_value** ([`ReaderResourceLimitsQos`](readerresourcelimitsqos.md#fastdds.ReaderResourceLimitsQos)) – new value for the ReaderResourceLimitsQos

#### reliability(\*args)

*Overload 1:*

Getter for ReliabilityQosPolicy

* **Return type:**
  [`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)
* **Returns:**
  ReliabilityQosPolicy reference

<br/>

*Overload 2:*

Getter for ReliabilityQosPolicy

* **Return type:**
  [`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)
* **Returns:**
  ReliabilityQosPolicy const reference

<br/>

*Overload 3:*

Setter for ReliabilityQosPolicy

* **Parameters:**
  **new_value** ([`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)) – new value for the ReliabilityQosPolicy

#### reliable_reader_qos(\*args)

*Overload 1:*

Getter for RTPSReliableReaderQos

* **Return type:**
  [`RTPSReliableReaderQos`](rtpsreliablereaderqos.md#fastdds.RTPSReliableReaderQos)
* **Returns:**
  RTPSReliableReaderQos reference

<br/>

*Overload 2:*

Getter for RTPSReliableReaderQos

* **Return type:**
  [`RTPSReliableReaderQos`](rtpsreliablereaderqos.md#fastdds.RTPSReliableReaderQos)
* **Returns:**
  RTPSReliableReaderQos const reference

<br/>

*Overload 3:*

Setter for RTPSReliableReaderQos

* **Parameters:**
  **new_value** ([`RTPSReliableReaderQos`](rtpsreliablereaderqos.md#fastdds.RTPSReliableReaderQos)) – new value for the RTPSReliableReaderQos

#### representation(\*args)

*Overload 1:*

Getter for DataRepresentationQosPolicy

* **Return type:**
  [`DataRepresentationQosPolicy`](../core/policy/datarepresentationqospolicy.md#fastdds.DataRepresentationQosPolicy)
* **Returns:**
  DataRepresentationQosPolicy reference

<br/>

*Overload 2:*

Getter for DataRepresentationQosPolicy

* **Return type:**
  [`DataRepresentationQosPolicy`](../core/policy/datarepresentationqospolicy.md#fastdds.DataRepresentationQosPolicy)
* **Returns:**
  DataRepresentationQosPolicy reference

<br/>

*Overload 3:*

Setter for DataRepresentationQosPolicy

* **Parameters:**
  **representation** ([`DataRepresentationQosPolicy`](../core/policy/datarepresentationqospolicy.md#fastdds.DataRepresentationQosPolicy)) – new value for the DataRepresentationQosPolicy

#### resource_limits(\*args)

*Overload 1:*

Getter for ResourceLimitsQosPolicy

* **Return type:**
  [`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)
* **Returns:**
  ResourceLimitsQosPolicy reference

<br/>

*Overload 2:*

Getter for ResourceLimitsQosPolicy

* **Return type:**
  [`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)
* **Returns:**
  ResourceLimitsQosPolicy const reference

<br/>

*Overload 3:*

Setter for ResourceLimitsQosPolicy

* **Parameters:**
  **new_value** ([`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)) – new value for the ResourceLimitsQosPolicy

#### *property* thisown

The membership flag

#### time_based_filter(\*args)

*Overload 1:*

Getter for TimeBasedFilterQosPolicy

* **Return type:**
  [`TimeBasedFilterQosPolicy`](../core/policy/timebasedfilterqospolicy.md#fastdds.TimeBasedFilterQosPolicy)
* **Returns:**
  TimeBasedFilterQosPolicy reference

<br/>

*Overload 2:*

Getter for TimeBasedFilterQosPolicy

* **Return type:**
  [`TimeBasedFilterQosPolicy`](../core/policy/timebasedfilterqospolicy.md#fastdds.TimeBasedFilterQosPolicy)
* **Returns:**
  TimeBasedFilterQosPolicy const reference

<br/>

*Overload 3:*

Setter for TimeBasedFilterQosPolicy

* **Parameters:**
  **new_value** ([`TimeBasedFilterQosPolicy`](../core/policy/timebasedfilterqospolicy.md#fastdds.TimeBasedFilterQosPolicy)) – new value for the TimeBasedFilterQosPolicy

#### type_consistency(\*args)

*Overload 1:*

Getter for TypeConsistencyEnforcementQosPolicy

* **Return type:**
  [`TypeConsistencyEnforcementQosPolicy`](../core/policy/typeconsistencyenforcementqospolicy.md#fastdds.TypeConsistencyEnforcementQosPolicy)
* **Returns:**
  TypeConsistencyEnforcementQosPolicy reference

<br/>

*Overload 2:*

Getter for TypeConsistencyEnforcementQosPolicy

* **Return type:**
  [`TypeConsistencyEnforcementQosPolicy`](../core/policy/typeconsistencyenforcementqospolicy.md#fastdds.TypeConsistencyEnforcementQosPolicy)
* **Returns:**
  TypeConsistencyEnforcementQosPolicy const reference

<br/>

*Overload 3:*

Setter for TypeConsistencyEnforcementQosPolicy

* **Parameters:**
  **new_value** ([`TypeConsistencyEnforcementQosPolicy`](../core/policy/typeconsistencyenforcementqospolicy.md#fastdds.TypeConsistencyEnforcementQosPolicy)) – new value for the TypeConsistencyEnforcementQosPolicy

#### user_data(\*args)

*Overload 1:*

Getter for UserDataQosPolicy

* **Return type:**
  [`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)
* **Returns:**
  UserDataQosPolicy reference

<br/>

*Overload 2:*

Getter for UserDataQosPolicy

* **Return type:**
  [`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)
* **Returns:**
  UserDataQosPolicy const reference

<br/>

*Overload 3:*

Setter for UserDataQosPolicy

* **Parameters:**
  **new_value** ([`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)) – new value for the UserDataQosPolicy

### fastdds.DATAREADER_QOS_DEFAULT *= <fastdds.DataReaderQos>*

Class DataReaderQos, containing all the possible Qos that can be set for a determined DataReader.
Although these values can be set and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.

### fastdds.DATAREADER_QOS_USE_TOPIC_QOS *= <fastdds.DataReaderQos>*

Class DataReaderQos, containing all the possible Qos that can be set for a determined DataReader.
Although these values can be set and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.
