<a id="python-api-pim-datawriterqos"></a>

<a id="datawriterqos"></a>

# DataWriterQos

### *class* fastdds.DataWriterQos

Class DataWriterQos, containing all the possible Qos that can be set for a determined DataWriter.
Although these values can be  and are transmitted
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
  DeadlineQosPolicy reference

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
  DestinationOrderQosPolicy reference

<br/>

*Overload 2:*

Getter for DestinationOrderQosPolicy

* **Return type:**
  [`DestinationOrderQosPolicy`](../core/policy/destinationorderqospolicy.md#fastdds.DestinationOrderQosPolicy)
* **Returns:**
  DestinationOrderQosPolicy reference

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
  DurabilityQosPolicy reference

<br/>

*Overload 2:*

Getter for DurabilityQosPolicy

* **Return type:**
  [`DurabilityQosPolicy`](../core/policy/durabilityqospolicy.md#fastdds.DurabilityQosPolicy)
* **Returns:**
  DurabilityQosPolicy reference

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
  DurabilityServiceQosPolicy reference

<br/>

*Overload 2:*

Getter for DurabilityServiceQosPolicy

* **Return type:**
  [`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)
* **Returns:**
  DurabilityServiceQosPolicy reference

<br/>

*Overload 3:*

Setter for DurabilityServiceQosPolicy

* **Parameters:**
  **durability_service** ([`DurabilityServiceQosPolicy`](../core/policy/durabilityserviceqospolicy.md#fastdds.DurabilityServiceQosPolicy)) – new value for the DurabilityServiceQosPolicy

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
:rtype: [`RTPSEndpointQos`](../core/policy/rtpsendpointqos.md#fastdds.RTPSEndpointQos)
:return: RTPSEndpointQos reference

<br/>

*Overload 3:*

Setter for RTPSEndpointQos

* **Parameters:**
  **endpoint** ([`RTPSEndpointQos`](../core/policy/rtpsendpointqos.md#fastdds.RTPSEndpointQos)) – new value for the RTPSEndpointQos

#### get_writerqos(pqos, tqos)

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
  HistoryQosPolicy reference

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
  LatencyBudgetQosPolicy reference

<br/>

*Overload 2:*

Getter for LatencyBudgetQosPolicy

* **Return type:**
  [`LatencyBudgetQosPolicy`](../core/policy/latencybudgetqospolicy.md#fastdds.LatencyBudgetQosPolicy)
* **Returns:**
  LatencyBudgetQosPolicy reference

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
  LifespanQosPolicy reference

<br/>

*Overload 2:*

Getter for LifespanQosPolicy

* **Return type:**
  [`LifespanQosPolicy`](../core/policy/lifespanqospolicy.md#fastdds.LifespanQosPolicy)
* **Returns:**
  LifespanQosPolicy reference

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
  LivelinessQosPolicy reference

<br/>

*Overload 2:*

Getter for LivelinessQosPolicy

* **Return type:**
  [`LivelinessQosPolicy`](../core/policy/livelinessqospolicy.md#fastdds.LivelinessQosPolicy)
* **Returns:**
  LivelinessQosPolicy reference

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
  OwnershipQosPolicy reference

<br/>

*Overload 2:*

Getter for OwnershipQosPolicy

* **Return type:**
  [`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)
* **Returns:**
  OwnershipQosPolicy reference

<br/>

*Overload 3:*

Setter for OwnershipQosPolicy

* **Parameters:**
  **ownership** ([`OwnershipQosPolicy`](../core/policy/ownershipqospolicy.md#fastdds.OwnershipQosPolicy)) – new value for the OwnershipQosPolicy

#### ownership_strength(\*args)

*Overload 1:*

Getter for OwnershipStrengthQosPolicy

* **Return type:**
  [`OwnershipStrengthQosPolicy`](../core/policy/ownershipstrengthqospolicy.md#fastdds.OwnershipStrengthQosPolicy)
* **Returns:**
  OwnershipStrengthQosPolicy reference

<br/>

*Overload 2:*

Getter for OwnershipStrengthQosPolicy

* **Return type:**
  [`OwnershipStrengthQosPolicy`](../core/policy/ownershipstrengthqospolicy.md#fastdds.OwnershipStrengthQosPolicy)
* **Returns:**
  OwnershipStrengthQosPolicy reference

<br/>

*Overload 3:*

Setter for OwnershipStrengthQosPolicy

* **Parameters:**
  **ownership_strength** ([`OwnershipStrengthQosPolicy`](../core/policy/ownershipstrengthqospolicy.md#fastdds.OwnershipStrengthQosPolicy)) – new value for the OwnershipStrengthQosPolicy

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
  PropertyPolicyQos reference

<br/>

*Overload 3:*

Setter for PropertyPolicyQos

* **Parameters:**
  **properties** (`PropertyPolicyQos`) – new value for the PropertyPolicyQos

#### publish_mode(\*args)

*Overload 1:*

Getter for PublishModeQosPolicy

* **Return type:**
  [`PublishModeQosPolicy`](../core/policy/publishmodeqospolicy.md#fastdds.PublishModeQosPolicy)
* **Returns:**
  PublishModeQosPolicy reference

<br/>

*Overload 2:*

Getter for PublishModeQosPolicy

* **Return type:**
  [`PublishModeQosPolicy`](../core/policy/publishmodeqospolicy.md#fastdds.PublishModeQosPolicy)
* **Returns:**
  PublishModeQosPolicy reference

<br/>

*Overload 3:*

Setter for PublishModeQosPolicy

* **Parameters:**
  **publish_mode** ([`PublishModeQosPolicy`](../core/policy/publishmodeqospolicy.md#fastdds.PublishModeQosPolicy)) – new value for the PublishModeQosPolicy

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
  ReliabilityQosPolicy reference

<br/>

*Overload 3:*

Setter for ReliabilityQosPolicy

* **Parameters:**
  **reliability** ([`ReliabilityQosPolicy`](../core/policy/reliabilityqospolicy.md#fastdds.ReliabilityQosPolicy)) – new value for the ReliabilityQosPolicy

#### reliable_writer_qos(\*args)

*Overload 1:*

Getter for RTPSReliableWriterQos

* **Return type:**
  [`RTPSReliableWriterQos`](rtpsreliablewriterqos.md#fastdds.RTPSReliableWriterQos)
* **Returns:**
  RTPSReliableWriterQos reference

<br/>

*Overload 2:*

Getter for RTPSReliableWriterQos

* **Return type:**
  [`RTPSReliableWriterQos`](rtpsreliablewriterqos.md#fastdds.RTPSReliableWriterQos)
* **Returns:**
  RTPSReliableWriterQos reference

<br/>

*Overload 3:*

Setter for RTPSReliableWriterQos

* **Parameters:**
  **reliable_writer_qos** ([`RTPSReliableWriterQos`](rtpsreliablewriterqos.md#fastdds.RTPSReliableWriterQos)) – new value for the RTPSReliableWriterQos

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
  ResourceLimitsQosPolicy reference

<br/>

*Overload 3:*

Setter for ResourceLimitsQosPolicy

* **Parameters:**
  **resource_limits** ([`ResourceLimitsQosPolicy`](../core/policy/resourcelimitsqospolicy.md#fastdds.ResourceLimitsQosPolicy)) – new value for the ResourceLimitsQosPolicy

#### *property* thisown

The membership flag

#### transport_priority(\*args)

*Overload 1:*

Getter for TransportPriorityQosPolicy

* **Return type:**
  [`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)
* **Returns:**
  TransportPriorityQosPolicy reference

<br/>

*Overload 2:*

Getter for TransportPriorityQosPolicy

* **Return type:**
  [`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)
* **Returns:**
  TransportPriorityQosPolicy reference

<br/>

*Overload 3:*

Setter for TransportPriorityQosPolicy

* **Parameters:**
  **transport_priority** ([`TransportPriorityQosPolicy`](../core/policy/transportpriorityqospolicy.md#fastdds.TransportPriorityQosPolicy)) – new value for the TransportPriorityQosPolicy

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
  UserDataQosPolicy reference

<br/>

*Overload 3:*

Setter for UserDataQosPolicy

* **Parameters:**
  **user_data** ([`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)) – new value for the UserDataQosPolicy

#### writer_data_lifecycle(\*args)

*Overload 1:*

Getter for WriterDataLifecycleQosPolicy

* **Return type:**
  [`WriterDataLifecycleQosPolicy`](../core/policy/writerdatalifecycleqospolicy.md#fastdds.WriterDataLifecycleQosPolicy)
* **Returns:**
  WriterDataLifecycleQosPolicy reference

<br/>

*Overload 2:*

Getter for WriterDataLifecycleQosPolicy

* **Return type:**
  [`WriterDataLifecycleQosPolicy`](../core/policy/writerdatalifecycleqospolicy.md#fastdds.WriterDataLifecycleQosPolicy)
* **Returns:**
  WriterDataLifecycleQosPolicy reference

<br/>

*Overload 3:*

Setter for WriterDataLifecycleQosPolicy

* **Parameters:**
  **writer_data_lifecycle** ([`WriterDataLifecycleQosPolicy`](../core/policy/writerdatalifecycleqospolicy.md#fastdds.WriterDataLifecycleQosPolicy)) – new value for the WriterDataLifecycleQosPolicy

#### writer_resource_limits(\*args)

*Overload 1:*

Getter for WriterResourceLimitsQos

* **Return type:**
  [`WriterResourceLimitsQos`](../core/policy/writerresourcelimitsqos.md#fastdds.WriterResourceLimitsQos)
* **Returns:**
  WriterResourceLimitsQos reference

<br/>

*Overload 2:*

Getter for WriterResourceLimitsQos

* **Return type:**
  [`WriterResourceLimitsQos`](../core/policy/writerresourcelimitsqos.md#fastdds.WriterResourceLimitsQos)
* **Returns:**
  WriterResourceLimitsQos reference

<br/>

*Overload 3:*

Setter for WriterResourceLimitsQos

* **Parameters:**
  **writer_resource_limits** ([`WriterResourceLimitsQos`](../core/policy/writerresourcelimitsqos.md#fastdds.WriterResourceLimitsQos)) – new value for the WriterResourceLimitsQos

### fastdds.DATAWRITER_QOS_DEFAULT *= <fastdds.DataWriterQos>*

Class DataWriterQos, containing all the possible Qos that can be set for a determined DataWriter.
Although these values can be  and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.

### fastdds.DATAWRITER_QOS_USE_TOPIC_QOS *= <fastdds.DataWriterQos>*

Class DataWriterQos, containing all the possible Qos that can be set for a determined DataWriter.
Although these values can be  and are transmitted
during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
Please consult each of them to check for implementation details and default values.
