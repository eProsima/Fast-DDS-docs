<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-core-policy"></a>

<a id="policy"></a>

# Policy

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to define how each
entity will behave.
To increase the flexibility of the system, the QoS is decomposed in several QoS Policies that can be configured
independently.
However, there may be cases where several policies conflict. Those conflicts are notified to the user through the
ReturnCodes that the QoS setter functions returns.

Each Qos Policy has a unique ID defined in the [`QosPolicyId_t`](../../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) enumerator.
This ID is used in some [Status](../status/status.md#dds-layer-core-status) instances to identify the specific Qos Policy
to which the Status refers.

There are QoS Policies that are immutable, which means that only can be specified either at the entity creation or
before calling the enable operation.

Each DDS Entity has a specific set of QoS Policies that can be a mix of Standard QoS Policies, XTypes Extensions and
eProsima Extensions.

* [Standard QoS Policies](standardQosPolicies.md)
  * [DeadlineQosPolicy](standardQosPolicies.md#deadlineqospolicy)
  * [DestinationOrderQosPolicy](standardQosPolicies.md#destinationorderqospolicy)
  * [DurabilityQosPolicy](standardQosPolicies.md#durabilityqospolicy)
  * [DurabilityServiceQosPolicy](standardQosPolicies.md#durabilityserviceqospolicy)
  * [EntityFactoryQosPolicy](standardQosPolicies.md#entityfactoryqospolicy)
  * [GroupDataQosPolicy](standardQosPolicies.md#groupdataqospolicy)
  * [HistoryQosPolicy](standardQosPolicies.md#historyqospolicy)
  * [LatencyBudgetQosPolicy](standardQosPolicies.md#latencybudgetqospolicy)
  * [LifespanQosPolicy](standardQosPolicies.md#lifespanqospolicy)
  * [LivelinessQosPolicy](standardQosPolicies.md#livelinessqospolicy)
  * [OwnershipQosPolicy](standardQosPolicies.md#ownershipqospolicy)
  * [OwnershipStrengthQosPolicy](standardQosPolicies.md#ownershipstrengthqospolicy)
  * [PartitionQosPolicy](standardQosPolicies.md#partitionqospolicy)
  * [PresentationQosPolicy](standardQosPolicies.md#presentationqospolicy)
  * [ReaderDataLifecycleQosPolicy](standardQosPolicies.md#readerdatalifecycleqospolicy)
  * [ReliabilityQosPolicy](standardQosPolicies.md#reliabilityqospolicy)
  * [ResourceLimitsQosPolicy](standardQosPolicies.md#resourcelimitsqospolicy)
  * [TimeBasedFilterQosPolicy](standardQosPolicies.md#timebasedfilterqospolicy)
  * [TopicDataQosPolicy](standardQosPolicies.md#topicdataqospolicy)
  * [TransportPriorityQosPolicy](standardQosPolicies.md#transportpriorityqospolicy)
  * [UserDataQosPolicy](standardQosPolicies.md#userdataqospolicy)
  * [WriterDataLifecycleQosPolicy](standardQosPolicies.md#writerdatalifecycleqospolicy)
* [eProsima Extensions](eprosimaExtensions.md)
  * [DataSharingQosPolicy](eprosimaExtensions.md#datasharingqospolicy)
  * [DisablePositiveACKsQosPolicy](eprosimaExtensions.md#disablepositiveacksqospolicy)
  * [RTPSReliableReaderQos](eprosimaExtensions.md#rtpsreliablereaderqos)
  * [RTPSReliableWriterQos](eprosimaExtensions.md#rtpsreliablewriterqos)
  * [FlowControllersQos](eprosimaExtensions.md#flowcontrollersqos)
  * [ParticipantResourceLimitsQos](eprosimaExtensions.md#participantresourcelimitsqos)
  * [PropertyPolicyQos](eprosimaExtensions.md#propertypolicyqos)
  * [PublishModeQosPolicy](eprosimaExtensions.md#publishmodeqospolicy)
  * [ReaderResourceLimitsQos](eprosimaExtensions.md#readerresourcelimitsqos)
  * [WriterResourceLimitsQos](eprosimaExtensions.md#writerresourcelimitsqos)
  * [RTPSEndpointQos](eprosimaExtensions.md#rtpsendpointqos)
  * [ThreadSettings](eprosimaExtensions.md#threadsettings)
  * [TransportConfigQos](eprosimaExtensions.md#transportconfigqos)
  * [WireProtocolConfigQos](eprosimaExtensions.md#wireprotocolconfigqos)
* [XTypes Extensions](xtypesExtensions.md)
  * [DataRepresentationQosPolicy](xtypesExtensions.md#datarepresentationqospolicy)
  * [TypeConsistencyEnforcementQosPolicy](xtypesExtensions.md#typeconsistencyenforcementqospolicy)
