<a id="api-pim-qospolicy"></a>

<a id="qospolicy"></a>

# QosPolicy

### class QosPolicy

Subclassed by [eprosima::fastdds::dds::DataRepresentationQosPolicy](datarepresentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy), [eprosima::fastdds::dds::DataSharingQosPolicy](datasharingqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy), [eprosima::fastdds::dds::DeadlineQosPolicy](deadlineqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DeadlineQosPolicy), [eprosima::fastdds::dds::DestinationOrderQosPolicy](destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy), [eprosima::fastdds::dds::DisablePositiveACKsQosPolicy](disablepositiveacksqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DisablePositiveACKsQosPolicy), [eprosima::fastdds::dds::DurabilityQosPolicy](durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy), [eprosima::fastdds::dds::DurabilityServiceQosPolicy](durabilityserviceqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityServiceQosPolicy), [eprosima::fastdds::dds::GenericDataQosPolicy](genericdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy), [eprosima::fastdds::dds::HistoryQosPolicy](historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy), [eprosima::fastdds::dds::LatencyBudgetQosPolicy](latencybudgetqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LatencyBudgetQosPolicy), [eprosima::fastdds::dds::LifespanQosPolicy](lifespanqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LifespanQosPolicy), [eprosima::fastdds::dds::LivelinessQosPolicy](livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy), [eprosima::fastdds::dds::OwnershipQosPolicy](ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy), [eprosima::fastdds::dds::OwnershipStrengthQosPolicy](ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy), [eprosima::fastdds::dds::PartitionQosPolicy](partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy), [eprosima::fastdds::dds::PresentationQosPolicy](presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy), [eprosima::fastdds::dds::PublishModeQosPolicy](publishmodeqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy), [eprosima::fastdds::dds::ReliabilityQosPolicy](reliabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ReliabilityQosPolicy), [eprosima::fastdds::dds::ResourceLimitsQosPolicy](resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy), [eprosima::fastdds::dds::TimeBasedFilterQosPolicy](timebasedfilterqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TimeBasedFilterQosPolicy), [eprosima::fastdds::dds::TransportConfigQos](transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos), [eprosima::fastdds::dds::TransportPriorityQosPolicy](transportpriorityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TransportPriorityQosPolicy), [eprosima::fastdds::dds::TypeConsistencyEnforcementQosPolicy](typeconsistencyenforcementqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy), eprosima::fastdds::dds::TypeIdV1, eprosima::fastdds::dds::TypeObjectV1, [eprosima::fastdds::dds::WireProtocolConfigQos](wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos), [eprosima::fastdds::dds::xtypes::TypeInformationParameter](../../topic/typeinformation.md#classeprosima_1_1fastdds_1_1dds_1_1xtypes_1_1TypeInformationParameter)

### Public Functions

### inline QosPolicy()

Constructor without parameters. 

### inline explicit QosPolicy(bool send_always)

Constructor. 

* **Parameters:**
  **send_always** – Boolean that set if the Qos need to be sent even if it is not changed 

### QosPolicy(const [QosPolicy](#_CPPv4N8eprosima7fastdds3dds9QosPolicy9QosPolicyERK9QosPolicy) &b) = default

Copy Constructor. 

* **Parameters:**
  **b** – Another instance of [QosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy)

### virtual ~QosPolicy() = default

Destructor. 

### inline virtual bool send_always() const

Whether it should always be sent.

* **Returns:**
  True if it should always be sent. 

### inline virtual void clear() = 0

Clears the [QosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### bool hasChanged

Boolean that indicates if the Qos has been changed with respect to the default Qos.
