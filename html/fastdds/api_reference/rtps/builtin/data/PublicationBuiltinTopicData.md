<a id="api-rtps-builtin-data-publicationbuiltintopicdata"></a>

<a id="publicationbuiltintopicdata"></a>

# PublicationBuiltinTopicData

### struct PublicationBuiltinTopicData

Structure [PublicationBuiltinTopicData](#structeprosima_1_1fastdds_1_1rtps_1_1PublicationBuiltinTopicData), contains the information on a discovered publication. 

### Public Members

### BuiltinTopicKey_t key = {{0, 0, 0}}

Builtin topic Key. 

### BuiltinTopicKey_t participant_key = {{0, 0, 0}}

Builtin participant topic Key. 

### fastcdr::string_255 topic_name

Topic name. 

### fastcdr::string_255 type_name

Type name. 

### [TopicKind_t](../../common/Types/TopicKind_t.md#_CPPv4N8eprosima7fastdds4rtps11TopicKind_tE) topic_kind = [TopicKind_t](../../common/Types/TopicKind_t.md#_CPPv4N8eprosima7fastdds4rtps11TopicKind_tE)::NO_KEY

Topic kind. 

### dds::[DurabilityQosPolicy](../../../dds_pim/core/policy/durabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds19DurabilityQosPolicyE) durability

Durability Qos, implemented in the library. 

### dds::[DurabilityServiceQosPolicy](../../../dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicyE) durability_service

Durability Service Qos, NOT implemented in the library. 

### dds::[DeadlineQosPolicy](../../../dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) deadline

Deadline Qos, implemented in the library. 

### dds::[LatencyBudgetQosPolicy](../../../dds_pim/core/policy/latencybudgetqospolicy.md#_CPPv4N8eprosima7fastdds3dds22LatencyBudgetQosPolicyE) latency_budget

Latency Budget Qos, NOT implemented in the library. 

### dds::[LivelinessQosPolicy](../../../dds_pim/core/policy/livelinessqospolicy.md#_CPPv4N8eprosima7fastdds3dds19LivelinessQosPolicyE) liveliness

Liveliness Qos, implemented in the library. 

### dds::[ReliabilityQosPolicy](../../../dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicyE) reliability

Reliability Qos, implemented in the library. 

### dds::[LifespanQosPolicy](../../../dds_pim/core/policy/lifespanqospolicy.md#_CPPv4N8eprosima7fastdds3dds17LifespanQosPolicyE) lifespan

Lifespan Qos, implemented in the library. 

### dds::[UserDataQosPolicy](../../../dds_pim/core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) user_data

User Data Qos, implemented in the library. 

### dds::[OwnershipQosPolicy](../../../dds_pim/core/policy/ownershipqospolicy.md#_CPPv4N8eprosima7fastdds3dds18OwnershipQosPolicyE) ownership

Ownership Qos, implemented in the library. 

### dds::[OwnershipStrengthQosPolicy](../../../dds_pim/core/policy/ownershipstrengthqospolicy.md#_CPPv4N8eprosima7fastdds3dds26OwnershipStrengthQosPolicyE) ownership_strength

Ownership Strength Qos, implemented in the library. 

### dds::[DestinationOrderQosPolicy](../../../dds_pim/core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) destination_order

Destination Order Qos, NOT implemented in the library. 

### dds::[PresentationQosPolicy](../../../dds_pim/core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicyE) presentation

Presentation Qos, NOT implemented in the library. 

### dds::[PartitionQosPolicy](../../../dds_pim/core/policy/partitionqospolicy.md#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicyE) partition

Partition Qos, implemented in the library. 

### dds::[TopicDataQosPolicy](../../../dds_pim/core/policy/topicdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18TopicDataQosPolicyE) topic_data

Topic Data Qos, NOT implemented in the library. 

### dds::[GroupDataQosPolicy](../../../dds_pim/core/policy/groupdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18GroupDataQosPolicyE) group_data

Group Data Qos, implemented in the library. 

### dds::xtypes::[TypeInformationParameter](../../../dds_pim/topic/typeinformation.md#_CPPv4N8eprosima7fastdds3dds6xtypes24TypeInformationParameterE) type_information

Type information. 

### dds::[DataRepresentationQosPolicy](../../../dds_pim/core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) representation

Data representation. 

### dds::[DisablePositiveACKsQosPolicy](../../../dds_pim/core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicyE) disable_positive_acks

Disable positive acks, implemented in the library. 

### dds::[DataSharingQosPolicy](../../../dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) data_sharing

Information for data sharing compatibility check. 

### fastcdr::optional<dds::[HistoryQosPolicy](../../../dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicyE)> history

[History](../../history/History_class.md#classeprosima_1_1fastdds_1_1rtps_1_1History) Qos, kind and depth. 

### fastcdr::optional<dds::[ResourceLimitsQosPolicy](../../../dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE)> resource_limits

Resource limits Qos. 

### fastcdr::optional<dds::[TransportPriorityQosPolicy](../../../dds_pim/core/policy/transportpriorityqospolicy.md#_CPPv4N8eprosima7fastdds3dds26TransportPriorityQosPolicyE)> transport_priority

Transport priority Qos. 

### fastcdr::optional<dds::[WriterDataLifecycleQosPolicy](../../../dds_pim/core/policy/writerdatalifecycleqospolicy.md#_CPPv4N8eprosima7fastdds3dds28WriterDataLifecycleQosPolicyE)> writer_data_lifecycle

Writer data lifecycle Qos. 

### fastcdr::optional<dds::[PublishModeQosPolicy](../../../dds_pim/core/policy/publishmodeqospolicy.md#_CPPv4N8eprosima7fastdds3dds20PublishModeQosPolicyE)> publish_mode

Publish mode qos policy. 

### fastcdr::optional<dds::[RTPSReliableWriterQos](../../../dds_pim/publisher/rtpsreliablewriterqos.md#_CPPv4N8eprosima7fastdds3dds21RTPSReliableWriterQosE)> rtps_reliable_writer

Reliable writer qos policy. 

### fastcdr::optional<dds::[RTPSEndpointQos](../../../dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQosE)> endpoint

[Endpoint](../../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) qos policy. 

### fastcdr::optional<dds::[WriterResourceLimitsQos](../../../dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQosE)> writer_resource_limits

Writer resource limits qos policy. 

### [GUID_t](../../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) guid

GUID. 

### [GUID_t](../../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) persistence_guid

Persistence GUID. 

### [GUID_t](../../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) participant_guid

Participant GUID. 

### [RemoteLocatorList](../../common/RemoteLocators/RemoteLocatorList.md#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) remote_locators

Remote locators. 

### std::uint32_t max_serialized_size = 0

Maximum serialized size of data type. 

### NetworkConfigSet_t loopback_transformation = {}

Network configuration. 

### ParameterPropertyList_t properties

[Property](../../common/Property/Property.md#classeprosima_1_1fastdds_1_1rtps_1_1Property) list.
