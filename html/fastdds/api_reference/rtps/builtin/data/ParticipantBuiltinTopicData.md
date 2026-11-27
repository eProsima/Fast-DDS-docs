<a id="api-rtps-builtin-data-participantbuiltintopicdata"></a>

<a id="participantbuiltintopicdata"></a>

# ParticipantBuiltinTopicData

### struct ParticipantBuiltinTopicData

### Public Functions

### ParticipantBuiltinTopicData() = default

Default constructor. 

### ParticipantBuiltinTopicData(const [VendorId_t](../../common/Types/VendorId_t.md#_CPPv4N8eprosima7fastdds4rtps10VendorId_tE) vendor_id, const dds::DomainId_t domain_id, const [RTPSParticipantAllocationAttributes](../../attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributesE) &allocation)

Constructor with allocation attributes. 

### Public Members

### BuiltinTopicKey_t key

Builtin topic Key. 

### dds::[UserDataQosPolicy](../../../dds_pim/core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) user_data

UserData QoS. 

### [GUID_t](../../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) guid

Participant GUID. 

### dds::ParameterPropertyList_t properties

Properties. 

### fastcdr::string_255 participant_name

Participant name. 

### [RemoteLocatorList](../../common/RemoteLocators/RemoteLocatorList.md#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) metatraffic_locators

Metatraffic locators. 

### [RemoteLocatorList](../../common/RemoteLocators/RemoteLocatorList.md#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) default_locators

Default locators. 

### dds::[Duration_t](../../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) lease_duration

Lease Duration. 

### [VendorId_t](../../common/Types/VendorId_t.md#_CPPv4N8eprosima7fastdds4rtps10VendorId_tE) vendor_id

Vendor id. 

### ProductVersion_t product_version

Product version. 

### dds::DomainId_t domain_id

Participant domain id. 

### fastcdr::optional<dds::[WireProtocolConfigQos](../../../dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE)> wire_protocol

Wire Protocol Qos.
