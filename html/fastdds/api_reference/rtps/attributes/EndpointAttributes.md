<a id="endpointattributes"></a>

# EndpointAttributes

### class EndpointAttributes

Structure [EndpointAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1EndpointAttributes), describing the attributes associated with an RTPS [Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint). 

### Public Functions

### inline std::int16_t getUserDefinedID() const

Get the user defined ID 

* **Returns:**
  User defined ID 

### inline std::int16_t getEntityID() const

Get the entity defined ID 

* **Returns:**
  Entity ID 

### inline void setUserDefinedID(std::int16_t id)

Set the user defined ID 

* **Parameters:**
  **id** – User defined ID to be set 

### inline void setEntityID(std::int16_t id)

Set the entity ID 

* **Parameters:**
  **id** – Entity ID to be set 

### inline void set_data_sharing_configuration(fastdds::dds::[DataSharingQosPolicy](../../dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) cfg)

Set the DataSharing configuration 

* **Parameters:**
  **cfg** – Configuration to be set 

### inline const fastdds::dds::[DataSharingQosPolicy](../../dds_pim/core/policy/datasharingqospolicy.md#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicyE) &data_sharing_configuration() const

Get the DataSharing configuration 

* **Returns:**
  Configuration of data sharing 

### Public Members

### [EndpointKind_t](../common/Types/EndpointKind_t.md#_CPPv4N8eprosima7fastdds4rtps14EndpointKind_tE) endpointKind = [EndpointKind_t](../common/Types/EndpointKind_t.md#_CPPv4N8eprosima7fastdds4rtps14EndpointKind_tE)::WRITER

[Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) kind, default value WRITER. 

### [TopicKind_t](../common/Types/TopicKind_t.md#_CPPv4N8eprosima7fastdds4rtps11TopicKind_tE) topicKind = [TopicKind_t](../common/Types/TopicKind_t.md#_CPPv4N8eprosima7fastdds4rtps11TopicKind_tE)::NO_KEY

Topic kind, default value NO_KEY. 

### [ReliabilityKind_t](../common/Types/ReliabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps17ReliabilityKind_tE) reliabilityKind = [ReliabilityKind_t](../common/Types/ReliabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps17ReliabilityKind_tE)::BEST_EFFORT

Reliability kind, default value BEST_EFFORT. 

### [DurabilityKind_t](../common/Types/DurabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps16DurabilityKind_tE) durabilityKind = [DurabilityKind_t](../common/Types/DurabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps16DurabilityKind_tE)::VOLATILE

Durability kind, default value VOLATILE. 

### [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) persistence_guid

GUID used for persistence. 

### [ExternalLocators](ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) external_unicast_locators

The collection of external locators to use for communication. 

### bool ignore_non_matching_locators = false

Whether locators that don’t match with the announced locators should be kept. 

### LocatorList_t unicastLocatorList

Unicast locator list. 

### LocatorList_t multicastLocatorList

Multicast locator list. 

### LocatorList_t remoteLocatorList

Remote locator list. 

### [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) properties

Properties. 

### fastdds::dds::[OwnershipQosPolicyKind](../../dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKindE) ownershipKind = fastdds::dds::[OwnershipQosPolicyKind](../../dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKindE)::[SHARED_OWNERSHIP_QOS](../../dds_pim/core/policy/ownershipqospolicykind.md#_CPPv4N8eprosima7fastdds3dds22OwnershipQosPolicyKind20SHARED_OWNERSHIP_QOSE)

Ownership.
