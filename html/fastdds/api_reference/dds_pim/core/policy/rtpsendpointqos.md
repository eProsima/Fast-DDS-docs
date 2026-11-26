<a id="api-pim-rtpsendpointqos"></a>

<a id="rtpsendpointqos"></a>

# RTPSEndpointQos

### class RTPSEndpointQos

### Public Members

### rtps::[LocatorList](../../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) unicast_locator_list

Unicast locator list. 

### rtps::[LocatorList](../../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) multicast_locator_list

Multicast locator list. 

### rtps::[LocatorList](../../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) remote_locator_list

Remote locator list. 

### fastdds::rtps::[ExternalLocators](../../../rtps/attributes/ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) external_unicast_locators

The collection of external locators to use for communication. 

### bool ignore_non_matching_locators = false

Whether locators that don’t match with the announced locators should be kept. 

### std::int16_t user_defined_id = -1

User Defined ID, used for StaticEndpointDiscovery.

By default, -1.

### std::int16_t entity_id = -1

[Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity)

ID, if the user wants to specify the EntityID of the endpoint.

By default, -1.

### fastdds::rtps::MemoryManagementPolicy_t history_memory_policy = fastdds::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE

Underlying History memory policy.

By default, PREALLOCATED_WITH_REALLOC_MEMORY_MODE.
