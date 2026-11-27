<a id="python-api-pim-rtpsendpointqos"></a>

<a id="rtpsendpointqos"></a>

# RTPSEndpointQos

### *class* fastdds.RTPSEndpointQos

#### clear()

#### *property* entity_id

Entity ID, if the user wants to specify the EntityID of the endpoint. 
By default, -1.

#### *property* external_unicast_locators

The collection of external locators to use for communication.

#### *property* history_memory_policy

Underlying History memory policy. 
By default, PREALLOCATED_WITH_REALLOC_MEMORY_MODE.

#### *property* ignore_non_matching_locators

Whether locators that don’t match with the announced locators should be kept.

#### *property* multicast_locator_list

Multicast locator list

#### *property* remote_locator_list

Remote locator list

#### *property* thisown

The membership flag

#### *property* unicast_locator_list

Unicast locator list

#### *property* user_defined_id

User Defined ID, used for StaticEndpointDiscovery. 
By default, -1.
