<a id="remotelocatorsallocationattributes"></a>

# RemoteLocatorsAllocationAttributes

### struct RemoteLocatorsAllocationAttributes

Holds limits for collections of remote locators. 

### Public Members

### std::size_t max_unicast_locators = 4u

Maximum number of unicast locators per remote entity.

This attribute controls the maximum number of unicast locators to keep for each discovered remote entity (be it a participant, reader of writer). It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain as this participant. 

### std::size_t max_multicast_locators = 1u

Maximum number of multicast locators per remote entity.

This attribute controls the maximum number of multicast locators to keep for each discovered remote entity (be it a participant, reader of writer). The default value of 1 is usually enough, as it doesn’t make sense to add more than one multicast locator per entity.
