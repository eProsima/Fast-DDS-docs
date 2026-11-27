<a id="memorymanagementpolicy"></a>

# MemoryManagementPolicy

### enum eprosima::fastdds::rtps::MemoryManagementPolicy

Enum MemoryuManagementPolicy_t, indicated the way memory is managed in terms of dealing with CacheChanges 

*Values:*

### enumerator PREALLOCATED_MEMORY_MODE

Preallocated memory. 

Size set to the data type maximum. Largest memory footprint but smallest allocation count. 

### enumerator PREALLOCATED_WITH_REALLOC_MEMORY_MODE

Default size preallocated, requires reallocation when a bigger message arrives. 

Smaller memory footprint at the cost of an increased allocation count. 

### enumerator DYNAMIC_RESERVE_MEMORY_MODE

### enumerator DYNAMIC_REUSABLE_MEMORY_MODE
