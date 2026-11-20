<a id="historyattributes"></a>

# HistoryAttributes

### class HistoryAttributes

Class [HistoryAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1HistoryAttributes), to specify the attributes of a [WriterHistory](../history/WriterHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory) or a [ReaderHistory](../history/ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). This class is only intended to be used with the RTPS API. The Publisher-Subscriber API has other fields to define this values (HistoryQosPolicy and ResourceLimitsQosPolicy). 

### Public Functions

### inline HistoryAttributes()

Default constructor. 

### inline HistoryAttributes(MemoryManagementPolicy_t memoryPolicy, std::uint32_t payload, std::int32_t initial, std::int32_t maxRes)

Constructor 

* **Parameters:**
  * **memoryPolicy** – Set whether memory can be dynamically reallocated or not 
  * **payload** – Maximum payload size. It is used when memory management policy is PREALLOCATED_MEMORY_MODE or PREALLOCATED_WITH_REALLOC_MEMORY_MODE. 
  * **initial** – Initial reserved caches. It is used when memory management policy is PREALLOCATED_MEMORY_MODE or PREALLOCATED_WITH_REALLOC_MEMORY_MODE. 
  * **maxRes** – Maximum reserved caches. 

### inline HistoryAttributes(MemoryManagementPolicy_t memoryPolicy, std::uint32_t payload, std::int32_t initial, std::int32_t maxRes, std::int32_t extra)

Constructor 

* **Parameters:**
  * **memoryPolicy** – Set whether memory can be dynamically reallocated or not 
  * **payload** – Maximum payload size. It is used when memory management policy is PREALLOCATED_MEMORY_MODE or PREALLOCATED_WITH_REALLOC_MEMORY_MODE. 
  * **initial** – Initial reserved caches. It is used when memory management policy is PREALLOCATED_MEMORY_MODE or PREALLOCATED_WITH_REALLOC_MEMORY_MODE. 
  * **maxRes** – Maximum reserved caches. 
  * **extra** – Extra reserved caches. 

### Public Members

### MemoryManagementPolicy_t memoryPolicy

Memory management policy. 

### std::uint32_t payloadMaxSize

Maximum payload size of the history, default value 500. 

### std::int32_t initialReservedCaches

Number of the initial Reserved Caches, default value 500. 

### std::int32_t maximumReservedCaches

Maximum number of reserved caches. Default value is 0 that indicates to keep reserving until something breaks. 

### std::int32_t extraReservedCaches

Number of extra caches that can be reserved for other purposes than the history. For example, on a full history, the writer could give as many as these to be used by the application but they will not be able to be inserted in the history unless some cache from the history is released.

Default value is 1.
