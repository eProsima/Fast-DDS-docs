<a id="cachechange-t"></a>

# CacheChange_t

### struct CacheChange_t

Structure [CacheChange_t](#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t), contains information on a specific CacheChange. 

### Public Functions

### inline CacheChange_t()

Default constructor. 

Creates an empty [CacheChange_t](#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 

### inline CacheChange_t(std::uint32_t payload_size, bool is_untyped = false)

Constructor with payload size 

* **Parameters:**
  * **payload_size** – Serialized payload size 
  * **is_untyped** – Flag to mark the change as untyped. 

### inline bool copy(const [CacheChange_t](#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch_ptr)

Copy a different change into this one. 

All the elements are copied, included the data, allocating new memory. 

* **Parameters:**
  **ch_ptr** – **[in]** Pointer to the change. 
* **Returns:**
  True if correct. 

### inline void copy_not_memcpy(const [CacheChange_t](#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch_ptr)

Copy information form a different change into this one. 

All the elements are copied except data. 

* **Parameters:**
  **ch_ptr** – **[in]** Pointer to the change. 

### inline std::uint32_t getFragmentCount() const

Get the number of fragments this change is split into. 

* **Returns:**
  number of fragments. 

### inline std::uint16_t getFragmentSize() const

Get the size of each fragment this change is split into. 

* **Returns:**
  size of fragment (0 means change is not fragmented). 

### inline bool is_fully_assembled()

Checks if all fragments have been received. 

* **Returns:**
  true when change is fully assembled (i.e. no missing fragments). 

### inline bool contains_first_fragment()

Checks if the first fragment is present. 

* **Returns:**
  true when it contains the first fragment. In other case, false. 

### inline void get_missing_fragments([FragmentNumberSet_t](../FragmentNumber/FragmentNumberSet_t.md#_CPPv4N8eprosima7fastdds4rtps19FragmentNumberSet_tE) &frag_sns)

Fills a FragmentNumberSet_t with the list of missing fragments. 

* **Parameters:**
  **frag_sns** – **[out]** FragmentNumberSet_t where result is stored. 

### inline void setFragmentSize(std::uint16_t fragment_size, bool create_fragment_list = false)

Set fragment size for this change. 

**Remark**

Parameter create_fragment_list should only be true when receiving the first fragment of a change. 

* **Parameters:**
  * **fragment_size** – Size of fragments. 
  * **create_fragment_list** – Whether to create missing fragments list or not.

### Public Members

### [ChangeKind_t](ChangeKind_t.md#_CPPv4N8eprosima7fastdds4rtps12ChangeKind_tE) kind = ALIVE

Kind of change, default value ALIVE. 

### [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) writerGUID = {}

[GUID_t](../Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) of the writer that generated this change. 

### [InstanceHandle_t](../InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) instanceHandle = {}

Handle of the data associated with this change. 

### [SequenceNumber_t](../SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) sequenceNumber = {}

SequenceNumber of the change. 

### [SerializedPayload_t](../SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) serializedPayload = {}

Serialized Payload associated with the change. 

### [SerializedPayload_t](../SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) inline_qos = {}

CDR serialization of inlined QoS for this change. 

### bool isRead = false

Indicates if the cache has been read (only used in READERS) 

### [Time_t](../Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) sourceTimestamp = {}

Source TimeStamp. 

### fastdds::rtps::[VendorId_t](../Types/VendorId_t.md#_CPPv4N8eprosima7fastdds4rtps10VendorId_tE) vendor_id = [c_VendorId_Unknown](../Types/const_values.md#_CPPv4N8eprosima7fastdds4rtps18c_VendorId_UnknownE)

Vendor Id of the writer that generated this change. 

### Public Static Functions

### static inline bool calculate_required_fragmented_payload_size(std::uint32_t payload_size, std::uint16_t fragment_size, std::uint32_t &min_required_size)

Calculate the minimum required payload size to store a fragmented change. 

* **Parameters:**
  * **payload_size** – **[in]** Size of the full payload. 
  * **fragment_size** – **[in]** Size of each fragment. 
  * **min_required_size** – **[out]** Minimum required size to store the fragmented payload.
