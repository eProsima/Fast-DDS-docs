<a id="serializedpayload-t"></a>

# SerializedPayload_t

### struct SerializedPayload_t

Structure [SerializedPayload_t](#structeprosima_1_1fastdds_1_1rtps_1_1SerializedPayload__t). 

### Public Functions

### inline SerializedPayload_t()

Default constructor. 

### inline explicit SerializedPayload_t(std::uint32_t len)

* **Parameters:**
  **len** – Maximum size of the payload 

### SerializedPayload_t(const [SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_t19SerializedPayload_tERK19SerializedPayload_t) &other) = delete

Copy constructor. 

### [SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &operator=(const [SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &other) = delete

Copy operator. 

### inline SerializedPayload_t([SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_t19SerializedPayload_tERR19SerializedPayload_t) &&other) noexcept

Move constructor. 

### [SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &operator=([SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &&other) noexcept

Move operator. 

### ~SerializedPayload_t()

Destructor It is expected to release the payload if the payload owner is not nullptr before destruction. 

### bool copy(const [SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) \*serData, bool with_limit = true)

Copy another structure (including allocating new space for the data). 

* **Parameters:**
  * **serData** – **[in]** Pointer to the structure to copy 
  * **with_limit** – if true, the function will fail when providing a payload too big 
* **Returns:**
  True if correct 

### bool reserve_fragmented([SerializedPayload_t](#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) \*serData)

Allocate new space for fragmented data. 

* **Parameters:**
  **serData** – **[in]** Pointer to the structure to copy 
* **Returns:**
  True if correct 

### void empty()

Empty the payload. 

* **Pre:**
  payload_owner must be nullptr 

### Public Members

### std::uint16_t encapsulation

Encapsulation of the data as suggested in the RTPS 2.1 specification chapter 10. 

### std::uint32_t length

Actual length of the data. 

### [octet](../Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*data

Pointer to the data. 

### std::uint32_t max_size

Maximum size of the payload. 

### std::uint32_t pos

Position when reading. 

### [IPayloadPool](../../history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE) \*payload_owner = nullptr

Pool that created the payload. 

### bool is_serialized_key = false

Whether the payload contains a serialized key, or the whole data. 

### Public Static Attributes

### static constexpr std::size_t representation_header_size = 4u

Size in bytes of the representation header as specified in the RTPS 2.3 specification chapter 10.
