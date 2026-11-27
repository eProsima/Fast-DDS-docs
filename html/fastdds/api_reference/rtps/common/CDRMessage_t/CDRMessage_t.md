<a id="cdrmessage-t"></a>

# CDRMessage_t

### struct CDRMessage_t

Structure [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t), contains a serialized message. 

### Public Functions

### inline explicit CDRMessage_t(std::uint32_t size)

Constructor with maximum size 

* **Parameters:**
  **size** – Maximum size 

### inline explicit CDRMessage_t(const [SerializedPayload_t](../SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload)

Constructor to wrap a serialized payload 

* **Parameters:**
  **payload** – Payload to wrap 

### Public Members

### [octet](../Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*buffer

Pointer to the buffer where the data is stored. 

### std::uint32_t pos

Read or write position. 

### std::uint32_t max_size

Max size of the message. 

### std::uint32_t reserved_size

Size allocated on buffer. May be higher than max_size. 

### std::uint32_t length

Current length of the message. 

### [Endianness_t](../Types/Endianness_t.md#_CPPv4N8eprosima7fastdds4rtps12Endianness_tE) msg_endian

Endianness of the message. 

### Public Static Functions

### static inline [CDRMessage_t](#_CPPv4N8eprosima7fastdds4rtps12CDRMessage_tE) wrap_from_other_message(const [CDRMessage_t](#_CPPv4N8eprosima7fastdds4rtps12CDRMessage_tE) &other, std::uint32_t length_from_current_position)

Wraps a portion of a [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t) into a new [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t). 


Note: If the specified length exceeds the available data in `other`, the returned [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t) will have length zero and buffer set to nullptr. 

* **Parameters:**
  * **other** – The [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t) to wrap from. 
  * **length_from_current_position** – The length of the portion to wrap, starting from the current position of `other`.
* **Returns:**
  A new [CDRMessage_t](#structeprosima_1_1fastdds_1_1rtps_1_1CDRMessage__t) wrapping the specified portion.
