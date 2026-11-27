<a id="api-utils-bufferawarecontext"></a>

<a id="bufferawarecontext-pro"></a>

# BufferAwareContext **[Pro]**

### struct BufferAwareContext : public eprosima::fastdds::dds::[TopicDataType](../../dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)::[Context](../../dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)

 **[Pro]**  A Context that is aware of [eprosima::fastdds::Buffer](buffer_class.md#classeprosima_1_1fastdds_1_1Buffer) and can serialize/deserialize it.

It is also a registry for [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) instances, allowing different backends to be registered and used for serialization/deserialization. 

### Public Functions

### void register_backend(const std::string &backend_type, const std::shared_ptr<[BufferBackend](bufferbackend.md#_CPPv4N8eprosima7fastdds13BufferBackendE)> &backend)

Registers a [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) for a specific backend type. 

* **Parameters:**
  * **backend_type** – The type of the backend (e.g., `cpu`, `cuda`, `demo`). 
  * **backend** – A shared pointer to the [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) instance to register. 

### std::shared_ptr<[BufferBackend](bufferbackend.md#_CPPv4N8eprosima7fastdds13BufferBackendE)> get_backend(const std::string &backend_type) const

Retrieves a registered [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) for a specific backend type. 

* **Parameters:**
  **backend_type** – The type of the backend to retrieve.
* **Returns:**
  A shared pointer to the registered [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) instance, or nullptr if not found. 

### template<typename \_Func>inline void for_each_backend([\_Func](#_CPPv4I0ENK8eprosima7fastdds18BufferAwareContext16for_each_backendEvRR5_Func) &&func) const

Iterates over all registered backends and applies a provided function to each. 

#### NOTE
This function locks the internal mutex to ensure thread safety while iterating over the backends, so the provided function should not call any method on the [BufferAwareContext](#structeprosima_1_1fastdds_1_1BufferAwareContext) as it may lead to a deadlock. 

* **Parameters:**
  **func** – A callable that takes two parameters:
  * the backend type (std::string).
  * a shared pointer to the [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend).

### std::size_t calculate_serialized_size(eprosima::fastcdr::CdrSizeCalculator &calculator, const eprosima::fastdds::[Buffer](buffer_class.md#_CPPv4I00EN8eprosima7fastdds6BufferE)<std::uint8_t> &data, std::size_t &current_alignment) const

Calculates the serialized size of a Buffer<uint8_t> instance. 

* **Parameters:**
  * **calculator** – The CdrSizeCalculator to use for size calculation. 
  * **data** – The Buffer<uint8_t> instance whose serialized size is to be calculated. 
  * **current_alignment** – The current alignment for serialization.
* **Returns:**
  The serialized size of the Buffer<uint8_t> instance. 

### void serialize(eprosima::fastcdr::Cdr &cdr, const eprosima::fastdds::[Buffer](buffer_class.md#_CPPv4I00EN8eprosima7fastdds6BufferE)<std::uint8_t> &data) const

Serializes a Buffer<uint8_t> instance into a Cdr stream. 

* **Parameters:**
  * **cdr** – The Cdr stream to serialize into. 
  * **data** – The Buffer<uint8_t> instance to serialize. 

### void deserialize(eprosima::fastcdr::Cdr &cdr, eprosima::fastdds::[Buffer](buffer_class.md#_CPPv4I00EN8eprosima7fastdds6BufferE)<std::uint8_t> &data) const

Deserializes a Buffer<uint8_t> instance from a Cdr stream. 

* **Parameters:**
  * **cdr** – The Cdr stream to deserialize from. 
  * **data** – The Buffer<uint8_t> instance to deserialize into.
