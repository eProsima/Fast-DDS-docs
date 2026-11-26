<a id="api-utils-bufferbackend"></a>

<a id="bufferbackend-pro"></a>

# BufferBackend **[Pro]**

### struct BufferBackend

 **[Pro]**  An interface for backends that can handle serialization and deserialization of BufferImplBase<uint8_t> instances. 

### Public Functions

### virtual std::size_t calculate_serialized_size(eprosima::fastcdr::CdrSizeCalculator &calculator, const [BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<std::uint8_t> &data, std::size_t &current_alignment) const = 0

Calculates the serialized size of a BufferImplBase<uint8_t> instance. 

* **Parameters:**
  * **calculator** – The CdrSizeCalculator to use for size calculation. 
  * **data** – The BufferImplBase<uint8_t> instance whose serialized size is to be calculated. 
  * **current_alignment** – The current alignment for serialization.
* **Returns:**
  The serialized size of the Buffer<uint8_t> instance. 

### virtual void serialize(eprosima::fastcdr::Cdr &cdr, const [BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<std::uint8_t> &data) const = 0

Serializes a BufferImplBase<uint8_t> instance into a Cdr stream. 

* **Parameters:**
  * **cdr** – The Cdr stream to serialize into. 
  * **data** – The BufferImplBase<uint8_t> instance to serialize. 

### virtual std::unique_ptr<[BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<std::uint8_t>> deserialize(eprosima::fastcdr::Cdr &cdr) const = 0

Deserializes a BufferImplBase<uint8_t> instance from a Cdr stream. 

* **Parameters:**
  **cdr** – The Cdr stream to deserialize from.
* **Returns:**
  A unique pointer to a new BufferImplBase<uint8_t> instance deserialized from the Cdr stream.
