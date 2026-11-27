<a id="ipayloadpool"></a>

# IPayloadPool

### class IPayloadPool

An interface for classes responsible of serialized payload management. 

### Public Functions

### virtual bool get_payload(std::uint32_t size, [SerializedPayload_t](../common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload) = 0

Get a serialized payload for a new sample. 

This method will usually be called in one of the following situations: 
* When a writer creates a new cache change 
* When a reader receives the first fragment of a cache change


In both cases, the received `size` will be for the whole serialized payload.

* **Parameters:**
  * **size** – **[in]** Number of bytes required for the serialized payload. 
  * **payload** – **[inout]** Payload of the cache change used in the operation
* **Returns:**
  whether the operation succeeded or not
* **Post:**
  * Field `payload.payload_owner` equals this 
  * Field `payload.data` points to a buffer of at least `size` bytes 
  * Field `payload.max_size` is greater than or equal to `size`

### virtual bool get_payload(const [SerializedPayload_t](../common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &data, [SerializedPayload_t](../common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload) = 0

Assign a serialized payload to a new sample. 

This method will usually be called when a reader receives a whole cache change.

#### NOTE
If `data` has no owner, it means it is allocated on the stack of a reception thread, and a copy should be performed. If the ownership of `data` needs to be changed, a consecutive call to this method needs to be performed with the arguments swapped, leveraging the post-condition of this method which ensures that `payload.payload_owner` points to `this`.

* **Parameters:**
  * **data** – **[inout]** Serialized payload received 
  * **payload** – **[inout]** Destination serialized payload
* **Returns:**
  whether the operation succeeded or not
* **Post:**
  * Field `payload.payload_owner` equals this 
  * Field `payload.data` points to a buffer of at least `data.length` bytes 
  * Field `payload.length` is equal to `data.length`
  * Field `payload.max_size` is greater than or equal to `data.length`
  * Content of `payload.data` is the same as `data.data`

### virtual bool release_payload([SerializedPayload_t](../common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload) = 0

Release a serialized payload from a sample. 

This method will be called when a cache change is removed from a history.

* **Parameters:**
  **payload** – **[inout]** Payload to be released
* **Returns:**
  whether the operation succeeded or not
* **Pre:**
  * Field `payload_owner` of `payload` equals this
* **Post:**
  * Field `payload_owner` of `payload` is `nullptr`
