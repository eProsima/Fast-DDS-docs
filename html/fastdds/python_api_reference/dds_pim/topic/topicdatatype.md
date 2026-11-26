<a id="python-api-pim-topicdatatype"></a>

<a id="topicdatatype"></a>

# TopicDataType

### *class* fastdds.TopicDataType(\*args, \*\*kwargs)

Class TopicDataType used to provide the DomainRTPSParticipant with the methods to serialize, deserialize and get the key of a specific data type.
The user should created a class that inherits from this one, where Serialize and deserialize methods MUST be implemented.

#### calculate_serialized_size(data, data_representation)

Calculates the serialized size of the provided data.

* **Parameters:**
  * **[****in****]** – data Pointer to data.
  * **[****in****]** – data_representation Representation that should be used for calculating the serialized size.
* **Return type:**
  int
* **Returns:**
  Serialized size of the data.

#### calculate_serialized_size_ctx(context, data, data_representation)

Calculates the serialized size of the provided data with context.
It can be reimplemented by the user to perform custom calculations depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during computation.
  * **[****in****]** – data Pointer to data.
  * **[****in****]** – data_representation Representation that should be used for calculating the serialized size.
* **Return type:**
  int
* **Returns:**
  Serialized size of the data.

#### compute_key(\*args)

*Overload 1:*

Get the key associated with the data.

* **Parameters:**
  * **[****in****]** – payload Pointer to the payload containing the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

<br/>

*Overload 2:*

Get the key associated with the data.

* **Parameters:**
  * **[****in****]** – data Pointer to the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

<br/>

*Overload 3:*

Get the key associated with the data.

* **Parameters:**
  * **[****in****]** – data Pointer to the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### compute_key_ctx(\*args)

*Overload 1:*

Get the key associated with the data with context.
It can be reimplemented by the user to perform custom key computation depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during computation.
  * **[****in****]** – payload Pointer to the payload containing the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

<br/>

*Overload 2:*

Get the key associated with the data with context.
It can be reimplemented by the user to perform custom key computation depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during computation.
  * **[****in****]** – data Pointer to the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

<br/>

*Overload 3:*

Get the key associated with the data with context.
It can be reimplemented by the user to perform custom key computation depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during computation.
  * **[****in****]** – data Pointer to the data.
  * **[****out****]** – ihandle Pointer to the Handle.
  * **[****in****]** – force_md5 Force MD5 checking.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### construct_sample(memory)

Construct a sample on a memory location.

* **Parameters:**
  **memory** (*void*) – Pointer to the memory location where the sample should be constructed.
* **Return type:**
  boolean
* **Returns:**
  whether this type supports in-place construction or not.

#### construct_sample_ctx(context, memory)

Construct a sample on a memory location using a context.
It can be reimplemented by the user to perform custom construction depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during construction.
  * **memory** (*void*) – Pointer to the memory location where the sample should be constructed.
* **Return type:**
  boolean
* **Returns:**
  whether this type supports in-place construction or not.

#### create_data()

Create a Data Type.

* **Return type:**
  void
* **Returns:**
  Void pointer to the created object.

#### create_data_ctx(context)

Create a Data Type with context.
It can be reimplemented by the user to perform custom creation depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  **context** (*std::shared_ptr< eprosima::fastdds::dds::TopicDataType::Context >*) – Context that can be used during creation.
* **Return type:**
  void
* **Returns:**
  Void pointer to the created object.

#### delete_data(data)

Remove a previously created object.

* **Parameters:**
  **data** (*void*) – Pointer to the created Data.

#### delete_data_ctx(context, data)

Remove a previously created object with context.
It can be reimplemented by the user to perform custom deletion depending on the context received
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** (*std::shared_ptr< eprosima::fastdds::dds::TopicDataType::Context >*) – Context that can be used during deletion.
  * **data** (*void*) – Pointer to the created Data.

#### deserialize(payload, data)

Deserialize method, it should be implemented by the user, since it is abstract.

* **Parameters:**
  * **[****in****]** – payload Pointer to the payload
  * **[****out****]** – data Pointer to the data
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### deserialize_ctx(context, payload, data)

Deserialize method with context, it can be reimplemented by the user to perform
custom deserialization depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during deserialization.
  * **[****in****]** – payload Pointer to the payload
  * **[****out****]** – data Pointer to the data
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### get_max_serialized_size_ctx(context)

Get the maximum serialized size of the type using a context
It can be reimplemented by the user to perform custom calculations depending on the context received.
The default implementation just returns the maximum serialized size without using the context.

* **Parameters:**
  **[****in****]** – context Context that can be used during computation.
* **Return type:**
  int
* **Returns:**
  Maximum serialized size of the type in bytes.

#### get_name()

Get topic data type name

* **Return type:**
  string
* **Returns:**
  Topic data type name

#### is_bounded()

Checks if the type is bounded.

#### is_bounded_ctx(context)

Checks if the type is bounded with context.
It can be reimplemented by the user to perform custom checks depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  **[****in****]** – context Context that can be used during check.

#### *property* is_compute_key_provided

Indicates whether the method to obtain the key has been implemented.

#### is_plain(arg2)

Checks if the type is plain when using a specific encoding.

#### is_plain_ctx(context, representation)

Checks if the type is plain when using a specific encoding and a context
It can be reimplemented by the user to perform custom checks depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  * **[****in****]** – context Context that can be used during check.
  * **[****in****]** – representation Representation of the data.

#### *property* max_serialized_type_size

Maximum serialized size of the type in bytes. If the type has unbounded fields, and therefore cannot have a maximum size, use 0.

#### register_type_object_representation()

Register TypeObject type representation

#### register_type_object_representation_ctx(context)

Register TypeObject type representation
It can be reimplemented by the user to perform custom registration depending on the context received.
The default implementation falls back to using the callback without context.

* **Parameters:**
  **[****in****]** – context Context that can be used during registration.

#### serialize(data, payload, data_representation)

Serialize method, it should be implemented by the user, since it is abstract. If not implemented, this method
will call the serialize method in which the topic data representation is not considered.
It is VERY IMPORTANT that the user sets the SerializedPayload length correctly.

* **Parameters:**
  * **[****in****]** – data Pointer to the data
  * **[****out****]** – payload Pointer to the payload
  * **[****in****]** – data_representation Representation that should be used to encode the data into the payload.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### serialize_ctx(context, data, payload, data_representation)

Serialize method with context, it can be reimplemented by the user to perform
custom serialization depending on the context received.
The default implementation falls back to using the callback without context.
It is VERY IMPORTANT that the user sets the SerializedPayload length correctly.

* **Parameters:**
  * **[****in****]** – context Context that can be used during serialization.
  * **[****in****]** – data Pointer to the data
  * **[****out****]** – payload Pointer to the payload
  * **[****in****]** – data_representation Representation that should be used to encode the data into the payload.
* **Return type:**
  boolean
* **Returns:**
  True if correct.

#### set_name(\*args)

*Overload 1:*

Set topic data type name

* **Parameters:**
  **nam** (*string*) – Topic data type name

<br/>

*Overload 2:*

Set topic data type name

* **Parameters:**
  **nam** (*std::string*) – Topic data type name

#### *property* thisown

The membership flag

#### type_identifiers()

Get the type identifiers

* **Return type:**
  eprosima::fastdds::dds::xtypes::TypeIdentifierPair
* **Returns:**
  ‘xtypes::TypeIdentifierPair’
