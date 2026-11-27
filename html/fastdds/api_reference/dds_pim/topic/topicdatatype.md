<a id="api-pim-topicdatatype"></a>

<a id="topicdatatype"></a>

# TopicDataType

### class TopicDataType

Class [TopicDataType](#classeprosima_1_1fastdds_1_1dds_1_1TopicDataType) used to provide the DomainRTPSParticipant with the methods to serialize, deserialize and get the key of a specific data type. The user should created a class that inherits from this one, where Serialize and deserialize methods MUST be implemented. 

Subclassed by [eprosima::fastdds::dds::DynamicPubSubType](../xtypes/dynamicpubsubtype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType)

### Public Functions

### TopicDataType() = default

Constructor. 

### virtual ~TopicDataType() = default

Destructor. 

### virtual bool serialize(const void \*const data, rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, eprosima::fastdds::dds::DataRepresentationId_t data_representation) = 0

Serialize method, it should be implemented by the user, since it is abstract. If not implemented, this method will call the serialize method in which the topic data representation is not considered. It is VERY IMPORTANT that the user sets the SerializedPayload length correctly.

* **Parameters:**
  * **data** – **[in]** Pointer to the data 
  * **payload** – **[out]** Pointer to the payload 
  * **data_representation** – **[in]** Representation that should be used to encode the data into the payload. 
* **Returns:**
  True if correct. 

### inline virtual bool serialize_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, const void \*const data, rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, eprosima::fastdds::dds::DataRepresentationId_t data_representation)

Serialize method with context, it can be reimplemented by the user to perform custom serialization depending on the context received. The default implementation falls back to using the callback without context. It is VERY IMPORTANT that the user sets the SerializedPayload length correctly.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during serialization. 
  * **data** – **[in]** Pointer to the data 
  * **payload** – **[out]** Pointer to the payload 
  * **data_representation** – **[in]** Representation that should be used to encode the data into the payload. 
* **Returns:**
  True if correct. 

### virtual bool deserialize(rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, void \*data) = 0

Deserialize method, it should be implemented by the user, since it is abstract.

* **Parameters:**
  * **payload** – **[in]** Pointer to the payload 
  * **data** – **[out]** Pointer to the data 
* **Returns:**
  True if correct. 

### inline virtual bool deserialize_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, void \*data)

Deserialize method with context, it can be reimplemented by the user to perform custom deserialization depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during deserialization. 
  * **payload** – **[in]** Pointer to the payload 
  * **data** – **[out]** Pointer to the data 
* **Returns:**
  True if correct. 

### virtual std::uint32_t calculate_serialized_size(const void \*const data, eprosima::fastdds::dds::DataRepresentationId_t data_representation) = 0

Calculates the serialized size of the provided data. 

* **Parameters:**
  * **data** – **[in]** Pointer to data. 
  * **data_representation** – **[in]** Representation that should be used for calculating the serialized size. 
* **Returns:**
  Serialized size of the data. 

### inline virtual std::uint32_t calculate_serialized_size_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, const void \*const data, eprosima::fastdds::dds::DataRepresentationId_t data_representation)

Calculates the serialized size of the provided data with context. 

It can be reimplemented by the user to perform custom calculations depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during computation. 
  * **data** – **[in]** Pointer to data. 
  * **data_representation** – **[in]** Representation that should be used for calculating the serialized size. 
* **Returns:**
  Serialized size of the data. 

### virtual void \*create_data() = 0

Create a Data Type.

* **Returns:**
  Void pointer to the created object. 

### inline virtual void \*create_data_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context)

Create a Data Type with context. It can be reimplemented by the user to perform custom creation depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  **context** – [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during creation. 
* **Returns:**
  Void pointer to the created object. 

### virtual void delete_data(void \*data) = 0

Remove a previously created object.

* **Parameters:**
  **data** – Pointer to the created Data. 

### inline virtual void delete_data_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, void \*data)

Remove a previously created object with context. It can be reimplemented by the user to perform custom deletion depending on the context received The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during deletion. 
  * **data** – Pointer to the created Data. 

### virtual bool compute_key(rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false) = 0

Get the key associated with the data.

* **Parameters:**
  * **payload** – **[in]** Pointer to the payload containing the data. 
  * **ihandle** – **[out]** Pointer to the Handle. 
  * **force_md5** – **[in]** Force MD5 checking. 
* **Returns:**
  True if correct. 

### inline virtual bool compute_key_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false)

Get the key associated with the data with context. It can be reimplemented by the user to perform custom key computation depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during computation. 
  * **payload** – **[in]** Pointer to the payload containing the data. 
  * **ihandle** – **[out]** Pointer to the Handle. 
  * **force_md5** – **[in]** Force MD5 checking. 
* **Returns:**
  True if correct. 

### virtual bool compute_key(const void \*const data, rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false) = 0

Get the key associated with the data.

* **Parameters:**
  * **data** – **[in]** Pointer to the data. 
  * **ihandle** – **[out]** Pointer to the Handle. 
  * **force_md5** – **[in]** Force MD5 checking. 
* **Returns:**
  True if correct. 

### inline virtual bool compute_key_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, const void \*const data, rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false)

Get the key associated with the data with context. It can be reimplemented by the user to perform custom key computation depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during computation. 
  * **data** – **[in]** Pointer to the data. 
  * **ihandle** – **[out]** Pointer to the Handle. 
  * **force_md5** – **[in]** Force MD5 checking. 
* **Returns:**
  True if correct. 

### inline void set_name(const std::string &nam)

Set topic data type name

* **Parameters:**
  **nam** – [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) data type name 

### inline void set_name(std::string &&nam)

Set topic data type name

* **Parameters:**
  **nam** – [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) data type name 

### inline const std::string &get_name() const

Get topic data type name

* **Returns:**
  [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) data type name 

### inline const xtypes::TypeIdentifierPair &type_identifiers() const

Get the type identifiers

* **Returns:**
  xtypes::TypeIdentifierPair

### inline virtual bool is_bounded() const

Checks if the type is bounded. 

### inline virtual bool is_bounded_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context) const

Checks if the type is bounded with context. It can be reimplemented by the user to perform custom checks depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during check. 

### inline virtual bool is_plain(DataRepresentationId_t) const

Checks if the type is plain when using a specific encoding. 

### inline virtual bool is_plain_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, DataRepresentationId_t representation) const

Checks if the type is plain when using a specific encoding and a context It can be reimplemented by the user to perform custom checks depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during check. 
  * **representation** – **[in]** Representation of the data. 

### inline virtual bool construct_sample(void \*memory) const

Construct a sample on a memory location.

* **Parameters:**
  **memory** – Pointer to the memory location where the sample should be constructed.
* **Returns:**
  whether this type supports in-place construction or not. 

### inline virtual bool construct_sample_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context, void \*memory) const

Construct a sample on a memory location using a context. It can be reimplemented by the user to perform custom construction depending on the context received. The default implementation falls back to using the callback without context.

* **Parameters:**
  * **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during construction. 
  * **memory** – Pointer to the memory location where the sample should be constructed.
* **Returns:**
  whether this type supports in-place construction or not. 

### inline virtual void register_type_object_representation()

Register TypeObject type representation. 

### inline virtual void register_type_object_representation_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context)

Register TypeObject type representation It can be reimplemented by the user to perform custom registration depending on the context received. The default implementation falls back to using the callback without context. 

* **Parameters:**
  **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during registration. 

### inline virtual std::uint32_t get_max_serialized_size_ctx(const std::shared_ptr<[Context](#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE)> &context)

Get the maximum serialized size of the type using a context It can be reimplemented by the user to perform custom calculations depending on the context received. The default implementation just returns the maximum serialized size without using the context. 

* **Parameters:**
  **context** – **[in]** [Context](#structeprosima_1_1fastdds_1_1dds_1_1TopicDataType_1_1Context) that can be used during computation. 
* **Returns:**
  Maximum serialized size of the type in bytes. 

### Public Members

### std::uint32_t max_serialized_type_size = {0}

Maximum serialized size of the type in bytes. If the type has unbounded fields, and therefore cannot have a maximum size, use 0. 

### bool is_compute_key_provided = {false}

Indicates whether the method to obtain the key has been implemented. 

### struct Context : public fastcdr::CdrContext

An interface to provide the user with a context when serializing and deserializing data.

This context can be used to store specific information of the data type that can be used during serialization and deserialization. For example, it can be used to store the upper bounds of strings and sequences.

It inherits from `fastcdr::CdrContext`, so it can be directly used in a fastcdr objects. 

Subclassed by [eprosima::fastdds::BufferAwareContext](../../utils/buffer/bufferawarecontext.md#structeprosima_1_1fastdds_1_1BufferAwareContext)
