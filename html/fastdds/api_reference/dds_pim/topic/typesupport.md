<a id="api-pim-typesupport"></a>

<a id="typesupport"></a>

# TypeSupport

### class TypeSupport : public std::shared_ptr<[TopicDataType](topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)>

Class [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) used to provide the DomainRTPSParticipant with the methods to serialize, deserialize and get the key of a specific data type. The user should created a class that inherits from this one, where Serialize and deserialize methods MUST be implemented. 

#### NOTE
This class inherits from std::shared_ptr<TopicDataType>. 

### Public Functions

### TypeSupport() noexcept = default

Constructor. 

### TypeSupport(const [TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupport11TypeSupportERK11TypeSupport) &type) noexcept = default

Copy Constructor. 

* **Parameters:**
  **type** – Another instance of [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport)

### TypeSupport([TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupport11TypeSupportERR11TypeSupport) &&type) noexcept = default

Move Constructor. 

* **Parameters:**
  **type** – Another instance of [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport)

### [TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) &operator=(const [TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) &type) noexcept = default

Copy Assignment. 

* **Parameters:**
  **type** – Another instance of [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport)

### [TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) &operator=([TypeSupport](#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) &&type) noexcept = default

Move Assignment. 

* **Parameters:**
  **type** – Another instance of [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport)

### inline explicit TypeSupport([TopicDataType](topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) \*ptr)

[TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) constructor that receives a [TopicDataType](topicdatatype.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataType) pointer. 

The passed pointer will be managed by the [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) object, so creating two [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) from the same pointer or deleting the passed pointer will produce a runtime error.

* **Parameters:**
  **ptr** – 

### virtual ReturnCode_t register_type([DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant) const

Registers the type on a participant. 

* **Parameters:**
  **participant** – [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) where the type is going to be registered 
* **Returns:**
  RETCODE_BAD_PARAMETER if the type name is empty, RETCODE_PRECONDITION_NOT_MET if there is another type with the same name registered on the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) and RETCODE_OK if it is registered correctly 

### virtual ReturnCode_t register_type([DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, std::string type_name) const

Registers the type on a participant. 

* **Parameters:**
  * **participant** – [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) where the type is going to be registered 
  * **type_name** – Name of the type to register 
* **Returns:**
  RETCODE_BAD_PARAMETER if the type name is empty, RETCODE_PRECONDITION_NOT_MET if there is another type with the same name registered on the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) and RETCODE_OK if it is registered correctly 

### inline virtual const std::string &get_type_name() const

Getter for the type name. 

* **Returns:**
  name of the data type 

### virtual bool serialize(const void \*const data, fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, DataRepresentationId_t data_representation)

Serializes the data. 

* **Parameters:**
  * **data** – Pointer to data 
  * **payload** – Pointer to payload 
  * **data_representation** – **[in]** Representation that should be used to encode the data into the payload. 
* **Returns:**
  true if it is serialized correctly, false if not 

### virtual bool deserialize(fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, void \*data)

Deserializes the data. 

* **Parameters:**
  * **payload** – Pointer to payload 
  * **data** – Pointer to data 
* **Returns:**
  true if it is deserialized correctly, false if not 

### inline virtual std::uint32_t calculate_serialized_size(const void \*const data, DataRepresentationId_t data_representation)

Returns a function which can be used to calculate the serialized size of the provided data. 

* **Parameters:**
  * **data** – **[in]** Pointer to data. 
  * **data_representation** – **[in]** Representation that should be used for calculating the serialized size. 
* **Returns:**
  Functor which calculates the serialized size of the data. 

### inline virtual void \*create_data()

Creates new data. 

* **Returns:**
  Pointer to the data 

### inline virtual void delete_data(void \*data)

Deletes data. 

* **Parameters:**
  **data** – Pointer to the data to delete 

### inline virtual bool compute_key(const void \*const data, InstanceHandle_t &i_handle, bool force_md5 = false)

Getter for the data key. 

* **Parameters:**
  * **data** – Pointer to serialized payload containing the data. 
  * **i_handle** – InstanceHandle pointer to store the key 
  * **force_md5** – boolean to force md5 (default: false) 
* **Returns:**
  true if the key is returned, false if not 

### inline virtual bool compute_key(fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, InstanceHandle_t &i_handle, bool force_md5 = false)

Getter for the data key. 

* **Parameters:**
  * **payload** – Pointer to data 
  * **i_handle** – InstanceHandle pointer to store the key 
  * **force_md5** – boolean to force md5 (default: false) 
* **Returns:**
  true if the key is returned, false if not 

### inline bool empty() const

Check if the [TypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) is empty. 

* **Returns:**
  true if empty, false if not 

### inline virtual bool is_bounded() const

Checks if the type is bounded. 

### inline virtual bool is_plain(DataRepresentationId_t data_representation) const

Checks if the type is plain when using a specific encoding.
