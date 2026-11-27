<a id="api-pim-dynamicpubsubtype"></a>

<a id="dynamicpubsubtype"></a>

# DynamicPubSubType

### class DynamicPubSubType : public virtual eprosima::fastdds::dds::[TopicDataType](../topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)

### Public Functions

### DynamicPubSubType(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type)

Constructs a [DynamicPubSubType](#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType) from a [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType). 

**Remark**

Ownership is not transferred. 

* **Parameters:**
  **type** – [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) object associated to the data

### DynamicPubSubType(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type, const xtypes::TypeInformation &type_information)

Constructs a [DynamicPubSubType](#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType) from a [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) and a xtypes::TypeInformation. 

This constructor is only meant to be used when the registration of the corresponding [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) in the participant is meant to avoid registering the xtypes::TypeObject in the xtypes::TypeObjectRegistry; that is, when the xtypes::TypeObject registration has already been performed. This is for instance the case when the [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) is created from a xtypes::TypeObject obtained from a remote xtypes::TypeInformation that only has the minimal type identifiers but not the complete ones.

**Remark**

Ownership is not transferred. 

* **Parameters:**
  * **type** – [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) object associated to the data 
  * **type_information** – xtypes::TypeInformation object associated to the data

### virtual void \*create_data() override

Create a new data object of the specified type. 

**Remark**

Ownership is transferred. This object must be removed using [delete_data](#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType_1a6566adb7a3c1cb2a6762870a4e48eae1)

* **Returns:**
  pointer to the new object

### virtual void delete_data(void \*data) override

Deletes an object previously allocated via [create_data](#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType_1a3ea4f1caf2dca2e396c5bd5ea5193f0c). 

**Remark**

Ownership is transferred. This object must be allocated using [create_data](#classeprosima_1_1fastdds_1_1dds_1_1DynamicPubSubType_1a3ea4f1caf2dca2e396c5bd5ea5193f0c)

* **Parameters:**
  **data** – pointer to the object to be deleted

### virtual bool deserialize(eprosima::fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, void \*data) override

Deserialize an object from the given payload. 

* **Parameters:**
  * **payload** – [rtps::SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#structeprosima_1_1fastdds_1_1rtps_1_1SerializedPayload__t) to parse 
  * **data** – object to fill in with payload data
* **Returns:**
  bool specifying success 

### traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type get_dynamic_type() const noexcept

Returns a copy of the internal [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) object. 

* **Returns:**
  pointer to the new object 

### virtual bool compute_key(eprosima::fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, eprosima::fastdds::rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false) override

Calculate the key associated to a given object. 

* **Parameters:**
  * **payload** – [rtps::SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#structeprosima_1_1fastdds_1_1rtps_1_1SerializedPayload__t) containing the serialized object which key is calculated 
  * **ihandle** – [rtps::InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to fill in 
  * **force_md5** – use always md5 even if key payload footprint is smaller than the hash
* **Returns:**
  bool specifying success 

### virtual bool compute_key(const void \*const data, eprosima::fastdds::rtps::[InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle, bool force_md5 = false) override

Calculate the key associated to a given object. 

* **Parameters:**
  * **data** – object which key is calculated 
  * **ihandle** – [rtps::InstanceHandle_t](../../rtps/common/InstanceHandle/InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to fill in 
  * **force_md5** – use always md5 even if key payload footprint is smaller than the hash
* **Returns:**
  bool specifying success 

### virtual std::uint32_t calculate_serialized_size(const void \*const data, DataRepresentationId_t data_representation) override

Provide a functor that calculates a specified object serialized size. 

* **Parameters:**
  * **data** – **[in]** object whose payload footprint to calculate 
  * **data_representation** – **[in]** Representation that should be used for calculating the serialized size.
* **Returns:**
  functor that calculates the size 

### virtual bool serialize(const void \*const data, eprosima::fastdds::rtps::[SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE) &payload, fastdds::dds::DataRepresentationId_t data_representation) override

Serialize an object into a given payload. 

* **Parameters:**
  * **data** – **[in]** object to serialize 
  * **payload** – **[out]** [rtps::SerializedPayload_t](../../rtps/common/SerializedPayload/SerializedPayload_t.md#structeprosima_1_1fastdds_1_1rtps_1_1SerializedPayload__t) to fill in 
  * **data_representation** – **[in]** Representation that should be used to encode the data into the payload.
* **Returns:**
  bool specifying success 

### ReturnCode_t set_dynamic_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type)

Sets up the internal [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) object. 

**Remark**

Ownership is not transferred. 

* **Parameters:**
  **type** – [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) to copy
* **Returns:**
  ReturnCode_t with operation status

### virtual void register_type_object_representation() override

Register TypeObject representation in Fast DDS TypeObjectRegistry.
