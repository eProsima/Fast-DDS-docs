<a id="api-pim-dynamicdatafactory"></a>

<a id="dynamicdatafactory"></a>

# DynamicDataFactory

### class DynamicDataFactory : public std::enable_shared_from_this<[DynamicDataFactory](#_CPPv4N8eprosima7fastdds3dds18DynamicDataFactoryE)>

### Public Functions

### virtual traits<[DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type create_data(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Creates a new [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference based on the given [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. All objects returned by this operation should eventually be deleted by calling delete_data. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference associated. 
* **Returns:**
  new [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference 

### virtual ReturnCode_t delete_data(traits<[DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type &data) = 0

Resets the internal reference if it is cached. 

* **Parameters:**
  **data** – **[in]** [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference whose internal cached reference to reset. 
* **Return values:**
  * **RETCODE_BAD_PARAMETER** – if reference is nil. 
  * **RETCODE_OK** – is otherwise returned. 
* **Returns:**
  standard ReturnCode_t 

### Public Static Functions

### static traits<[DynamicDataFactory](#_CPPv4N8eprosima7fastdds3dds18DynamicDataFactoryE)>::ref_type get_instance()

Returns the singleton factory object. 

**Remark**

This method is non thread-safe. 

* **Returns:**
  [DynamicDataFactory](#classeprosima_1_1fastdds_1_1dds_1_1DynamicDataFactory) reference. 

### static ReturnCode_t delete_instance()

Resets the singleton reference. 

* **Return values:**
  * **RETCODE_BAD_PARAMETER** – if singleton reference is currently nil. 
  * **RETCODE_OK** – otherwise. 
* **Returns:**
  ReturnCode_t
