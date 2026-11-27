<a id="api-pim-annotationdescriptor"></a>

<a id="annotationdescriptor"></a>

# AnnotationDescriptor

### class AnnotationDescriptor

### Public Functions

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type() const = 0

Returns a reference to the type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &type() = 0

Returns a reference to the type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual ReturnCode_t get_value(ObjectName &value, const ObjectName &key) = 0

Getter given a key for the **value** property. 

* **Parameters:**
  * **value** – **[inout]** The value. 
  * **key** – **[in]** Key used to retrieve the value. 
* **Returns:**
  ReturnCode_t returns operation success 

### virtual ReturnCode_t get_all_value(Parameters &value) = 0

Getter for all the values. 

* **Parameters:**
  **value** – **[inout]** Parameters interface to the strings map. 
* **Returns:**
  ReturnCode_t returns operation success. 

### virtual ReturnCode_t set_value(const ObjectName &key, const ObjectName &value) = 0

Setter given a key for the **value** property. 

* **Parameters:**
  * **key** – **[in]** null terminated string 
  * **value** – **[in]** null terminated string 
* **Returns:**
  ReturnCode_t returns operation success 

### virtual ReturnCode_t copy_from(traits<[AnnotationDescriptor](#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type descriptor) = 0

Overwrites the contents of this descriptor with those of another descriptor (see [standard] 7.5.2.3.1) 

* **Parameters:**
  **descriptor** – **[in]** object 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  standard ReturnCode_t

### virtual bool equals(traits<[AnnotationDescriptor](#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type descriptor) = 0

State comparison according with the [standard] sections **7.5.2.3.2** 

* **Parameters:**
  **descriptor** – **[in]** reference. 
* **Returns:**
  **bool** `true` on equality 

### virtual bool is_consistent() = 0

Indicates whether the states of all of this descriptor’s properties are consistent according with the [standard] section **7.5.2.3.3**. 

* **Returns:**
  **bool** `true` if consistent.
