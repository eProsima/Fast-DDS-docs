<a id="api-pim-dynamictype"></a>

<a id="dynamictype"></a>

# DynamicType

### class DynamicType : public std::enable_shared_from_this<[DynamicType](#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>

### Public Functions

### virtual ReturnCode_t get_descriptor(traits<[TypeDescriptor](typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE)>::ref_type &descriptor) = 0

Provides a summary of the state of this type overwriting a provided object (see [standard] 7.5.2.8.7) 

* **Parameters:**
  **descriptor** – **[inout]** [TypeDescriptor](typedescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) reference where copied the information. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  ReturnCode_t

### virtual ObjectName get_name() = 0

Returns the fully qualified name of this type. 

* **Returns:**
  Type name. 

### virtual TypeKind get_kind() = 0

Returns the TypeKind associated. 

* **Returns:**
  TypeKind

### virtual ReturnCode_t get_member_by_name(traits<[DynamicTypeMember](dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type &member, const ObjectName &name) = 0

Returns the member that corresponds to the specified name. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the reference to the member. 
  * **name** – **[in]** Member name of the member being queried. 
* **Return values:**
  * **RETCODE_OK** – when the member was found. 
  * **RETCODE_BAD_PARAMETER** – when the member doesn’t exist. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_all_members_by_name(DynamicTypeMembersByName &member) = 0

Returns all members by ObjectName. 

* **Parameters:**
  **member** – **[inout]** DynamicTypeMembersByName reference where the information is copied. 
* **Return values:**
  **RETCODE_OK** – 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_member(traits<[DynamicTypeMember](dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type &member, MemberId id) = 0

Returns the member that corresponds to the specified MemberId. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the reference to the member. 
  * **id** – **[in]** MemberId
* **Return values:**
  * **RETCODE_OK** – when the member was found. 
  * **RETCODE_BAD_PARAMETER** – when the member doesn’t exist. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_all_members(DynamicTypeMembersById &member) = 0

Returns all members by MemberId. 

* **Parameters:**
  **member** – **[inout]** DynamicTypeMembersById reference where the information is copied. 
* **Return values:**
  **RETCODE_OK** – 
* **Returns:**
  ReturnCode_t

### virtual std::uint32_t get_member_count() = 0

This operation returns the current number of members. 

* **Returns:**
  Current number of members 

### virtual ReturnCode_t get_member_by_index(traits<[DynamicTypeMember](dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type &member, std::uint32_t index) = 0

This operation returns the member that corresponds to the specified index. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the reference to the member. 
  * **index** – **[in]** Index 
* **Return values:**
  * **RETCODE_OK** – when the member was found. 
  * **RETCODE_BAD_PARAMETER** – when the index is out-of-range. 
* **Returns:**
  ReturnCode_t

### virtual std::uint32_t get_annotation_count() = 0

Returns the number of applied annotations to the type. 

* **Returns:**
  Number of annotations. 

### virtual ReturnCode_t get_annotation(traits<[AnnotationDescriptor](annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type &descriptor, std::uint32_t idx) = 0

Returns an applied annotation by index. 

* **Parameters:**
  * **descriptor** – **[inout]** [AnnotationDescriptor](annotationdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1AnnotationDescriptor) reference where the information is copied. 
  * **idx** – **[in]** Index. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil or index is out-of-range. 
* **Returns:**
  standard ReturnCode_t

### virtual std::uint32_t get_verbatim_text_count() = 0

Returns the number of applied verbatim text to the type. 

* **Returns:**
  Number of verbatim texts. 

### virtual ReturnCode_t get_verbatim_text(traits<[VerbatimTextDescriptor](verbatimtextdescriptor.md#_CPPv4N8eprosima7fastdds3dds22VerbatimTextDescriptorE)>::ref_type &descriptor, std::uint32_t idx) = 0

Returns an applied verbatim text by index. 

* **Parameters:**
  * **descriptor** – **[inout]** [VerbatimTextDescriptor](verbatimtextdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1VerbatimTextDescriptor) reference where the information is copied. 
  * **idx** – **[in]** Index. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil or index is out-of-range. 
* **Returns:**
  standard ReturnCode_t

### virtual bool equals(traits<[DynamicType](#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type other) = 0

State comparison according with the [standard] sections **7.5.2.8.4** 

* **Parameters:**
  **other** – **[in]** [DynamicType](#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference to compare to 
* **Returns:**
  **bool** `true` on equality
