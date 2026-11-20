<a id="api-pim-dynamictypebuilder"></a>

<a id="dynamictypebuilder"></a>

# DynamicTypeBuilder

### class DynamicTypeBuilder : public std::enable_shared_from_this<[DynamicTypeBuilder](#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>

### Public Functions

### virtual ReturnCode_t get_descriptor(traits<[TypeDescriptor](typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE)>::ref_type &descriptor) = 0

Provides a summary of the state of this type overwriting a provided object. 

* **Parameters:**
  **descriptor** – **[inout]** [TypeDescriptor](typedescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor)
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

Returns a member looked for by name. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the member. 
  * **name** – **[in]** Member name. 
* **Return values:**
  * **RETCODE_OK** – when member was found. 
  * **RETCODE_BAD_PARAMETER** – when member wasn’t found. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_all_members_by_name(DynamicTypeMembersByName &member) = 0

Returns all members sorted by name. 

* **Parameters:**
  **member** – **[inout]** DynamicTypeMemberByName reference used to return all members. 
* **Return values:**
  **RETCODE_OK** – always. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_member(traits<[DynamicTypeMember](dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type &member, MemberId id) = 0

Returns a member looked for by MemberId. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the member. 
  * **id** – **[in]** Member identifier. 
* **Return values:**
  * **RETCODE_OK** – when member was found. 
  * **RETCODE_BAD_PARAMETER** – when member wasn’t found. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_all_members(DynamicTypeMembersById &member) = 0

Returns all members sorted by MemberId. 

* **Parameters:**
  **member** – **[inout]** DynamicTypeMemberById reference used to return all members. 
* **Return values:**
  **RETCODE_OK** – always. 
* **Returns:**
  ReturnCode_t

### virtual std::uint32_t get_member_count() = 0

This operation returns the current number of members. 

* **Returns:**
  Current number of members 

### virtual ReturnCode_t get_member_by_index(traits<[DynamicTypeMember](dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type &member, std::uint32_t index) = 0

This operation returns the member that corresponds to the specified index. 

* **Parameters:**
  * **member** – **[inout]** [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference used to return the member. 
  * **index** – **[in]** Index 
* **Return values:**
  * **RETCODE_OK** – when member was found. 
  * **RETCODE_BAD_PARAMETER** – when index is out-of-range. 
* **Returns:**
  ReturnCode_t

### virtual std::uint32_t get_annotation_count() = 0

This operation returns the current number of annotations to the type. 

* **Returns:**
  Current number of annotations 

### virtual ReturnCode_t get_annotation(traits<[AnnotationDescriptor](annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type &descriptor, std::uint32_t idx) = 0

This operation returns the annotation that corresponds to the specified index. 

* **Parameters:**
  * **descriptor** – **[inout]** [AnnotationDescriptor](annotationdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1AnnotationDescriptor) reference where information is copied. 
  * **idx** – **[in]** Index 
* **Return values:**
  * **RETCODE_OK** – when member was found. 
  * **RETCODE_BAD_PARAMETER** – when reference is nil or index is out-of-range. 
* **Returns:**
  ReturnCode_t

### virtual bool equals(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type other) = 0

Compares current state against a [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

* **Parameters:**
  **other** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference to compare to. 
* **Returns:**
  **bool** `true` on equality 

### virtual ReturnCode_t add_member(traits<[MemberDescriptor](memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)>::ref_type descriptor) = 0

Adds a ‘member’ to this type, where the new ‘member’ has the meaning defined in the specification of the [DynamicTypeMember](dynamictypemember.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) class. 

* **Parameters:**
  **descriptor** – **[in]** [MemberDescriptor](memberdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) reference used for the new member. 
* **Return values:**
  * **RETCODE_OK** – when the member was created successfully. 
  * **RETCODE_BAD_PARAMETER** – when there is an inconsistency. 
  * **RETCODE_PRECONDITION_NOT_MET** – when the type does not have members. 
* **Returns:**
  ReturnCode_t 

### virtual ReturnCode_t apply_annotation(traits<[AnnotationDescriptor](annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type descriptor) = 0

Apply the given annotation to this type. 

* **Parameters:**
  **descriptor** – **[in]** [AnnotationDescriptor](annotationdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1AnnotationDescriptor) reference to be applied. 
* **Return values:**
  * **RETCODE_OK** – when the annotation was applied successful. 
  * **RETCODE_BAD_PARAMETER** – when there is an inconsistency. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t apply_annotation_to_member(MemberId member_id, traits<[AnnotationDescriptor](annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE)>::ref_type descriptor) = 0

Apply the given annotation to a member of this type. 

* **Parameters:**
  * **member_id** – **[in]** Member identifier. 
  * **descriptor** – **[in]** [AnnotationDescriptor](annotationdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1AnnotationDescriptor) reference to be applied. 
* **Return values:**
  * **RETCODE_OK** – when the annotation was applied successful. 
  * **RETCODE_BAD_PARAMETER** – when there is an inconsistency. 
* **Returns:**
  ReturnCode_t

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type build() = 0

Create an immutable [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) object containing a snapshot of this builder’s current state. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference.
