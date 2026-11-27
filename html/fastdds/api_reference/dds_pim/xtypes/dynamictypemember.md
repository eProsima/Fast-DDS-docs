<a id="api-pim-dynamictypemember"></a>

<a id="dynamictypemember"></a>

# DynamicTypeMember

### class DynamicTypeMember : public std::enable_shared_from_this<[DynamicTypeMember](#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>

Represents a “member” of a type. 

A “member” in this sense may be a member of an aggregated type, a constant within an enumeration, or some other type substructure. 

### Public Functions

### virtual ReturnCode_t get_descriptor(traits<[MemberDescriptor](memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)>::ref_type &descriptor) = 0

Provides a summary of the state of this type overwriting a provided object (see [standard] 7.5.2.6.2) 

* **Parameters:**
  **descriptor** – **[inout]** [MemberDescriptor](memberdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) reference where the information is copied. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  standard ReturnCode_t

### virtual std::uint32_t get_annotation_count() = 0

Returns the number of applied annotations to the member. 

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

Returns the number of applied verbatim text to the member. 

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

### virtual bool equals(traits<[DynamicTypeMember](#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE)>::ref_type other) = 0

State comparison according with the [standard] sections **7.5.2.6.3** 

* **Parameters:**
  **other** – **[in]** [DynamicTypeMember](#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeMember) reference to compare to 
* **Returns:**
  **bool** `true` on equality 

### virtual MemberId get_id() = 0

Getter for **id** property according with the [standard] section **7.5.2.6.4** 

* **Returns:**
  MemberId

### virtual ObjectName get_name() = 0

Returns the name of this member. 

* **Returns:**
  Member name.
