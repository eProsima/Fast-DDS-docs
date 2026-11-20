<a id="api-pim-typedescriptor"></a>

<a id="typedescriptor"></a>

# TypeDescriptor

### class TypeDescriptor

[TypeDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) definition according to [standard] section **7.5.2.4**. 

### Public Functions

### virtual TypeKind kind() const = 0

Returns the TypeKind associated. 

* **Returns:**
  standard TypeKind. 

### virtual TypeKind &kind() = 0

Returns the TypeKind associated. 

* **Returns:**
  standard TypeKind. 

### virtual void kind(TypeKind kind) = 0

Modifies the underlying TypeKind. 

* **Parameters:**
  **kind** – **[in]** TypeKind to be set. 

### virtual ObjectName &name() = 0

Returns the fully qualified name of this type. 

* **Returns:**
  Fully qualified name. 

### virtual const ObjectName &name() const = 0

Returns the fully qualified name of this type. 

* **Returns:**
  Fully qualified name. 

### virtual void name(const ObjectName &name) = 0

Modifies the underlying type name by copy. 

* **Parameters:**
  **name** – **[in]** Fully qualified name. 

### virtual void name(ObjectName &&name) = 0

Modifies the underlying type name by move. 

* **Parameters:**
  **name** – **[in]** Fully qualified name. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type base_type() const = 0

Returns a reference to the base type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &base_type() = 0

Returns a reference to the base type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void base_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying base type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type discriminator_type() const = 0

Returns a reference discriminator type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &discriminator_type() = 0

Returns a reference discriminator type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void discriminator_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying discriminator type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type literal_type() const = 0

Returns a reference to the literal type. 

The reference can be nil.

#### NOTE
This is only valid for TK_ENUM types, and it represents the expected type of its literals. 

#### WARNING
This method is an extension of the standard [TypeDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) interface (see [standard] section **7.5.2.4** ) 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &literal_type() = 0

Returns a reference to the literal type. 

The reference can be nil.

#### NOTE
This is only valid for TK_ENUM types, and it represents the expected type of its literals. 

#### WARNING
This method is an extension of the standard [TypeDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) interface (see [standard] section **7.5.2.4** ) 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void literal_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying literal type reference. 

#### NOTE
This is only valid for TK_ENUM types, and it represents the expected type of its literals. 

#### WARNING
This method is an extension of the standard [TypeDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) interface (see [standard] section **7.5.2.4** ) 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual const BoundSeq &bound() const = 0

Returns the bound. 

* **Returns:**
  BoundSeq. 

### virtual BoundSeq &bound() = 0

Returns the bound. 

* **Returns:**
  BoundSeq. 

### virtual void bound(const BoundSeq &bound) = 0

Modifies the underlying bound by copy. 

* **Parameters:**
  **bound** – **[in]** BoundSeq

### virtual void bound(BoundSeq &&bound) = 0

Modifies the underlying bound by move. 

* **Parameters:**
  **bound** – **[in]** BoundSeq

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type element_type() const = 0

Returns a reference element type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &element_type() = 0

Returns a reference element type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void element_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying element type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type key_element_type() const = 0

Returns a reference key element type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &key_element_type() = 0

Returns a reference key element type. 

The reference can be nil. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void key_element_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying key element type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual ExtensibilityKind extensibility_kind() const = 0

Returns the extensibility kind. 

return ExtensibilityKind

### virtual ExtensibilityKind &extensibility_kind() = 0

Returns the extensibility kind. 

return ExtensibilityKind

### virtual void extensibility_kind(ExtensibilityKind extensibility_kind) = 0

Modifies the extensibility kind. 

* **Parameters:**
  **extensibility_kind** – **[in]** ExtensibilityKind

### virtual bool is_nested() const = 0

Returns the is_nested property. 

return Boolean 

### virtual bool &is_nested() = 0

Returns the is_nested property. 

return Boolean 

### virtual void is_nested(bool is_nested) = 0

Modifies the is_nested property. 

* **Parameters:**
  **is_nested** – **[in]** Boolean value to be set. 

### virtual ReturnCode_t copy_from(traits<[TypeDescriptor](#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE)>::ref_type descriptor) = 0

Overwrites the contents of this descriptor with those of another descriptor (see [standard] 7.5.2.4.3). 

* **Parameters:**
  **descriptor** – **[in]** reference. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  ReturnCode_t

### virtual bool equals(traits<[TypeDescriptor](#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE)>::ref_type descriptor) = 0

Compares according with the [standard] section **7.5.2.4.6**. 

* **Parameters:**
  **descriptor** – **[in]** reference to compare to. 
* **Returns:**
  **bool** `true` on equality 

### virtual bool is_consistent() = 0

Indicates whether the states of all of this descriptor’s properties are consistent according with the [standard] section **7.5.2.4.7**. 

* **Returns:**
  **bool** `true` if consistent.
