<a id="api-pim-memberdescriptor"></a>

<a id="memberdescriptor"></a>

# MemberDescriptor

### class MemberDescriptor

### Public Functions

### virtual ObjectName &name() = 0

Returns the name of this member. 

* **Returns:**
  Member’s name. 

### virtual const ObjectName &name() const = 0

Returns the name of this member. 

* **Returns:**
  Member’s name. 

### virtual void name(const ObjectName &name) = 0

Modifies the underlying member’s name by copy. 

* **Parameters:**
  **name** – **[in]** Member’s name. 

### virtual void name(ObjectName &&name) = 0

Modifies the underlying member’s name by move. 

* **Parameters:**
  **name** – **[in]** Member’s name. 

### virtual MemberId id() const = 0

Returns the MemberId of the member. 

* **Returns:**
  MemberId. 

### virtual MemberId &id() = 0

Returns the MemberId of the member. 

* **Returns:**
  MemberId. 

### virtual void id(MemberId id) = 0

Modifies the underlying MemberId. 

* **Parameters:**
  **id** – **[in]** MemberId to be set. 

### virtual MemberId position() const = 0

Returns the position of the bitmask member. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** ) 

* **Returns:**
  Position of the bitmask member. 

### virtual MemberId &position() = 0

Returns the position of the bitmask member. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** ) 

* **Returns:**
  Position of the bitmask member. 

### virtual void position(MemberId position) = 0

Modifies the position of the bitmask member. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** ) 

* **Parameters:**
  **position** – **[in]** Position of the bitmask member. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type() const = 0

Returns a reference to the member’s type. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &type() = 0

Returns a reference to the member’s type. 

* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual void type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Modifies the underlying member’s type reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

### virtual std::string &default_value() = 0

Returns the default value. 

* **Returns:**
  Default value. 

### virtual const std::string &default_value() const = 0

Returns the default value. 

* **Returns:**
  Default value. 

### virtual void default_value(const std::string &default_value) = 0

Modifies the underlying default value by copy. 

* **Parameters:**
  **default_value** – **[in]** Default value. 

### virtual void default_value(std::string &&default_value) = 0

Modifies the underlying default value by move. 

* **Parameters:**
  **default_value** – **[in]** Default value. 

### virtual std::string &literal_value() = 0

Returns the assigned value for members representing an enumerated literal. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** )

* **Returns:**
  The enumerated literal value 

### virtual const std::string &literal_value() const = 0

Returns the assigned value for members representing an enumerated literal. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** )

* **Returns:**
  The enumerated literal value 

### virtual void literal_value(const std::string &literal_value) = 0

Modifies the underlying literal value by copy. 

#### WARNING
This method is an extension of the standard [MemberDescriptor](#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) interface (see [standard] **7.5.2.7** ) 

* **Parameters:**
  **literal_value** – **[in]** Literal value. 

### virtual std::uint32_t &index() = 0

Returns the order of definition of the member. 

* **Returns:**
  Order of definition. 

### virtual std::uint32_t index() const = 0

Returns the order of definition of the member. 

* **Returns:**
  Order of definition. 

### virtual const UnionCaseLabelSeq &label() const = 0

Returns the labels the member belongs to. 

* **Returns:**
  UnionCaseLabelSeq. 

### virtual UnionCaseLabelSeq &label() = 0

Returns the labels the member belongs to. 

* **Returns:**
  UnionCaseLabelSeq. 

### virtual void label(const UnionCaseLabelSeq &label) = 0

Modifies the labels the member belongs to by copy. 

* **Parameters:**
  **label** – **[in]** UnionCaseLabelSeq

### virtual void label(UnionCaseLabelSeq &&label) = 0

Modifies the labels the member belongs to by move. 

* **Parameters:**
  **label** – **[in]** UnionCaseLabelSeq

### virtual TryConstructKind try_construct_kind() const = 0

Returns the TryConstructKind of the member. 

* **Returns:**
  TryConstructKind. 

### virtual TryConstructKind &try_construct_kind() = 0

Returns the TryConstructKind of the member. 

* **Returns:**
  TryConstructKind. 

### virtual void try_construct_kind(TryConstructKind try_construct_kind) = 0

Modifies the TryConstructKind of the member. 

* **Parameters:**
  **try_construct_kind** – **[in]** TryConstructKind. 

### virtual bool is_key() const = 0

Returns the if the member is key. 

* **Returns:**
  If the member is key. 

### virtual bool &is_key() = 0

Returns the if the member is key. 

* **Returns:**
  If the member is key. 

### virtual void is_key(bool is_key) = 0

Modifies if the member is key. 

* **Parameters:**
  **is_key** – **[in]** Boolean 

### virtual bool is_optional() const = 0

Returns the if the member is optional. 

* **Returns:**
  If the member is optional. 

### virtual bool &is_optional() = 0

Returns the if the member is optional. 

* **Returns:**
  If the member is optional. 

### virtual void is_optional(bool is_optional) = 0

Modifies if the member is optional. 

* **Parameters:**
  **is_optional** – **[in]** Boolean 

### virtual bool is_must_understand() const = 0

Returns the if the member is must_understand. 

* **Returns:**
  If the member is must_understand. 

### virtual bool &is_must_understand() = 0

Returns the if the member is must_understand. 

* **Returns:**
  If the member is must_understand. 

### virtual void is_must_understand(bool is_must_understand) = 0

Modifies if the member is must_understand. 

* **Parameters:**
  **is_must_understand** – **[in]** Boolean 

### virtual bool is_shared() const = 0

Returns the if the member is shared. 

* **Returns:**
  If the member is shared. 

### virtual bool &is_shared() = 0

Returns the if the member is shared. 

* **Returns:**
  If the member is shared. 

### virtual void is_shared(bool is_shared) = 0

Modifies if the member is shared. 

* **Parameters:**
  **is_shared** – **[in]** Boolean 

### virtual bool is_default_label() const = 0

Returns the if the member is default_label. 

* **Returns:**
  If the member is default_label. 

### virtual bool &is_default_label() = 0

Returns the if the member is default_label. 

* **Returns:**
  If the member is default_label. 

### virtual void is_default_label(bool is_default_label) = 0

Modifies if the member is default_label. 

* **Parameters:**
  **is_default_label** – **[in]** Boolean 

### virtual bool is_default_literal() const = 0

Returns if the member is a default literal. 

* **Returns:**
  If the member is a default literal 

### virtual bool &is_default_literal() = 0

Returns if the member is a default literal. 

* **Returns:**
  If the member is a default literal 

### virtual void is_default_literal(bool is_default_literal) = 0

Modifies if the member is a default literal. 

* **Parameters:**
  **is_default_literal** – **[in]** Boolean 

### virtual ReturnCode_t copy_from(traits<[MemberDescriptor](#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)>::ref_type descriptor) = 0

Overwrites the contents of this descriptor with those of another descriptor (see [standard] 7.5.2.7.1). 

* **Parameters:**
  **descriptor** – **[in]** reference. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  ReturnCode_t

### virtual bool equals(traits<[MemberDescriptor](#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)>::ref_type descriptor) = 0

Compares according with the [standard] section **7.5.2.7.4**. 

* **Parameters:**
  **descriptor** – **[in]** reference to compare to. 
* **Returns:**
  **bool** `true` on equality 

### virtual bool is_consistent() = 0

Indicates whether the states of all of this descriptor’s properties are consistent according with the [standard] section **7.5.2.7.7**. 

* **Returns:**
  **bool** `true` if consistent.
