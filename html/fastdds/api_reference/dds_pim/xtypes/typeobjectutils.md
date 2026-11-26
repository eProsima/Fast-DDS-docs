<a id="api-pim-typeobjectutils"></a>

<a id="typeobjectutils"></a>

# TypeObjectUtils

### class TypeObjectUtils

### Public Static Functions

### static const TypeObjectHashId build_type_object_hash_id(std::uint8_t discriminator, const EquivalenceHash &hash)

Build TypeObjectHashId instance. 

* **Parameters:**
  * **discriminator** – **[in]** TypeObjectHashId discriminator to be set. 
  * **hash** – **[in]** StronglyConnectedComponent equivalence hash to be set. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given discriminator is not EK_COMPLETE/EK_MINIMAL. 
* **Returns:**
  const TypeObjectHashId instance. 

### static CollectionElementFlag build_collection_element_flag(TryConstructFailAction try_construct_kind, bool external)

Build CollectionElementFlag instance. 

* **Parameters:**
  * **try_construct_kind** – **[in]** try_construct annotation value. 
  * **external** – **[in]** external annotation value. 
* **Returns:**
  CollectionElementFlag instance. 

### static StructMemberFlag build_struct_member_flag(TryConstructFailAction try_construct_kind, bool optional, bool must_understand, bool key, bool external)

Build StructMemberFlag instance. 

* **Parameters:**
  * **try_construct_kind** – **[in]** try_construct annotation value. 
  * **optional** – **[in]** optional annotation value. 
  * **must_understand** – **[in]** must_understand annotation value. 
  * **key** – **[in]** key annotation value. 
  * **external** – **[in]** external annotation value. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if both key and optional flags are enabled. 
* **Returns:**
  StructMemberFlag instance. 

### static UnionMemberFlag build_union_member_flag(TryConstructFailAction try_construct_kind, bool default_member, bool external)

Build UnionMemberFlag instance. 

* **Parameters:**
  * **try_construct_kind** – **[in]** try_construct annotation value. 
  * **default_member** – **[in]** is default member. 
  * **external** – **[in]** external annotation value. 
* **Returns:**
  UnionMemberFlag instance. 

### static UnionDiscriminatorFlag build_union_discriminator_flag(TryConstructFailAction try_construct_kind, bool key)

Build UnionDiscriminatorFlag instance. 

* **Parameters:**
  * **try_construct_kind** – **[in]** try_construct annotation value. 
  * **key** – **[in]** key annotation value. 
* **Returns:**
  UnionDiscriminatorFlag instance. 

### static EnumeratedLiteralFlag build_enumerated_literal_flag(bool default_literal)

Build EnumeratedLiteralFlag instance. 

* **Parameters:**
  **default_literal** – **[in]** default_literal annotation value. 
* **Returns:**
  EnumeratedLiteralFlag instance. 

### static StructTypeFlag build_struct_type_flag(ExtensibilityKind extensibility_kind, bool nested, bool autoid_hash)

Build StructTypeFlag instance. 

AnnotationParameterFlag: Unused. No flags apply. AliasMemberFlag: Unused. No flags apply. BitflagFlag: Unused. No flags apply. BitsetMemberFlag: Unused. No flags apply. 

* **Parameters:**
  * **extensibility_kind** – **[in]** extensibility annotation value. 
  * **nested** – **[in]** nested annotation value. 
  * **autoid_hash** – **[in]** autoid annotation has HASH value. 
* **Returns:**
  StructTypeFlag instance. 

### static UnionTypeFlag build_union_type_flag(ExtensibilityKind extensibility_kind, bool nested, bool autoid_hash)

Build UnionTypeFlag instance. 

* **Parameters:**
  * **extensibility_kind** – **[in]** extensibility annotation value. 
  * **nested** – **[in]** nested annotation value. 
  * **autoid_hash** – **[in]** autoid annotation has HASH value. 
* **Returns:**
  UnionTypeFlag instance. 

### static const StringSTypeDefn build_string_s_type_defn(SBound bound)

Build StringSTypeDefn instance. 

CollectionTypeFlag: Unused. No flags apply. AnnotationTypeFlag: Unused. No flags apply. AliasTypeFlag: Unused. No flags apply. EnumTypeFlag: Unused. No flags apply. BitmaskTypeFlag: Unused. No flags apply. BitsetTypeFlag: Unused. No flags apply. 

* **Parameters:**
  **bound** – **[in]** Bound for the small string/wstring. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if bound is 0. 
* **Pre:**
  bound > 0 (INVALID_SBOUND) 
* **Returns:**
  const StringSTypeDefn instance. 

### static const StringLTypeDefn build_string_l_type_defn(LBound bound)

Build StringLTypeDefn instance. 

* **Parameters:**
  **bound** – **[in]** Bound for the large string/wstring. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if bound is lower than 256. 
* **Pre:**
  bound > 255 
* **Returns:**
  const StringLTypeDefn instance. 

### static const PlainCollectionHeader build_plain_collection_header(EquivalenceKind equiv_kind, CollectionElementFlag element_flags)

Build PlainCollectionHeader instance. 

* **Parameters:**
  * **equiv_kind** – **[in]** EquivalenceKind: EK_MINIMAL/EK_COMPLETE/EK_BOTH 
  * **element_flags** – **[in]** CollectionElementFlags to be set. This element must be constructed with the corresponding builder to ensure its consistency. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – if the given element_flags are inconsistent. This exception is only thrown in Debug build mode. 
* **Returns:**
  const PlainCollectionHeader instance. 

### static const PlainSequenceSElemDefn build_plain_sequence_s_elem_defn(const PlainCollectionHeader &header, SBound s_bound, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier)

Build PlainSequenceSElemDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **s_bound** – **[in]** Sequence bound. 
  * **element_identifier** – **[in]** Sequence element TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. The given bound is 0.
  2. The given TypeIdentifier EquivalenceKind is not consistent with the one contained in the header.
  3. Inconsistent header (only in Debug build mode).
  4. Inconsistent element_identifier (only in Debug build mode). 
* **Pre:**
  bound > 0 (INVALID_SBOUND) 
* **Pre:**
  element_identifier has been initialized. 
* **Returns:**
  const PlainSequenceSElemDefn instance. 

### static const PlainSequenceLElemDefn build_plain_sequence_l_elem_defn(const PlainCollectionHeader &header, LBound l_bound, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier)

Build PlainSequenceLElemDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **l_bound** – **[in]** Sequence bound. 
  * **element_identifier** – **[in]** Sequence element TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. Bound lower than 256.
  2. The given TypeIdentifier EquivalenceKind is not consistent with the one contained in the header.
  3. Inconsistent header (only in Debug build mode).
  4. Inconsistent element_identifier (only in Debug build mode). 
* **Pre:**
  bound > 255 
* **Pre:**
  element_identifier has been initialized. 
* **Returns:**
  const PlainSequenceLElemDefn instance. 

### static inline void add_array_dimension(std::vector<SBound> &array_bound_seq, SBound dimension_bound)

Add dimension bound to the array bound sequence. 

* **Template Parameters:**
  * **array** – Either a SBoundSeq or LBoundSeq. 
  * **element** – Either a SBound or LBound. 
* **Parameters:**
  * **array_bound_seq** – **[inout]** Sequence with the array bounds. 
  * **dimension_bound** – **[in]** Dimension bound to be added into the sequence. 

### static const PlainArraySElemDefn build_plain_array_s_elem_defn(const PlainCollectionHeader &header, const SBoundSeq &array_bound_seq, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier)

Build PlainArraySElemDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **array_bound_seq** – **[in]** Bounds for the array dimensions. 
  * **element_identifier** – **[in]** Array element TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. Any given bound in array_bound_seq is 0.
  2. The given TypeIdentifier EquivalenceKind is not consistent with the one contained in the header.
  3. Inconsistent header (only in Debug build mode).
  4. Inconsistent element_identifier (only in Debug build mode). 
* **Pre:**
  Any element in array_bound_seq must be greater than 0 (INVALID_SBOUND) 
* **Pre:**
  element_identifier has been initialized. 
* **Returns:**
  const PlainArraySElemDefn instance. 

### static const PlainArrayLElemDefn build_plain_array_l_elem_defn(const PlainCollectionHeader &header, const LBoundSeq &array_bound_seq, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier)

Build PlainArrayLElemDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **array_bound_seq** – **[in]** Bounds for the array dimensions. 
  * **element_identifier** – **[in]** Array element TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. Any given bound in array_bound_seq is 0.
  2. There is no dimension with a bound greater than 255.
  3. The given TypeIdentifier EquivalenceKind is not consistent with the one contained in the header.
  4. Inconsistent header (only in Debug build mode).
  5. Inconsistent element_identifier (only in Debug build mode). 
* **Pre:**
  At least one element of array_bound_seq must be greater than 255 and no element must be 0 (INVALID_SBOUND) 
* **Pre:**
  element_identifier has been initialized. 
* **Returns:**
  const PlainArrayLElemDefn instance. 

### static const PlainMapSTypeDefn build_plain_map_s_type_defn(const PlainCollectionHeader &header, const SBound bound, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier, const CollectionElementFlag key_flags, const eprosima::fastcdr::external<TypeIdentifier> &key_identifier)

Build PlainMapSTypeDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **bound** – **[in]** Map bound. 
  * **element_identifier** – **[in]** Map element TypeIdentifier. 
  * **key_flags** – **[in]** Flags applying to map key. 
  * **key_identifier** – **[in]** Map key TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. Given bound is zero (INVALID_SBOUND)
  2. Inconsistent element_identifier EquivalenceKind with the one contained in the header.
  3. Direct hash key_identifier or indirect hash TypeIdentifier with exception to string/wstring. XTypes v1.3 Clause 7.2.2.4.3: Implementers of this specification need only support key elements of signed and unsigned integer types and of narrow and wide string types.
  4. Inconsistent key_flags.
  5. Inconsistent header (only in Debug build mode).
  6. Inconsistent element_identifier or key_identifier (only in Debug build mode). 
* **Pre:**
  bound > 0 (INVALID_SBOUND) 
* **Pre:**
  Both element_identifier and key_identifier have been initialized. 
* **Returns:**
  const PlainMapSTypeDefn instance. 

### static const PlainMapLTypeDefn build_plain_map_l_type_defn(const PlainCollectionHeader &header, const LBound bound, const eprosima::fastcdr::external<TypeIdentifier> &element_identifier, const CollectionElementFlag key_flags, const eprosima::fastcdr::external<TypeIdentifier> &key_identifier)

Build PlainMapLTypeDefn instance. 

* **Parameters:**
  * **header** – **[in]** PlainCollectionHeader to be set. 
  * **bound** – **[in]** Map bound. 
  * **element_identifier** – **[in]** Map element TypeIdentifier. 
  * **key_flags** – **[in]** Flags applying to map key. 
  * **key_identifier** – **[in]** Map key TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception
  1. Given bound is lower than 256
  2. Inconsistent element_identifier EquivalenceKind with the one contained in the header.
  3. Direct hash key_identifier or indirect hash TypeIdentifier with exception to string/wstring. XTypes v1.3 Clause 7.2.2.4.3: Implementers of this specification need only support key elements of signed and unsigned integer types and of narrow and wide string types.
  4. Inconsistent key_flags.
  5. Inconsistent header (only in Debug build mode).
  6. Inconsistent element_identifier or key_identifier (only in Debug build mode). 
* **Pre:**
  bound > 255 
* **Pre:**
  Both element_identifier and key_identifier have been initialized. 
* **Returns:**
  const PlainMapLTypeDefn instance. 

### static const StronglyConnectedComponentId build_strongly_connected_component_id(const TypeObjectHashId &sc_component_id, std::int32_t scc_length, std::int32_t scc_index)

Build StronglyConnectedComponentId instance. 

* **Parameters:**
  * **sc_component_id** – **[in]** Strongly Connected Component (SCC) ID. 
  * **scc_length** – **[in]** Number of components within SCC. 
  * **scc_index** – **[in]** Identify specific component within SCC. 
* **Returns:**
  const StronglyConnectedComponentId instance. 

### static const ExtendedTypeDefn build_extended_type_defn()

Build ExtendedTypeDefn instance (empty. Available for future extension). 

* **Returns:**
  const ExtendedTypeDefn instance. 

### static ReturnCode_t build_and_register_s_string_type_identifier(const StringSTypeDefn &string, const std::string &type_name, TypeIdentifierPair &type_ids, bool wstring = false)

Register small string/wstring TypeIdentifier into TypeObjectRegistry. 

Primitive types are registered when TypeObjectRegistry is instantiated. 

* **Parameters:**
  * **string** – **[in]** StringSTypeDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the StringSTypeDefn just registered. 
  * **wstring** – **[in]** Flag to build a wstring. Default false. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_l_string_type_identifier(const StringLTypeDefn &string, const std::string &type_name, TypeIdentifierPair &type_ids, bool wstring = false)

Register large string/wstring TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **string** – **[in]** StringLTypeDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the StringLTypeDefn just registered. 
  * **wstring** – **[in]** Flag to build a wstring. Default false. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_s_sequence_type_identifier(const PlainSequenceSElemDefn &plain_seq, const std::string &type_name, TypeIdentifierPair &type_ids)

Register small sequence TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_seq** – **[in]** PlainSequenceSElemDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainSequenceSElemDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_l_sequence_type_identifier(const PlainSequenceLElemDefn &plain_seq, const std::string &type_name, TypeIdentifierPair &type_ids)

Register large sequence TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_seq** – **[in]** PlainSequenceLElemDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainSequenceLElemDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_s_array_type_identifier(const PlainArraySElemDefn &plain_array, const std::string &type_name, TypeIdentifierPair &type_ids)

Register small array TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_array** – **[in]** PlainArraySElemDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainArraySElemDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_l_array_type_identifier(const PlainArrayLElemDefn &plain_array, const std::string &type_name, TypeIdentifierPair &type_ids)

Register large array TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_array** – **[in]** PlainArrayLElemDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainArrayLElemDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_s_map_type_identifier(const PlainMapSTypeDefn &plain_map, const std::string &type_name, TypeIdentifierPair &type_ids)

Register small map TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_map** – **[in]** PlainMapSTypeDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainMapSTypeDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_l_map_type_identifier(const PlainMapLTypeDefn &plain_map, const std::string &type_name, TypeIdentifierPair &type_ids)

Register large map TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **plain_map** – **[in]** PlainMapLTypeDefn union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the PlainMapLTypeDefn just registered. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given member is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_scc_type_identifier(const StronglyConnectedComponentId &scc, const std::string &type_name)

Register StronglyConnectedComponent TypeIdentifier into TypeObjectRegistry. 

* **Parameters:**
  * **scc** – **[in]** StronglyConnectedComponent union member to set. 
  * **type_name** – **[in]** Type name to be registered. 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static const ExtendedAnnotationParameterValue build_extended_annotation_parameter_value()

Build ExtendedAnnotationParameterValue instance (empty. Available for future extension). 

* **Returns:**
  const ExtendedAnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(bool boolean_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **boolean_value** – **[in]** Boolean value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value_byte(std::uint8_t byte_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **byte_value** – **[in]** Byte value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::int8_t int8_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **int8_value** – **[in]** Int8 value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::uint8_t uint8_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **uint8_value** – **[in]** Unsigned int8 value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::int16_t int16_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **int16_value** – **[in]** Short value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::uint16_t uint16_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **uint16_value** – **[in]** Unsigned short value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::int32_t int32_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **int32_value** – **[in]** Long value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::uint32_t uint32_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **uint32_value** – **[in]** Unsigned long value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::int64_t int64_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **int64_value** – **[in]** Long long value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(std::uint64_t uint64_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **uint64_value** – **[in]** Unsigned long long value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(float float32_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **float32_value** – **[in]** Float value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(double float64_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **float64_value** – **[in]** Double value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(long double float128_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **float128_value** – **[in]** Long double value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(char char_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **char_value** – **[in]** Char value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(wchar_t wchar_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **wchar_value** – **[in]** Wide char value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value_enum(std::int32_t enumerated_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **enumerated_value** – **[in]** Enumerated value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(const eprosima::fastcdr::fixed_string<128> &string8_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **string8_value** – **[in]** String value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AnnotationParameterValue build_annotation_parameter_value(const std::wstring &string16_value)

Build AnnotationParameterValue instance. 

* **Parameters:**
  **string16_value** – **[in]** Wide string value to set in the union. 
* **Returns:**
  const AnnotationParameterValue instance. 

### static const AppliedAnnotationParameter build_applied_annotation_parameter(const NameHash &paramname_hash, const AnnotationParameterValue &value)

Build AppliedAnnotationParameter instance. 

* **Parameters:**
  * **paramname_hash** – **[in]** Parameter name hash. 
  * **value** – **[in]** Parameter value. 
* **Returns:**
  const AppliedAnnotationParameter instance. 

### static void add_applied_annotation_parameter(AppliedAnnotationParameterSeq &param_seq, const AppliedAnnotationParameter &param)

Add AppliedAnnotationParameter to the sequence. 

* **Parameters:**
  * **param_seq** – **[inout]** AppliedAnnotationParameter sequence to be modified. 
  * **param** – **[in]** AppliedAnnotationParameter to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the parameter being added has already been included in the sequence. 

### static const AppliedAnnotation build_applied_annotation(const TypeIdentifier &annotation_typeid, const eprosima::fastcdr::optional<AppliedAnnotationParameterSeq> &param_seq)

Build AppliedAnnotation instance. 

* **Parameters:**
  * **annotation_typeid** – **[in]** Annotation TypeIdentifier. 
  * **param_seq** – **[in]** Annotation parameters. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given annotation_typeid TypeIdentifier does not correspond to an annotation TypeObject (only in Debug build mode).
  2. Given AppliedAnnotationParameterSeq is inconsistent (only in Debug build mode).
  3. Given annotation TypeIdentifier corresponds to a builtin annotation and the given parameters are inconsistent (only in Debug build mode). 
* **Returns:**
  const AppliedAnnotation instance. 

### static void add_applied_annotation(AppliedAnnotationSeq &ann_custom_seq, const AppliedAnnotation &ann_custom)

Add AppliedAnnotation to the sequence. 

* **Parameters:**
  * **ann_custom_seq** – **[inout]** AppliedAnnotation sequence to be modified. 
  * **ann_custom** – **[in]** AppliedAnnotation to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given AppliedAnnotation is not consistent (only in Debug build mode).
  2. Given AppliedAnnotation is already present in the sequence. 

### static const AppliedVerbatimAnnotation build_applied_verbatim_annotation(PlacementKind placement, const eprosima::fastcdr::fixed_string<32> &language, const std::string &text)

Build AppliedVerbatimAnnotation instance. 

* **Parameters:**
  * **placement** – **[in]** Verbatim annotation placement parameter. 
  * **language** – **[in]** Verbatim annotation language parameter. 
  * **text** – **[in]** Verbatim annotation text parameter. 
* **Returns:**
  const AppliedVerbatimAnnotation instance. 

### static const AppliedBuiltinMemberAnnotations build_applied_builtin_member_annotations(const eprosima::fastcdr::optional<std::string> &unit, const eprosima::fastcdr::optional<AnnotationParameterValue> &min, const eprosima::fastcdr::optional<AnnotationParameterValue> &max, const eprosima::fastcdr::optional<std::string> &hash_id)

Build AppliedBuiltinMemberAnnotations instance. 

* **Parameters:**
  * **unit** – **[in]** Unit annotation value. 
  * **min** – **[in]** Min annotation value. 
  * **max** – **[in]** Max annotation value. 
  * **hash_id** – **[in]** Hashid annotation value. 
* **Returns:**
  const AppliedBuiltinMemberAnnotations instance. 

### static const CommonStructMember build_common_struct_member(MemberId member_id, StructMemberFlag member_flags, const TypeIdentifier &member_type_id)

Build CommonStructMember instance. 

* **Parameters:**
  * **member_id** – **[in]** Member identifier. 
  * **member_flags** – **[in]** Member flags: optional, must_understand, key, and external. 
  * **member_type_id** – **[in]** Member TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception:
  1. The given flags are not consistent (only in Debug build mode).
  2. The given TypeIdentifier is not consistent (only in Debug build mode). 
* **Returns:**
  const CommonStructMember instance. 

### static const CompleteMemberDetail build_complete_member_detail(const MemberName &name, const eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> &ann_builtin, const eprosima::fastcdr::optional<AppliedAnnotationSeq> &ann_custom)

Build CompleteMemberDetail instance. 

* **Parameters:**
  * **name** – **[in]** Member name. 
  * **ann_builtin** – **[in]** Member builtin annotations. 
  * **ann_custom** – **[in]** Member custom annotations. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Empty member name.
  2. Given AppliedAnnotationSeq is not consistent (only Debug build mode). 
* **Returns:**
  const CompleteMemberDetail instance. 

### static const CompleteStructMember build_complete_struct_member(const CommonStructMember &common, const CompleteMemberDetail &detail)

Build CompleteStructMember instance. 

MinimalMemberDetail constructed from CompleteMemberDetail

* **Parameters:**
  * **common** – **[in]** CommonStructMember to be set. 
  * **detail** – **[in]** CompleteMemberDetail to be set. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonStructMember is inconsistent (only Debug build mode).
  2. Given CompleteMemberDetail is inconsistent (only Debug build mode). 
* **Returns:**
  const CompleteMemberDetail instance. 

### static void add_complete_struct_member(CompleteStructMemberSeq &member_seq, const CompleteStructMember &member)

Add CompleteStructMember to the sequence. 

* **Parameters:**
  * **member_seq** – **[inout]** CompleteStructMember sequence to be modified. 
  * **member** – **[in]** CompleteStructMember to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CompleteStructMember is not consistent (only in Debug build mode).
  2. There is already another member in the sequence with the same member id or the same member name (only in Debug build mode). 

### static const AppliedBuiltinTypeAnnotations build_applied_builtin_type_annotations(const eprosima::fastcdr::optional<AppliedVerbatimAnnotation> &verbatim)

Build AppliedBuiltinTypeAnnotations instance. 

MinimalStructMember constructed from CompleteStructMember

* **Parameters:**
  **verbatim** – **[in]** AppliedVerbatimAnnotation to be set. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given verbatim annotation is inconsistent (only in Debug build mode). 
* **Returns:**
  const AppliedBuiltinTypeAnnotations instance. 

### static const CompleteTypeDetail build_complete_type_detail(const eprosima::fastcdr::optional<AppliedBuiltinTypeAnnotations> &ann_builtin, const eprosima::fastcdr::optional<AppliedAnnotationSeq> &ann_custom, const QualifiedTypeName &type_name)

Build CompleteTypeDetail instance. 

MinimalTypeDetail constructed from CompleteTypeDetail: empty. Available for future extension. 

* **Parameters:**
  * **ann_builtin** – **[in]** Verbatim annotation. 
  * **ann_custom** – **[in]** Applied annotations. 
  * **type_name** – **[in]** Name of the type. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. any applied annotation is not consistent (only Debug build mode). 
* **Returns:**
  const CompleteTypeDetail instance. 

### static const CompleteStructHeader build_complete_struct_header(const TypeIdentifier &base_type, const CompleteTypeDetail &detail)

Build CompleteStructHeader instance. 

* **Parameters:**
  * **base_type** – **[in]** TypeIdentifier of the parent structure (inheritance). 
  * **detail** – **[in]** CompleteTypeDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given TypeIdentifier is not consistent (direct HASH or empty TypeIdentifier). In Debug build mode the corresponding TypeObject is also checked in case of direct HASH TypeIdentifier.
  2. Given CompleteTypeDetail is not consistent (only in Debug build mode). 
* **Returns:**
  const CompleteStructHeader instance. 

### static const CompleteStructType build_complete_struct_type(StructTypeFlag struct_flags, const CompleteStructHeader &header, const CompleteStructMemberSeq &member_seq)

Build CompleteStructType instance. 

MinimalStructHeader constructed from CompleteStructHeader. 

* **Parameters:**
  * **struct_flags** – **[in]** StructTypeFlags. 
  * **header** – **[in]** CompleteStructHeader. 
  * **member_seq** – **[in]** Sequence of CompleteStructMembers. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given StructTypeFlag is not consistent (only in Debug build mode).
  2. Given CompleteStructHeader is not consistent (only in Debug build mode).
  3. Given CompleteStructMemberSeq is not consistent (only in Debug build mode).
  4. Given flags are not consistent with the builtin annotations. 
* **Returns:**
  const CompleteStructType instance. 

### static void add_union_case_label(UnionCaseLabelSeq &label_seq, std::int32_t label)

Add label to the union case label sequence. 

MinimalStructType constructed from CompleteStructType. 

* **Parameters:**
  * **label_seq** – **[inout]** Sequence to be modified. 
  * **label** – **[in]** Label to be added. 

### static const CommonUnionMember build_common_union_member(MemberId member_id, UnionMemberFlag member_flags, const TypeIdentifier &type_id, const UnionCaseLabelSeq &label_seq)

Build CommonUnionMember instance. 

* **Parameters:**
  * **member_id** – **[in]** Member identifier. 
  * **member_flags** – **[in]** Member flags. 
  * **type_id** – **[in]** Member TypeIdentifier. 
  * **label_seq** – **[in]** Member applicable case labels. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given UnionMemberFlags are not consistent (only Debug build mode).
  2. Given TypeIdentifier is not consistent (only Debug build mode). 
* **Returns:**
  const CommonUnionMember instance. 

### static const CompleteUnionMember build_complete_union_member(const CommonUnionMember &common, const CompleteMemberDetail &detail)

Build CompleteUnionMember instance. 

* **Parameters:**
  * **common** – **[in]** CommonUnionMember. 
  * **detail** – **[in]** CompleteMemberDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonUnionMember is not consistent (only in Debug build mode).
  2. Given CompleteMemberDetail is not consistent (only in Debug build mode). 
* **Returns:**
  const CompleteUnionMember instance. 

### static void add_complete_union_member(CompleteUnionMemberSeq &complete_union_member_seq, const CompleteUnionMember &member)

Add CompleteUnionMember to sequence. 

* **Parameters:**
  * **complete_union_member_seq** – **[inout]** Sequence to be modified. 
  * **member** – **[in]** Complete union member to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CompleteUnionMember is not consistent (only in Debug build mode).
  2. There is already another member in the sequence with the same member id or the same member name (only in Debug build mode).
  3. If the given member is marked as default, if there is already another member marked as default (only in Debug build mode).
  4. There are repeated union case labels (only in Debug build mode).
  5. Member name is protected (“discriminator”). 

### static const CommonDiscriminatorMember build_common_discriminator_member(UnionDiscriminatorFlag member_flags, const TypeIdentifier &type_id)

Build CommonDiscriminatorMember instance. 

MinimalUnionMember constructed from CompleteUnionMember. 

* **Parameters:**
  * **member_flags** – **[in]** Discriminator flags. 
  * **type_id** – **[in]** Discriminator TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given discriminator flags are inconsistent (only in Debug build mode).
  2. Given TypeIdentifier is not consistent. XTypes v1.3 Clause 7.2.2.4.4.3 The discriminator of a union must be one of the following types: Boolean, Byte, Char8, Char16, Int8, Uint8, Int16, Uint16, Int32, Uint32, Int64, Uint64, any enumerated type, any alias type that resolves, directly or indirectly, to one of the aforementioned types. 
* **Returns:**
  const CommonDiscriminatorMember instance. 

### static const CompleteDiscriminatorMember build_complete_discriminator_member(const CommonDiscriminatorMember &common, const eprosima::fastcdr::optional<AppliedBuiltinTypeAnnotations> &ann_builtin, const eprosima::fastcdr::optional<AppliedAnnotationSeq> &ann_custom)

Build CompleteDiscriminatorMember instance. 

* **Parameters:**
  * **common** – **[in]** CommonDiscriminatorMember. 
  * **ann_builtin** – **[in]** Verbatim annotation. 
  * **ann_custom** – **[in]** Applied annotations. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonDiscriminatorMember is inconsistent (only in Debug build mode).
  2. AppliedBuiltinTypeAnnotation is inconsistent (only in Debug build mode).
  3. Any given AppliedAnnotation is inconsistent (only in Debug build mode).
  4. CommonDiscriminatorMember is inconsistent with given builtin annotations. 
* **Returns:**
  const CompleteDiscriminatorMember instance. 

### static const CompleteUnionHeader build_complete_union_header(const CompleteTypeDetail &detail)

Build CompleteUnionHeader instance. 

MinimalDiscriminatorMember constructed from CompleteDiscriminatorMember. 

* **Parameters:**
  **detail** – CompleteTypeDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given CompleteTypeDetail is not consistent (only in Debug build mode). 
* **Returns:**
  const CompleteUnionHeader instance. 

### static const CompleteUnionType build_complete_union_type(UnionTypeFlag union_flags, const CompleteUnionHeader &header, const CompleteDiscriminatorMember &discriminator, const CompleteUnionMemberSeq &member_seq)

Build CompleteUnionType instance. 

MinimalUnionHeader constructed from CompleteUnionHeader. 

* **Parameters:**
  * **union_flags** – **[in]** 
  * **header** – **[in]** 
  * **discriminator** – **[in]** 
  * **member_seq** – **[in]** 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given UnionTypeFlags are not consistent (only in Debug build mode).
  2. Given CompleteUnionHeader is not consistent (only in Debug build mode).
  3. Given CompleteDiscriminatorMember inconsistent (only in Debug build mode).
  4. Given CompleteUnionMemberSeq is not consistent (only in Debug build mode).
  5. Given flags are not consistent with the builtin annotations. 
* **Returns:**
  const CompleteUnionType instance. 

### static const CommonAnnotationParameter build_common_annotation_parameter(AnnotationParameterFlag member_flags, const TypeIdentifier &member_type_id)

Build CommonAnnotationParameter instance. 

MinimalUnionType constructed from CompleteUnionType. 

* **Parameters:**
  * **member_flags** – **[in]** AnnotationParameterFlag: empty. No flags apply. It must be zero. 
  * **member_type_id** – **[in]** Member TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given AnnotationParameterFlag are not empty.
  2. Given TypeIdentifier is not consistent (only in Debug build mode). 
* **Returns:**
  const CommonAnnotationParameter instance. 

### static const CompleteAnnotationParameter build_complete_annotation_parameter(const CommonAnnotationParameter &common, const MemberName &name, const AnnotationParameterValue &default_value)

Build CompleteAnnotationParameter instance. 

* **Parameters:**
  * **common** – **[in]** CommonAnnotationParameter. 
  * **name** – **[in]** Member name. 
  * **default_value** – **[in]** Annotation default value. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonAnnotationParameter is inconsistent (only in Debug build mode).
  2. CommonAnnotationParameter TypeIdentifier is inconsistent with AnnotationParameterValue type.
  3. Given parameter name is empty. 
* **Returns:**
  const CompleteAnnotationParameter instance. 

### static void add_complete_annotation_parameter(CompleteAnnotationParameterSeq &sequence, const CompleteAnnotationParameter &param)

Add CompleteAnnotationParameter to sequence. 

* **Parameters:**
  * **sequence** – **[inout]** Sequence to be modified. 
  * **param** – **[in]** Complete annotation parameter to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CompleteAnnotationParameter is not consistent (only in Debug build mode).
  2. There is already another member in the sequence with the same member id or the same member name (only in Debug build mode). 

### static const CompleteAnnotationHeader build_complete_annotation_header(const QualifiedTypeName &annotation_name)

Build CompleteAnnotationHeader instance. 

MinimalAnnotationParameter constructed from CompleteAnnotationParameter. 

* **Parameters:**
  **annotation_name** – **[in]** Qualified annotation type name. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – if the annotation_name is empty. 
* **Returns:**
  const CompleteAnnotationHeader instance. 

### static const CompleteAnnotationType build_complete_annotation_type(AnnotationTypeFlag annotation_flag, const CompleteAnnotationHeader &header, const CompleteAnnotationParameterSeq &member_seq)

Build CompleteAnnotationType instance. 

MinimalAnnotationHeader constructed from CompleteAnnotationHeader: empty. Available for future extension. 

* **Parameters:**
  * **annotation_flag** – **[in]** Unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteAnnotationHeader. 
  * **member_seq** – **[in]** CompleteAnnotationParameter sequence. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any annotation flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Any CompleteAnnotationParameter in the sequence is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteAnnotationType instance. 

### static const CommonAliasBody build_common_alias_body(AliasMemberFlag related_flags, const TypeIdentifier &related_type)

Build CommonAliasBody instance. 

MinimalAnnotationType constructed from CompleteAnnotationType. 

* **Parameters:**
  * **related_flags** – **[in]** AliasMemberFlag: unused. No flags apply. It must be 0. 
  * **related_type** – **[in]** Related TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any alias member flag is set.
  2. Non-consistent TypeIdentifier (only in Debug build mode). 
* **Returns:**
  const CommonAliasBody instance. 

### static const CompleteAliasBody build_complete_alias_body(const CommonAliasBody &common, const eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> &ann_builtin, const eprosima::fastcdr::optional<AppliedAnnotationSeq> &ann_custom)

Build CompleteAliasBody instance. 

* **Parameters:**
  * **common** – **[in]** CommonAliasBody. 
  * **ann_builtin** – **[in]** Applied builtin member annotations: unit, max, min, range, hashid 
  * **ann_custom** – **[in]** Applied custom annotations 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonAliasBody is inconsistent (only Debug build mode).
  2. AppliedAnnotationSeq is inconsistent (only Debug build mode).
  3. hashid builtin annotation is set. 
* **Returns:**
  const CompleteAliasBody instance. 

### static const CompleteAliasHeader build_complete_alias_header(const CompleteTypeDetail &detail)

Build CompleteAliasHeader instance. 

MinimalAliasBody constructed from CompleteAliasBody. 

* **Parameters:**
  **detail** – **[in]** Complete type detail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given CompleteTypeDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteAliasHeader instance. 

### static const CompleteAliasType build_complete_alias_type(AliasTypeFlag alias_flags, const CompleteAliasHeader &header, const CompleteAliasBody &body)

Build CompleteAliasType instance. 

MinimalAliasHeader constructed from CompleteAliasHeader: empty. Available for future extension. 

* **Parameters:**
  * **alias_flags** – **[in]** Alias type flags: unused. No flags apply. It must be zero. 
  * **header** – **[in]** CompleteAliasHeader. 
  * **body** – **[in]** CompleteAliasBody. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any alias type flag is set.
  2. Inconsistent header and/or body (only in Debug build mode). 
* **Returns:**
  const CompleteAliasType instance. 

### static const CompleteElementDetail build_complete_element_detail(const eprosima::fastcdr::optional<AppliedBuiltinMemberAnnotations> &ann_builtin, const eprosima::fastcdr::optional<AppliedAnnotationSeq> &ann_custom)

Build CompleteElementDetail instance. 

MinimalAliasType constructed from CompleteAliasType. 

* **Parameters:**
  * **ann_builtin** – **[in]** Applied builtin member annotations: unit, max, min, range, hashid 
  * **ann_custom** – **[in]** Applied custom annotations 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. AppliedAnnotationSeq is inconsistent (only Debug build mode).
  2. hashid builtin annotation is applied. 
* **Returns:**
  const CompleteElementDetail instance. 

### static const CommonCollectionElement build_common_collection_element(CollectionElementFlag element_flags, const TypeIdentifier &type)

Build CommonCollectionElement instance. 

* **Parameters:**
  * **element_flags** – **[in]** CollectionElementFlag. 
  * **type** – **[in]** TypeIdentifier. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given collection element flags are not consistent (only in Debug build mode).
  2. Given TypeIdentifier is not consistent (only in Debug build mode). 
* **Returns:**
  const CommonCollectionElement instance 

### static const CompleteCollectionElement build_complete_collection_element(const CommonCollectionElement &common, const CompleteElementDetail &detail)

Build CompleteCollectionElement instance. 

* **Parameters:**
  * **common** – **[in]** CommonCollectionElement. 
  * **detail** – **[in]** CompleteElementDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonCollectionElement is not consistent (only in Debug build mode).
  2. Given CompleteElementDetail is not consistent (only in Debug build mode). 
* **Returns:**
  const CompleteCollectionElement instance 

### static const CommonCollectionHeader build_common_collection_header(LBound bound)

Build CommonCollectionHeader instance. 

MinimalCollectionElement constructed from CompleteCollectionElement. 

* **Parameters:**
  **bound** – **[in]** Collection bound. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given bound is not consistent. 
* **Returns:**
  const CommonCollectionHeader instance. 

### static const CompleteCollectionHeader build_complete_collection_header(const CommonCollectionHeader &common, const eprosima::fastcdr::optional<CompleteTypeDetail> &detail)

Build CompleteCollectionHeader instance. 

* **Parameters:**
  * **common** – **[in]** CommonCollectionHeader
  * **detail** – **[in]** CompleteTypeDetail
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonCollectionHeader is inconsistent (only in Debug build mode).
  2. Given CompleteTypeDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteCollectionHeader instance. 

### static const CompleteSequenceType build_complete_sequence_type(CollectionTypeFlag collection_flag, const CompleteCollectionHeader &header, const CompleteCollectionElement &element)

Build CompleteSequenceType instance. 

MinimalCollectionHeader constructed from CompleteCollectionHeader. 

* **Parameters:**
  * **collection_flag** – **[in]** collection type flag: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteCollectionHeader. 
  * **element** – **[in]** CompleteCollectionElement. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any collection flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Given element is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteSequenceType instance 

### static const CommonArrayHeader build_common_array_header(const LBoundSeq &bound_seq)

Build CommonArrayHeader instance. 

MinimalSequenceType constructed from CompleteSequenceType. 

* **Parameters:**
  **bound_seq** – **[in]** Sequence of the dimension’s bounds. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if any given bound is 0 (invalid). 
* **Returns:**
  const CommonArrayHeader instance. 

### static const CompleteArrayHeader build_complete_array_header(const CommonArrayHeader &common, const CompleteTypeDetail &detail)

Build CompleteArrayHeader instance. 

* **Parameters:**
  * **common** – **[in]** CommonArrayHeader. 
  * **detail** – **[in]** CompleteTypeDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonArrayHeader is inconsistent (only in Debug build mode).
  2. Given CompleteTypeDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteArrayHeader instance. 

### static const CompleteArrayType build_complete_array_type(CollectionTypeFlag collection_flag, const CompleteArrayHeader &header, const CompleteCollectionElement &element)

Build CompleteArrayType instance. 

MinimalArrayHeader constructed from CompleteArrayHeader. 

* **Parameters:**
  * **collection_flag** – **[in]** collection type flag: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteArrayHeader. 
  * **element** – **[in]** CompleteCollectionElement. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any collection flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Given element is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteArrayType instance. 

### static const CompleteMapType build_complete_map_type(CollectionTypeFlag collection_flag, const CompleteCollectionHeader &header, const CompleteCollectionElement &key, const CompleteCollectionElement &element)

Build CompleteMapType instance. 

MinimalArrayType constructed from CompleteArrayType. 

* **Parameters:**
  * **collection_flag** – **[in]** collection type flag: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteArrayHeader. 
  * **key** – **[in]** CompleteCollectionElement describing map key. 
  * **element** – **[in]** CompleteCollectionElement describing map element. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any collection flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Given key TypeIdentifier is inconsistent.
  4. Given key description is inconsistent (only in Debug build mode).
  5. Given element is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteMapType instance. 

### static const CommonEnumeratedLiteral build_common_enumerated_literal(std::int32_t value, EnumeratedLiteralFlag flags)

Build CommonEnumeratedLiteral instance. 

MinimalMapType constructed from CompleteMapType. 

* **Parameters:**
  * **value** – **[in]** Enumerated literal value. 
  * **flags** – **[in]** Enumerated literal flags: only default flag apply. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if any other flag different from default is set (only in Debug build mode). 
* **Returns:**
  const CommonEnumeratedLiteral instance. 

### static const CompleteEnumeratedLiteral build_complete_enumerated_literal(const CommonEnumeratedLiteral &common, const CompleteMemberDetail &detail)

Build CompleteEnumeratedLiteral instance. 

* **Parameters:**
  * **common** – **[in]** CommonEnumeratedLiteral. 
  * **detail** – **[in]** CompleteMemberDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonEnumeratedLiteral is inconsistent (only in Debug build mode).
  2. Given CompleteMemberDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteEnumeratedLiteral instance. 

### static void add_complete_enumerated_literal(CompleteEnumeratedLiteralSeq &sequence, const CompleteEnumeratedLiteral &enum_literal)

Add CompleteEnumeratedLiteral to sequence. 

* **Parameters:**
  * **sequence** – **[in]** Sequence to be modified. 
  * **enum_literal** – **[inout]** CompleteEnumeratedLiteral to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonEnumeratedLiteral is not consistent (only in Debug build mode).
  2. There is already another literal in the sequence with the same value or the same member name (only in Debug build mode). 

### static const CommonEnumeratedHeader build_common_enumerated_header(BitBound bit_bound, bool bitmask = false)

Build CommonEnumeratedHeader instance. 

MinimalEnumeratedLiteral constructed from CompleteEnumeratedLiteral. 

* **Parameters:**
  * **bit_bound** – **[in]** XTypes v1.3 Clause 7.3.1.2.1.5 It is important to note that the value member of the [bit_bound] annotation may take any value from 1 to 32, inclusive, when this annotation is applied to an enumerated type. 
  * **bitmask** – **[in]** Flag in case that the header being built corresponds to a Bitmask. By default is false. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given bit_bound is not consistent. 
* **Returns:**
  const CommonEnumeratedHeader instance. 

### static const CompleteEnumeratedHeader build_complete_enumerated_header(const CommonEnumeratedHeader &common, const CompleteTypeDetail &detail, bool bitmask = false)

Build CompleteEnumeratedHeader instance. 

* **Parameters:**
  * **common** – **[in]** CommonEnumeratedHeader. 
  * **detail** – **[in]** CompleteTypeDetail. 
  * **bitmask** – **[in]** flag set if the given header corresponds to a bitmask. Only required in Debug build mode. Set to false by default. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonEnumeratedHeader is inconsistent (only in Debug build mode).
  2. Given CompleteTypeDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteEnumeratedHeader instance. 

### static const CompleteEnumeratedType build_complete_enumerated_type(EnumTypeFlag enum_flags, const CompleteEnumeratedHeader &header, const CompleteEnumeratedLiteralSeq &literal_seq)

Build CompleteEnumeratedType instance. 

MinimalEnumeratedHeader constructed from CompleteEnumeratedHeader. 

* **Parameters:**
  * **enum_flags** – **[in]** Enumeration flags: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteEnumeratedHeader. 
  * **literal_seq** – **[in]** Sequence of CompleteEnumeratedLiterals. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any flag is set.
  2. Given CompleteEnumeratedHeader is inconsistent (only in Debug build mode).
  3. Given CompleteEnumeratedLiteralSeq is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteEnumeratedType instance. 

### static const CommonBitflag build_common_bitflag(std::uint16_t position, BitflagFlag flags)

Build CommonBitflag instance. 

MinimalEnumeratedType constructed from CompleteEnumeratedType. 

* **Parameters:**
  * **position** – **[in]** Bit position in the bitmask. 
  * **flags** – **[in]** Bit flags: unused. No flags apply. It must be 0. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. any given flag is set.
  2. given position is inconsistent. XTypes v1.3 Clause 7.2.2.4.1.2 Each bit in this subset is identified by name and by an index, numbered from 0 to (bound-1). The bound must be greater than zero and no greater than 64. 
* **Returns:**
  const CommonBitflag instance. 

### static const CompleteBitflag build_complete_bitflag(const CommonBitflag &common, const CompleteMemberDetail &detail)

Build CompleteBitflag instance. 

* **Parameters:**
  * **common** – **[in]** CommonBitflag. 
  * **detail** – **[in]** CompleteMemberDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonBitflag is inconsistent (only in Debug build mode).
  2. Given CompleteMemberDetail is inconsistent (only in Debug build mode).
  3. Non-applicable builtin annotations applied. 
* **Returns:**
  const CompleteBitflag instance. 

### static void add_complete_bitflag(CompleteBitflagSeq &sequence, const CompleteBitflag &bitflag)

Add complete bitflag to the sequence. 

* **Parameters:**
  * **sequence** – **[inout]** Sequence to be modified. 
  * **bitflag** – **[in]** CompleteBitflag to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given bitflag is inconsistent (only in Debug build mode).
  2. There is already another bitflag in the sequence with the same position or the same name (only in Debug build mode). 

### static const CompleteBitmaskType build_complete_bitmask_type(BitmaskTypeFlag bitmask_flags, const CompleteBitmaskHeader &header, const CompleteBitflagSeq &flag_seq)

Build CompleteBitmaskType instance. 

MinimalBitflag constructed from CompleteBitflag. CommonBitmaskHeader is not used. CompleteBitmaskHeader is defined as CompleteEnumeratedHeader. MinimalBitmaskHeader is defined as MinimalEnumeratedHeader. 

* **Parameters:**
  * **bitmask_flags** – **[in]** Bitmask flags: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteBitmaskHeader/CompleteEnumeratedHeader 
  * **flag_seq** – **[in]** Sequence of CompleteBitflag. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. any given flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Given Bitflag sequence is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteBitmaskType instance. 

### static const CommonBitfield build_common_bitfield(std::uint16_t position, BitsetMemberFlag flags, std::uint8_t bitcount, TypeKind holder_type)

Build CommonBitfield instance. 

* **Parameters:**
  * **position** – **[in]** Bitfield starting position bit. 
  * **flags** – **[in]** Bitfield flags: unused. No flags apply. It must be 0. 
  * **bitcount** – **[in]** Bitfield number of bits. IDL v4.2 Clause 7.4.13.4.3.2 The first one (positive_int_const) is the number of bits that can be stored (its [bitfield] size). The maximum value is 64. 
  * **holder_type** – **[in]** Type used to manipulate the bitfield. IDL v4.2 Clause 7.4.13.4.3.2 The second optional one (destination_type) specifies the type that will be used to manipulate the bit field as a whole. This type can be boolean, octet or any integer type either signed or unsigned. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given position is not consistent.
  2. Any flag is set.
  3. Given bitcount is not consistent.
  4. Given holder_type is not consistent. 
* **Returns:**
  const CommonBitfield instance. 

### static const CompleteBitfield build_complete_bitfield(const CommonBitfield &common, const CompleteMemberDetail &detail)

Build CompleteBitfield instance. 

* **Parameters:**
  * **common** – **[in]** CommonBitfield. 
  * **detail** – **[in]** CompleteMemberDetail. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given CommonBitfield is inconsistent (only Debug build mode).
  2. Give CompleteMemberDetail is inconsistent (only Debug build mode).
  3. Non-applicable builtin annotations are applied. 
* **Returns:**
  const CompleteBitfield instance. 

### static void add_complete_bitfield(CompleteBitfieldSeq &sequence, const CompleteBitfield &bitfield)

Add complete bitfield to the sequence. 

* **Parameters:**
  * **sequence** – **[inout]** Sequence to be modified. 
  * **bitfield** – **[in]** CompleteBitfield to be added. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Given bitfield is inconsistent (only in Debug build mode).
  2. There is another bitfield with the same name and/or the same position. 

### static const CompleteBitsetHeader build_complete_bitset_header(const CompleteTypeDetail &detail)

Build CompleteBitsetHeader instance. 

MinimalBitfield constructed from CompleteBitfield. 

* **Parameters:**
  **detail** – **[in]** CompleteTypeDetail
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given CompleteTypeDetail is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteBitsetHeader instance. 

### static const CompleteBitsetType build_complete_bitset_type(BitsetTypeFlag bitset_flags, const CompleteBitsetHeader &header, const CompleteBitfieldSeq &field_seq)

Build CompleteBitsetType instance. 

MinimalBitsetHeader constructed from CompleteBitsetHeader. 

* **Parameters:**
  * **bitset_flags** – **[in]** Bitset flags: unused. No flags apply. It must be 0. 
  * **header** – **[in]** CompleteBitsetHeader. 
  * **field_seq** – **[in]** Sequence of complete bitfields. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if:
  1. Any given flag is set.
  2. Given header is inconsistent (only in Debug build mode).
  3. Given bitfield sequence is inconsistent (only in Debug build mode). 
* **Returns:**
  const CompleteBitsetType instance. 

### static const CompleteExtendedType build_complete_extended_type()

Build CompleteExtendedType instance. (empty. Available for future extension) 

MinimalBitsetType constructed from CompleteBitsetType. 

* **Returns:**
  const CompleteExtendedType instance. 

### static ReturnCode_t build_and_register_alias_type_object(const CompleteAliasType &alias_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register alias TypeObject into TypeObjectRegistry. CompleteAliasType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **alias_type** – **[in]** CompleteAliasType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteAliasType just registered and the generated MinimalAliasType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_annotation_type_object(const CompleteAnnotationType &annotation_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register annotation TypeObject into TypeObjectRegistry. CompleteAnnotationType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **annotation_type** – **[in]** CompleteAnnotationType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteAnnotationType just registered and the generated MinimalAnnotationType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_struct_type_object(const CompleteStructType &struct_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register structure TypeObject into TypeObjectRegistry. CompleteStructType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **struct_type** – **[in]** CompleteStructType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteStructType just registered and the generated MinimalStructType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_union_type_object(const CompleteUnionType &union_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register union TypeObject into TypeObjectRegistry. CompleteUnionType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **union_type** – **[in]** CompleteUnionType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteUnionType just registered and the generated MinimalUnionType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_bitset_type_object(const CompleteBitsetType &bitset_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register bitset TypeObject into TypeObjectRegistry. CompleteBitsetType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **bitset_type** – **[in]** CompleteBitsetType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteBitsetType just registered and the generated MinimalBitsetType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_sequence_type_object(const CompleteSequenceType &sequence_type, const std::string &type_name)

Register sequence TypeObject into TypeObjectRegistry. CompleteSequenceType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **sequence_type** – **[in]** CompleteSequenceType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_array_type_object(const CompleteArrayType &array_type, const std::string &type_name)

Register array TypeObject into TypeObjectRegistry. CompleteArrayType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **array_type** – **[in]** CompleteArrayType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_map_type_object(const CompleteMapType &map_type, const std::string &type_name)

Register map TypeObject into TypeObjectRegistry. CompleteMapType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **map_type** – **[in]** CompleteMapType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_enumerated_type_object(const CompleteEnumeratedType &enumerated_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register enumeration TypeObject into TypeObjectRegistry. CompleteEnumeratedType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **enumerated_type** – **[in]** CompleteEnumeratedType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteEnumeratedType just registered and the generated MinimalEnumeratedType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static ReturnCode_t build_and_register_bitmask_type_object(const CompleteBitmaskType &bitmask_type, const std::string &type_name, TypeIdentifierPair &type_ids)

Register bitmask TypeObject into TypeObjectRegistry. CompleteBitmaskType is provided and the minimal TypeObject is constructed from the complete one. 

* **Parameters:**
  * **bitmask_type** – **[in]** CompleteBitmaskType. 
  * **type_name** – **[in]** Name to be registered in the registry. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteBitmaskType just registered and the generated MinimalBitmaskType. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given type is inconsistent (only in Debug build mode). 
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_BAD_PARAMETER if type_name is empty. 

### static const NameHash name_hash(const std::string &name)

Calculate the MD5 hash of the provided name. 

* **Parameters:**
  **name** – **[in]** String which hash is calculated. 
* **Returns:**
  const NameHash Hash of the given string. 

### static void type_object_consistency(const TypeObject &type_object)

Check TypeObject consistency. 

* **Parameters:**
  **type_object** – **[in]** Instance to be checked. 
* **Throws:**
  `eprosima::fastdds::dds::xtypes::InvalidArgumentError` – exception if the given TypeObject is not consistent. 

### static const TypeIdentifier &retrieve_minimal_type_identifier(const TypeIdentifierPair &type_ids, bool &ec)

If one of the TypeIdentifier in TypeIdentifierPair is minimal, returns its reference. 

* **Parameters:**
  * **type_ids** – **[in]** TypeIdentifierPair used to retrieve the TypeIdentifier. 
  * **ec** – **[out]** Returns if there was an error. 
* **Returns:**
  Reference to the minimal TypeIdentifier. 

### static const TypeIdentifier &retrieve_complete_type_identifier(const TypeIdentifierPair &type_ids, bool &ec)

If one of the TypeIdentifier in TypeIdentifierPair is complete, returns its reference. 

* **Parameters:**
  * **type_ids** – **[in]** TypeIdentifierPair used to retrieve the TypeIdentifier. 
  * **ec** – **[out]** Returns if there was an error. 
* **Returns:**
  Reference to the complete TypeIdentifier.
