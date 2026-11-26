<a id="api-pim-dynamicdata"></a>

<a id="dynamicdata"></a>

# DynamicData

### class DynamicData : public std::enable_shared_from_this<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>

### Public Functions

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type() = 0

Retrieve the [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference associated to this [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData). 

* **Returns:**
  Non-nil [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference 

### virtual ReturnCode_t get_descriptor(traits<[MemberDescriptor](memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)>::ref_type &value, MemberId id) = 0

Retrieves the [MemberDescriptor](memberdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) associated to a member. 

* **Parameters:**
  * **value** – **[inout]** Non-nil [MemberDescriptor](memberdescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1MemberDescriptor) reference where the information is copied. 
  * **id** – **[in]** Identifier of the member to be retrieved. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil or member identifier is not found. 

### virtual bool equals(traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type other) = 0

Compares two [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData), equality requires: 

* Their respective type definitions are equal
* All contained values are equal and occur in the same order
* If the samples’ type is an aggregated type, previous rule shall be amended as follows:
  1. Members shall be compared without regard to their order. 

* **Parameters:**
  **other** – **[in]** [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference to compare to 
* **Returns:**
  `true` on equality 

### virtual MemberId get_member_id_by_name(const ObjectName &name) = 0

Queries MemberId by name. 

The query result depends on the type of the sample. Only next types support accessing by name. 
* Aggregated type. 
* Map type. 
* Bitmask type. 

* **Parameters:**
  **name** – **[in]** string 
* **Returns:**
  MemberId or MEMBER_ID_INVALID on failure 

### virtual MemberId get_member_id_at_index(std::uint32_t index) = 0

Queries MemberId by index The query result depends on the type of the sample. 

Only next types support accessing by index. 
* Aggregated type. 
* Sequence type. 
* String type. 
* Map type. 
* Array type. 
* Bitmask type. 

* **Parameters:**
  **index** – **[in]** Index. 
* **Returns:**
  MemberId or MEMBER_ID_INVALID on failure 

### virtual std::uint32_t get_item_count() = 0

Provides the **item** **count** of the data and depends on the type of object: 

* If the object is of a collection type, returns the number of elements currently in the collection. In the case of an array type, this value will always be equal to the product of the bounds of all array dimensions. 
* If the object is of a bitmask type, return the number of named flags that are currently set in the bitmask. 
* If the object is of a structure or annotation type, return the number of members in the object. This value may be different than the number of members in the corresponding [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType). 
* If the object is of a union type, return the number of members in the object. This number will be two if the discriminator value selects a member and one otherwise. 
* if the object is of a primitive or enumerated type, it is atomic: return one. 
* if the object is of an alias type, return the value appropriate for the alias base type. 

* **Returns:**
  count as defined above 

### virtual ReturnCode_t clear_all_values() = 0

Clear all members associated to the object. 

* **Return values:**
  **RETCODE_OK** – when the cleaning was successful. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t clear_nonkey_values() = 0

Clear all members not associated to the key. 

* **Return values:**
  **RETCODE_OK** – when the cleaning was successful. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t clear_value(MemberId id) = 0

Clear a member. 

The meaning of “clearing” depends on the type of the sample: 
* If aggregated type, set it to its default value. 
* If variable-length collection type, remove the indicated element, shifting any subsequence elements to the next-lowest index. 
* If array type, set the indicated element to its default value. 
* If bitmask type, clear the indicated bit. 
* If enumerated type, set it to the first value of the enumerated type. 
* If primitive type, set it to its default value. 

* **Parameters:**
  **id** – **[in]** Identifier of the member to purge 
* **Return values:**
  * **RETCODE_OK** – when the cleaning was successful. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid. 
* **Returns:**
  ReturnCode_t

### virtual traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type loan_value(MemberId id) = 0

**Loans** a [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference within the sample 

**Remark**

This loan shall be returned by the [DynamicData::return_loaned_value](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData_1a9251d9e52bcb0a48c160a84a340fb3eb) operation 

* **Parameters:**
  **id** – **[in]** identifier of the object to retrieve 
* **Returns:**
  [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference loaned or **nil** on outstanding loaned data 

### virtual ReturnCode_t return_loaned_value(traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type value) = 0

Returns a loan retrieved using [DynamicData::loan_value](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData_1a0d0085bc1e792b70c0d1dab51ebe0c79). 

* **Parameters:**
  **value** – **[in]** [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference previously loaned 
* **Return values:**
  * **RETCODE_OK** – when the loan was returned successfully. 
  * **RETCODE_PRECONDITION_NOT_MET** – when the loan is invalid. 

### virtual traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type clone() = 0

Creates and returns a new data sample with the same contents as this one. 

A comparison of this object and the clone using equals immediately following this call will return `true`. 

* **Returns:**
  [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference 

### virtual ReturnCode_t get_int32_value(std::int32_t &value, MemberId id) = 0

Retrieves an **int32** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **int32** to populate 
  * **id** – **[in]** identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int32_value(MemberId id, std::int32_t value) = 0

Sets an **int32** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **int32** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint32_value(std::uint32_t &value, MemberId id) = 0

Retrieves an **uint32** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **uint32** to populate 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint32_value(MemberId id, std::uint32_t value) = 0

Sets an **uint32** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **uint32** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int8_value(std::int8_t &value, MemberId id) = 0

Retrieves an **int8** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **int8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int8_value(MemberId id, std::int8_t value) = 0

Sets an **int8** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **int8** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint8_value(std::uint8_t &value, MemberId id) = 0

Retrieves an **uint8** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **uint8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint8_value(MemberId id, std::uint8_t value) = 0

Sets an **uint8** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **uint8** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int16_value(std::int16_t &value, MemberId id) = 0

Retrieves an **int16** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **int16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int16_value(MemberId id, std::int16_t value) = 0

Sets an **int16** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **int16** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint16_value(std::uint16_t &value, MemberId id) = 0

Retrieves an **uint16** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **uint16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint16_value(MemberId id, std::uint16_t value) = 0

Sets an **uint16** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **uint16** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int64_value(std::int64_t &value, MemberId id) = 0

Retrieves an **int64** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **int64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int64_value(MemberId id, std::int64_t value) = 0

Sets an **int64** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **int64** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **int64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint64_value(std::uint64_t &value, MemberId id) = 0

Retrieves an **uint64** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **uint64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint64_value(MemberId id, std::uint64_t value) = 0

Sets an **uint64** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **uint64** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **uint64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float32_value(float &value, MemberId id) = 0

Retrieves an **float32** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **float32** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float32_value(MemberId id, float value) = 0

Sets an **float32** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **float32** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float64_value(double &value, MemberId id) = 0

Retrieves an **float64** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **float64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float64_value(MemberId id, double value) = 0

Sets an **float64** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **float64** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float128_value(long double &value, MemberId id) = 0

Retrieves an **float128** value associated to an identifier. 

**Remark**

Only available on platforms supporting long double 

* **Parameters:**
  * **value** – **[inout]** **float128** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float128**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float128_value(MemberId id, long double value) = 0

Sets an **float128** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **float128** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **float128**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_char8_value(char &value, MemberId id) = 0

Retrieves an **char8** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **char8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **char8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_char8_value(MemberId id, char value) = 0

Sets an **char8** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **char8** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **char8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_char16_value(wchar_t &value, MemberId id) = 0

Retrieves an **char16** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **char16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **char16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_char16_value(MemberId id, wchar_t value) = 0

Sets an **char16** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **char16** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **char16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_byte_value(eprosima::fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) &value, MemberId id) = 0

Retrieves an **byte** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **byte** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **byte**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_byte_value(MemberId id, eprosima::fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) value) = 0

Sets an **byte** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **byte** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **byte**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_boolean_value(bool &value, MemberId id) = 0

Retrieves an **bool** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to query. 
  * **value** – **[inout]** **bool** to populate. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **bool**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_boolean_value(MemberId id, bool value) = 0

Sets an **bool** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** identifier of the member to set. 
  * **value** – **[in]** **bool** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **bool**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_string_value(std::string &value, MemberId id) = 0

Retrieves an **string** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **string** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **string**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_string_value(MemberId id, const std::string &value) = 0

Sets an **string** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **string** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **string** or the string length is greater than the string bound. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_wstring_value(std::wstring &value, MemberId id) = 0

Retrieves an **wstring** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **wstring** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **wstring**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_wstring_value(MemberId id, const std::wstring &value) = 0

Sets an **wstring** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** **wstring** to set. 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **wstring** or the string length is greater than the string bound. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_complex_value(traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type &value, MemberId id) = 0

Retrieves a **complex** value associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference to populate 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to **complex**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_complex_value(MemberId id, traits<[DynamicData](#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)>::ref_type value) = 0

Sets a **complex** value associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** [DynamicData](#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the value reference is nil or MemberId is invalid or the member type is not promotable to **complex**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int32_values(Int32Seq &value, MemberId id) = 0

Retrieves a sequence of **int32** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **int32** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int32_values(MemberId id, const Int32Seq &value) = 0

Sets a sequence of **int32** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **int32** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint32_values(UInt32Seq &value, MemberId id) = 0

Retrieves a sequence of **uint32** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **uint32** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint32_values(MemberId id, const UInt32Seq &value) = 0

Sets a sequence of **uint32** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **uint32** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int8_values(Int8Seq &value, MemberId id) = 0

Retrieves a sequence of **int8** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **int8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int8_values(MemberId id, const Int8Seq &value) = 0

Sets a sequence of **int8** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **int8** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint8_values(UInt8Seq &value, MemberId id) = 0

Retrieves a sequence of **uint8** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **uint8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint8_values(MemberId id, const UInt8Seq &value) = 0

Sets a sequence of **uint8** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **uint8** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int16_values(Int16Seq &value, MemberId id) = 0

Retrieves a sequence of **int16** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **int16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int16_values(MemberId id, const Int16Seq &value) = 0

Sets a sequence of **int16** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **int16** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint16_values(UInt16Seq &value, MemberId id) = 0

Retrieves a sequence of **uint16** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **uint16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint16_values(MemberId id, const UInt16Seq &value) = 0

Sets a sequence of **uint16** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **uint16** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_int64_values(Int64Seq &value, MemberId id) = 0

Retrieves a sequence of **int64** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **int64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_int64_values(MemberId id, const Int64Seq &value) = 0

Sets a sequence of **int64** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **int64** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **int64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_uint64_values(UInt64Seq &value, MemberId id) = 0

Retrieves a sequence of **uint64** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **uint64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_uint64_values(MemberId id, const UInt64Seq &value) = 0

Sets a sequence of **uint64** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **uint64** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **uint64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float32_values(Float32Seq &value, MemberId id) = 0

Retrieves a sequence of **float32** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **float32** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float32_values(MemberId id, const Float32Seq &value) = 0

Sets a sequence of **float32** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **float32** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float32**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float64_values(Float64Seq &value, MemberId id) = 0

Retrieves a sequence of **float64** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **float64** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float64_values(MemberId id, const Float64Seq &value) = 0

Sets a sequence of **float64** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **float64** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float64**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_float128_values(Float128Seq &value, MemberId id) = 0

Retrieves a sequence of **float128** values associated to an identifier. 

**Remark**

Only available on platforms supporting long double 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **float128** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float128**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_float128_values(MemberId id, const Float128Seq &value) = 0

Sets a sequence of **float128** values associated to an identifier. 

**Remark**

Only available on platforms supporting long double 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **float128** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **float128**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_char8_values(CharSeq &value, MemberId id) = 0

Retrieves a sequence of **char8** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **char8** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **char8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_char8_values(MemberId id, const CharSeq &value) = 0

Sets a sequence of **char8** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **char8** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **char8**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_char16_values(WcharSeq &value, MemberId id) = 0

Retrieves a sequence of **char16** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **char16** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **char16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_char16_values(MemberId id, const WcharSeq &value) = 0

Sets a sequence of **char16** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **char16** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **char16**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_byte_values(ByteSeq &value, MemberId id) = 0

Retrieves a sequence of **byte** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **byte** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **byte**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_byte_values(MemberId id, const ByteSeq &value) = 0

Sets a sequence of **byte** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **byte** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **byte**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_boolean_values(BooleanSeq &value, MemberId id) = 0

Retrieves a sequence of **bool** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **bool** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **bool**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_boolean_values(MemberId id, const BooleanSeq &value) = 0

Sets a sequence of **bool** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **bool** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **bool**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_string_values(StringSeq &value, MemberId id) = 0

Retrieves a sequence of **string** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **string** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **string**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_string_values(MemberId id, const StringSeq &value) = 0

Sets a sequence of **string** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **string** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **string**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t get_wstring_values(WstringSeq &value, MemberId id) = 0

Retrieves a sequence of **wstring** values associated to an identifier. 

* **Parameters:**
  * **value** – **[inout]** **Sequence** of **wstring** to populate. 
  * **id** – **[in]** Identifier of the member to query. 
* **Return values:**
  * **RETCODE_OK** – when the value was retrieved successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **wstring**. 
* **Returns:**
  ReturnCode_t

### virtual ReturnCode_t set_wstring_values(MemberId id, const WstringSeq &value) = 0

Sets a sequence of **wstring** values associated to an identifier. 

* **Parameters:**
  * **id** – **[in]** Identifier of the member to set. 
  * **value** – **[in]** Sequence of **wstring** to set 
* **Return values:**
  * **RETCODE_OK** – when the value was set successfully. 
  * **RETCODE_BAD_PARAMETER** – when the MemberId is invalid or the member type is not promotable to sequence of **wstring**. 
* **Returns:**
  ReturnCode_t
