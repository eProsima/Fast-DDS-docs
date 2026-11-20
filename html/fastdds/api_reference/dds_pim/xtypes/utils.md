<a id="api-pim-xtypes-utils"></a>

<a id="utils"></a>

# Utils

**[Pro]** The overloads of `json_serialize` and `json_deserialize` that take a `JsonSerializeOptions` or a
`JsonDeserializeOptions` argument are only available in *Fast DDS* **[Pro]**.

### ReturnCode_t eprosima::fastdds::dds::idl_serialize(const [DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)::\_ref_type &dynamic_type, std::ostream &output) noexcept

Serializes a [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) into its IDL representation. 

* **Parameters:**
  * **dynamic_type** – **[in]** The [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) to serialize. 
  * **output** – **[inout]** `std::ostream` reference containing the IDL representation. 
* **Return values:**
  **RETCODE_OK** – when serialization fully succeeds, and inner (member serialization) failing code otherwise. 

### ReturnCode_t eprosima::fastdds::dds::json_serialize(const [DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)::\_ref_type &data, DynamicDataJsonFormat format, std::ostream &output) noexcept

Serializes a [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) into a JSON object, which is then dumped into an `std::ostream`. 

Equivalent to calling the JsonSerializeOptions overload with default options and the given `format`. 

* **Parameters:**
  * **data** – **[in]** [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference to be serialized. 
  * **format** – **[in]** DynamicDataJsonFormat JSON serialization format. 
  * **output** – **[inout]** `std::ostream` reference where the JSON object is dumped. 
* **Return values:**
  **RETCODE_OK** – when serialization fully succeeds, and inner (member serialization) failing code otherwise. 

### ReturnCode_t eprosima::fastdds::dds::json_serialize(const [DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)::\_ref_type &data, const JsonSerializeOptions &options, std::ostream &output) noexcept

 **[Pro]**  Serializes a [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) into a JSON object, which is then dumped into an `std::ostream`. See JsonSerializeOptions for the available controls (JSON style, collection truncation). 

* **Parameters:**
  * **data** – **[in]** [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference to be serialized. 
  * **options** – **[in]** JsonSerializeOptions controlling format and collection truncation. 
  * **output** – **[inout]** `std::ostream` reference where the JSON object is dumped. 
* **Return values:**
  **RETCODE_OK** – when serialization fully succeeds, and inner (member serialization) failing code otherwise. 

### ReturnCode_t eprosima::fastdds::dds::json_deserialize(const std::string &input, const [DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)::\_ref_type &dynamic_type, DynamicDataJsonFormat format, [DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)::\_ref_type &data) noexcept

Deserializes a JSON object string into a [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData). 

* **Parameters:**
  * **input** – **[in]** JSON object string to be deserialized. 
  * **dynamic_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) corresponding to the [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) where the JSON string is deserialized. 
  * **format** – **[in]** DynamicDataJsonFormat JSON serialization format. 
  * **data** – **[inout]** [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference where the JSON string is deserialized. Must be null. 
* **Return values:**
  **RETCODE_OK** – when deserialization fully succeeds, RETCODE_BAD_PARAMETER when parsing fails or preconditions are not met, and inner (member deserialization) failing code otherwise. 

### ReturnCode_t eprosima::fastdds::dds::json_deserialize(const std::string &input, const [DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)::\_ref_type &dynamic_type, const JsonDeserializeOptions &options, [DynamicData](dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE)::\_ref_type &data) noexcept

 **[Pro]**  Deserializes a JSON object string into a [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData).

Documents produced by [json_serialize](#namespaceeprosima_1_1fastdds_1_1dds_1aeb49c79edc4a834a8223e4fcc8af1784) with a non-zero `max_collection_items` that triggered truncation cannot be deserialized back: truncation replaces a JSON array with a JSON string label, which violates the schema expected for sequence/array members. Such inputs are rejected with `RETCODE_BAD_PARAMETER`.

* **Parameters:**
  * **input** – **[in]** JSON object string to be deserialized. 
  * **dynamic_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) corresponding to the [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) where the JSON string is deserialized. 
  * **options** – **[in]** JsonDeserializeOptions, only the JSON mapping is used. 
  * **data** – **[inout]** [DynamicData](dynamicdata.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicData) reference where the JSON string is deserialized. Must be null. 
* **Return values:**
  **RETCODE_OK** – when deserialization fully succeeds, RETCODE_BAD_PARAMETER when parsing fails or preconditions are not met (including truncated-collection labels in `input`), and inner (member deserialization) failing code otherwise.
