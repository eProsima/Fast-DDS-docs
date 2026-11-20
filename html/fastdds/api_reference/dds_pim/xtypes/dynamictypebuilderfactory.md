<a id="api-pim-dynamictypebuilderfactory"></a>

<a id="dynamictypebuilderfactory"></a>

# DynamicTypeBuilderFactory

### class DynamicTypeBuilderFactory : public std::enable_shared_from_this<[DynamicTypeBuilderFactory](#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE)>

### Public Functions

### virtual traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type get_primitive_type(TypeKind kind) = 0

Retrieves the cached [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference associated to a given primitive. 

* **Parameters:**
  **kind** – **[in]** Type identifying the primitive type to retrieve. 
* **Returns:**
  [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_type(traits<[TypeDescriptor](typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE)>::ref_type descriptor) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference based on the given [TypeDescriptor](typedescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) state. 

* **Parameters:**
  **descriptor** – **[in]** [TypeDescriptor](typedescriptor.md#classeprosima_1_1fastdds_1_1dds_1_1TypeDescriptor) to be copied. 
* **Returns:**
  New [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_type_copy(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type type) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference based on the given [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference to be used. 
* **Returns:**
  New [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_type_w_type_object(const xtypes::TypeObject &type_object) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference based on the given xtypes::TypeObject instance. 

* **Parameters:**
  **type_object** – **[in]** xtypes::TypeObject instance to be used. 
* **Returns:**
  New [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_string_type(std::uint32_t bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing a bounded string type. 

* **Parameters:**
  **bound** – **[in]** `uint32_t` representing the maximum number of elements that may be stored. If the value is equal to LENGTH_UNLIMITED, the string type shall be considered to be unbounded. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_wstring_type(std::uint32_t bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing a bounded wstring type. 

* **Parameters:**
  **bound** – **[in]** `uint32_t` representing the maximum number of elements that may be stored. If the value is equal to LENGTH_UNLIMITED, the wstring type shall be considered to be unbounded. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_sequence_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type element_type, std::uint32_t bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing a sequence. 

* **Parameters:**
  * **element_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference which becomes the element type 
  * **bound** – **[in]** `uint32_t` representing the maximum number of elements that may be stored. If the value is equal to LENGTH_UNLIMITED, the sequence type shall be considered to be unbounded. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_array_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type element_type, const BoundSeq &bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing an array. 

* **Parameters:**
  * **element_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference which becomes the element type 
  * **bound** – **[in]** `uint32_t` sequence representing the desired dimensions. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_map_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type key_element_type, traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type element_type, std::uint32_t bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing a map. 

* **Parameters:**
  * **key_element_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference which becomes the map’s key type 
  * **element_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference which becomes the map’s value type 
  * **bound** – **[in]** `uint32_t` representing the maximum number of elements that may be stored. If the value is equal to LENGTH_UNLIMITED, the map type shall be considered to be unbounded. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_bitmask_type(std::uint32_t bound) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference representing a bitmask. 

* **Parameters:**
  **bound** – **[in]** `uint32_t` representing the maximum number of elements that may be stored. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual void set_preprocessor(const std::string &preprocessor) = 0

Sets the path to the preprocessor executable to be used when parsing type descriptions. 

* **Parameters:**
  **preprocessor** – **[in]** path to the preprocessor executable. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_type_w_uri(const std::string &document_url, const std::string &type_name, const IncludePathSeq &include_paths) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference by parsing the type description at the given URL. 

* **Parameters:**
  * **document_url** – **[in]** pointing to the url containing the type description, or to a directory whose IDL files are searched as described in [for_each_type_w_uri](#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilderFactory_1ab855d518910e47a0b24ef3e79051ea5e). 
  * **type_name** – **[in]** Fully qualified name of the type to be loaded from the document. 
  * **include_paths** – **[in]** A collection of URLs to directories to be searched for additional type description documents. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual ReturnCode_t for_each_type_w_uri(const std::string &document_url, const IncludePathSeq &include_paths, std::function<bool(traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type)> callback) = 0

Parse an input document and apply a callback on each encountered type. 

When `document_url` is a directory, every IDL file (``.idl extension) in it, including those in any subdirectory, is parsed in path order. Included files are searched in `include_paths` first and then in the directory itself. A declaration that cannot be loaded is left out and the rest of its file still loads. A file is skipped when it fails to parse, or when it declares a type already loaded with a different definition. Types reached through several files are handed to the callback once.

* **Parameters:**
  * **document_url** – **[in]** pointing to the url containing the type description, or to a directory. 
  * **include_paths** – **[in]** A collection of URLs to directories to be searched for additional type description documents. 
  * **callback** – **[in]** function to be called when one of the following declared types are parsed in the document:
    * Structures
    * Unions
    * Enumerations
    * Aliases
    * Bitmasks
    * Bitsets It receives a [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference to the parsed type and returns a boolean value, that represents whether the parsing process should continue (`true` return value) or not. 
* **Returns:**
  RETCODE_OK if the parsing was successful, RETCODE_BAD_PARAMETER if `document_url` does not exist or cannot be read, or an error code otherwise. 

### virtual traits<[DynamicTypeBuilder](dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)>::ref_type create_type_w_document(const std::string &document, const std::string &type_name, const IncludePathSeq &include_paths) = 0

Creates a new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference by parsing the type description contained in the given string. 

**Remark**

Not implemented yet. 

* **Parameters:**
  * **document** – **[in]** containing the type description. 
  * **type_name** – **[in]** Fully qualified name of the type to be loaded from the string. 
  * **include_paths** – **[in]** A collection of URLs to directories to be searched for additional type description documents. 
* **Returns:**
  new [DynamicTypeBuilder](dynamictypebuilder.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilder) reference. Nil reference returned in error case. 

### virtual ReturnCode_t delete_type(traits<[DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)>::ref_type &type) = 0

Resets the internal reference if it is cached. 

* **Parameters:**
  **type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) reference whose internal cached reference to reset. 
* **Return values:**
  **RETCODE_OK** – is always returned. 
* **Returns:**
  standard ReturnCode_t 

### Public Static Functions

### static traits<[DynamicTypeBuilderFactory](#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE)>::ref_type get_instance()

Returns the singleton factory object. 

**Remark**

This method is non thread-safe. 

* **Returns:**
  [DynamicTypeBuilderFactory](#classeprosima_1_1fastdds_1_1dds_1_1DynamicTypeBuilderFactory) reference. 

### static ReturnCode_t delete_instance()

Resets the singleton reference. 

* **Return values:**
  * **RETCODE_BAD_PARAMETER** – if singleton reference is currently nil. 
  * **RETCODE_OK** – otherwise. 
* **Returns:**
  ReturnCode_t
