<a id="api-pim-typeobjectregistry"></a>

<a id="typeobjectregistry"></a>

# TypeObjectRegistry

### class ITypeObjectRegistry

### Public Functions

### virtual ReturnCode_t register_type_object(const std::string &type_name, const CompleteTypeObject &complete_type_object, TypeIdentifierPair &type_ids) = 0

Register a local TypeObject. The MinimalTypeObject is generated from the CompleteTypeObject, and both are registered into the registry with the corresponding TypeIdentifiers and TypeObject serialized sizes. 

* **Parameters:**
  * **type_name** – **[in]** Name of the type being registered. 
  * **complete_type_object** – **[in]** CompleteTypeObject related to the given type name. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the CompleteTypeObject just registered and the generated MinimalTypeObject. 
* **Pre:**
  type_name must not be empty. 
* **Pre:**
  complete_type_object must be consistent (only checked in Debug build mode).
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeObject registered with the given type_name. RETCODE_PRECONDITION_NOT_MET if the given type_name is empty or if the type_object is inconsistent. 

### virtual ReturnCode_t register_type_object(const TypeObject &type_object, TypeIdentifierPair &type_ids) = 0

Register a remote TypeObject. This auxiliary method might register only the minimal TypeObject and TypeIdentifier or register both TypeObjects constructing the minimal from the complete TypeObject information. TypeObject consistency is not checked in this method as the order of the dependencies received by the TypeLookupService is not guaranteed. The consistency is checked by the TypeLookupService after all dependencies are registered. 

* **Parameters:**
  * **type_object** – **[in]** Related TypeObject being registered. 
  * **type_ids** – **[inout]** Returns the registered TypeIdentifier. TypeIdentifierPair::type_identifier1 might be TK_NONE. In other case this function will check it is consistence with the provided TypeObject. 
* **Pre:**
  TypeIdentifierPair::type_identifier1 discriminator must match TypeObject discriminator or be TK_NONE. TypeIdentifierPair::type_identifier1 consistency is only checked in Debug build mode.
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered. RETCODE_PRECONDITION_NOT_MET if the discriminators differ. RETCODE_PRECONDITION_NOT_MET if the TypeIdentifier is not consistent with the given TypeObject. 

### virtual ReturnCode_t register_typeobject_w_dynamic_type(const [DynamicType](dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE)::\_ref_type &dynamic_type, TypeIdentifierPair &type_ids) = 0

Register [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) TypeObject. 

* **Parameters:**
  * **dynamic_type** – **[in]** [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) to be registered. 
  * **type_ids** – **[out]** TypeIdentifierPair corresponding to the registered [DynamicType](dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) TypeObject. 
* **Returns:**
  ReturnCode_t RETCODE_OK always. 

### virtual ReturnCode_t register_type_identifier(const std::string &type_name, TypeIdentifierPair &type_identifier) = 0

Register an indirect hash TypeIdentifier. 

* **Parameters:**
  * **type_name** – **[in]** Name of the type being registered. 
  * **type_identifier** – **[inout]** TypeIdentifierPair related to the given type name. It must be set in TypeIdentifierPair::type_identifier1. At the end this object is filled with both TypeIdentifiers. 
* **Pre:**
  TypeIdentifier must not be a direct hash TypeIdentifier. 
* **Pre:**
  TypeIdentifier must be consistent (only checked in Debug build mode). 
* **Pre:**
  type_name must not be empty.
* **Returns:**
  ReturnCode_t RETCODE_OK if correctly registered in TypeObjectRegistry. RETCODE_BAD_PARAMETER if there is already another different TypeIdentifier registered with the given type_name. RETCODE_PRECONDITION_NOT_MET if the given TypeIdentifier is inconsistent or a direct hash TypeIdentifier or if the given type_name is empty. 

### virtual ReturnCode_t get_type_objects(const std::string &type_name, TypeObjectPair &type_objects) = 0

Get the TypeObjects related to the given type name. 

* **Parameters:**
  * **type_name** – **[in]** Name of the type being queried. 
  * **type_objects** – **[out]** Both complete and minimal TypeObjects related with the given type_name. 
* **Pre:**
  type_name must not be empty.
* **Returns:**
  ReturnCode_t RETCODE_OK if the TypeObjects are found in the registry. RETCODE_NO_DATA if the given type_name has not been registered. RETCODE_BAD_PARAMETER if the type_name correspond to a indirect hash TypeIdentifier. RETCODE_PRECONDITION_NOT_MET if the type_name is empty. 

### virtual ReturnCode_t get_type_identifiers(const std::string &type_name, TypeIdentifierPair &type_identifiers) = 0

Get the TypeIdentifiers related to the given type name. 

* **Parameters:**
  * **type_name** – **[in]** Name of the type being queried. 
  * **type_identifiers** – **[out]** For direct hash TypeIdentifiers, both minimal and complete TypeIdentifiers are returned. For indirect hash TypeIdentifiers, only the corresponding TypeIdentifier is returned 
* **Pre:**
  type_name must not be empty.
* **Returns:**
  ReturnCode_t RETCODE_OK if the TypeIdentifiers are found in the registry. RETCODE_NO_DATA if the type_name has not been registered. RETCODE_PRECONDITION_NOT_MET if the type_name is empty. 

### virtual ReturnCode_t get_type_object(const TypeIdentifier &type_identifier, TypeObject &type_object) = 0

Get the TypeObject related to the given TypeIdentifier. 

* **Parameters:**
  * **type_identifier** – **[in]** TypeIdentifier being queried. 
  * **type_object** – **[out]** TypeObject related with the given TypeIdentifier. 
* **Pre:**
  TypeIdentifier must be a direct hash TypeIdentifier.
* **Returns:**
  ReturnCode_t RETCODE_OK if the TypeObject is found within the registry. RETCODE_NO_DATA if the given TypeIdentifier is not found in the registry. RETCODE_PRECONDITION_NOT_MET if the TypeIdentifier is not a direct hash. 

### virtual ReturnCode_t get_complete_type_object(const TypeIdentifierPair &type_identifiers, CompleteTypeObject &type_object) = 0

Get the CompleteTypeObject related to the given type identifiers. 

* **Parameters:**
  * **type_identifiers** – **[in]** Identifiers of the type being queried. 
  * **type_object** – **[out]** CompleteTypeObject related with the given identifiers. 
* **Returns:**
  ReturnCode_t RETCODE_OK if the CompleteTypeObject are found in the registry. RETCODE_NO_DATA if the given type_identifiers has not been registered. RETCODE_BAD_PARAMETER if the type_identifiers correspond to an indirect hash. 

### virtual ReturnCode_t get_type_information(const TypeIdentifierPair &type_ids, TypeInformation &type_information, bool with_dependencies = false) = 0

Get the TypeInformation related to a specific type_name. 

* **Parameters:**
  * **type_ids** – **[in]** TypeIdentifierPair which type information is queried. 
  * **type_information** – **[out]** Related TypeInformation for the given TypeIdentifier. 
  * **with_dependencies** – **[in]** 
* **Pre:**
  type_ids must not be empty.
* **Returns:**
  ReturnCode_t RETCODE_OK if the type_ids are found within the registry. RETCODE_NO_DATA if the given type_ids is not found. RETCODE_BAD_PARAMETER if the given TypeIdentifier corresponds to a indirect hash TypeIdentifier. RETCODE_PRECONDITION_NOT_MET if any type_ids is empty.
