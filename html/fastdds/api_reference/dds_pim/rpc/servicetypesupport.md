<a id="api-pim-servicetypesupport"></a>

<a id="servicetypesupport"></a>

# ServiceTypeSupport

### class ServiceTypeSupport

### Public Functions

### ServiceTypeSupport() noexcept = default

Constructor. 

### ServiceTypeSupport(const [ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupport18ServiceTypeSupportERK18ServiceTypeSupport) &service_type) noexcept = default

Copy Constructor. 

* **Parameters:**
  **service_type** – Another instance of [ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport)

### ServiceTypeSupport([ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupport18ServiceTypeSupportERR18ServiceTypeSupport) &&service_type) noexcept = default

Move Constructor. 

* **Parameters:**
  **service_type** – Another instance of [ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport)

### [ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) &operator=(const [ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) &service_type) noexcept = default

Copy Assignment. 

* **Parameters:**
  **service_type** – Another instance of [ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport)

### [ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) &operator=([ServiceTypeSupport](#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) &&service_type) noexcept = default

Move Assignment. 

* **Parameters:**
  **service_type** – Another instance of [ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport)

### inline ServiceTypeSupport([TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) request_type, [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) reply_type)

[ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport) constructor that receives two [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) objects (Request + reply TypeSupports) 

* **Parameters:**
  * **request_type** – [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) of the request type 
  * **reply_type** – [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) of the reply type 

### virtual ~ServiceTypeSupport() = default

Destructor. 

### virtual ReturnCode_t register_service_type([DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*participant, std::string service_type_name) const

Registers the service type on a participant. 

* **Parameters:**
  * **participant** – [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) where the service type is going to be registered 
  * **service_type_name** – Name of the service type to register 
* **Returns:**
  RETCODE_BAD_PARAMETER if the service name is empty, RETCODE_PRECONDITION_NOT_MET if there is another service with the same name registered on the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) and RETCODE_OK if it is registered correctly 

### inline const [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) request_type() const

Returns the [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) of the request type. 

### inline const [TypeSupport](../topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) reply_type() const

Returns the [TypeSupport](../topic/typesupport.md#classeprosima_1_1fastdds_1_1dds_1_1TypeSupport) of the reply type. 

### inline bool empty_types() const

Check if the [ServiceTypeSupport](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1ServiceTypeSupport) object contains empty request/reply types.
