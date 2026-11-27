<a id="api-pim-entity"></a>

<a id="entity"></a>

# Entity

### class Entity

The [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity) class is the abstract base class for all the objects that support QoS policies, a listener and a status condition. 

Subclassed by [eprosima::fastdds::dds::DomainEntity](domainentity.md#classeprosima_1_1fastdds_1_1dds_1_1DomainEntity), [eprosima::fastdds::dds::DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant)

### Public Functions

### inline Entity(const [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Constructor. 

* **Parameters:**
  **mask** – [StatusMask](status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) (default: all) 

### inline virtual ReturnCode_t enable()

This operation enables the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

* **Returns:**
  RETCODE_OK 

### inline void close()

This operation disables the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity) before closing it. 

### inline const [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &get_status_mask() const

Retrieves the set of relevant statuses for the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

* **Returns:**
  Reference to the [StatusMask](status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) with the relevant statuses set to 1 

### const [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &get_status_changes() const

Retrieves the set of triggered statuses in the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

Triggered statuses are the ones whose value has changed since the last time the application read the status. When the entity is first created or if the entity is not enabled, all communication statuses are in the non-triggered state, so the list returned by the get_status_changes operation will be empty. The list of statuses returned by the get_status_changes operation refers to the status that are triggered on the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity) itself and does not include statuses that apply to contained entities.

* **Returns:**
  const reference to the [StatusMask](status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) with the triggered statuses set to 1 

### inline const InstanceHandle_t &get_instance_handle() const

Retrieves the instance handler that represents the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

* **Returns:**
  Reference to the InstanceHandle 

### inline bool is_enabled() const

Checks if the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity) is enabled. 

* **Returns:**
  true if enabled, false if not 

### inline [StatusCondition](condition/statuscondition.md#_CPPv4N8eprosima7fastdds3dds15StatusConditionE) &get_statuscondition()

Allows access to the [StatusCondition](condition/statuscondition.md#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition) associated with the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

* **Returns:**
  Reference to [StatusCondition](condition/statuscondition.md#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition) object 

### inline const [StatusCondition](condition/statuscondition.md#_CPPv4N8eprosima7fastdds3dds15StatusConditionE) &get_statuscondition() const

Allows access to the [StatusCondition](condition/statuscondition.md#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition) associated with the [Entity](#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

* **Returns:**
  Const Reference to [StatusCondition](condition/statuscondition.md#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition) object
