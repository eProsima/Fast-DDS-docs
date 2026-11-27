<a id="api-pim-statuscondition"></a>

<a id="statuscondition"></a>

# StatusCondition

### class StatusCondition : public eprosima::fastdds::dds::[Condition](condition.md#_CPPv4N8eprosima7fastdds3dds9ConditionE)

The [StatusCondition](#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition) class is a specific [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) that is associated with each [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity). 

### Public Functions

### virtual bool get_trigger_value() const override

Retrieves the trigger_value of the [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition). 

* **Returns:**
  true if trigger_value is set to ‘true’, ‘false’ otherwise 

### ReturnCode_t set_enabled_statuses(const [StatusMask](../status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Defines the list of communication statuses that are taken into account to determine the trigger_value. 

* **Parameters:**
  **mask** – defines the mask for the status 
* **Returns:**
  RETCODE_OK with everything ok, error code otherwise 

### const [StatusMask](../status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &get_enabled_statuses() const

Retrieves the list of communication statuses that are taken into account to determine the trigger_value. 

* **Returns:**
  Status set or default status if it has not been set 

### [Entity](../entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE) \*get_entity() const

Returns the [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) associated. 

* **Returns:**
  [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity)
