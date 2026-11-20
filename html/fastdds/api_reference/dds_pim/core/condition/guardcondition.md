<a id="api-pim-guardcondition"></a>

<a id="guardcondition"></a>

# GuardCondition

### class GuardCondition : public eprosima::fastdds::dds::[Condition](condition.md#_CPPv4N8eprosima7fastdds3dds9ConditionE)

The [GuardCondition](#classeprosima_1_1fastdds_1_1dds_1_1GuardCondition) class is a specific [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) whose trigger_value is completely under the control of the application. 

The purpose of the [GuardCondition](#classeprosima_1_1fastdds_1_1dds_1_1GuardCondition) is to provide the means for the application to manually wakeup a [WaitSet](waitset.md#classeprosima_1_1fastdds_1_1dds_1_1WaitSet). This is accomplished by attaching the [GuardCondition](#classeprosima_1_1fastdds_1_1dds_1_1GuardCondition) to the [WaitSet](waitset.md#classeprosima_1_1fastdds_1_1dds_1_1WaitSet) and then setting the trigger_value by means of the set_trigger_value operation. 

### Public Functions

### virtual bool get_trigger_value() const override

Retrieves the trigger_value of the [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition). 

* **Returns:**
  true if trigger_value is set to ‘true’, ‘false’ otherwise 

### ReturnCode_t set_trigger_value(bool value)

Set the trigger_value. 

* **Parameters:**
  **value** – new value for trigger 
* **Returns:**
  RETURN_OK
