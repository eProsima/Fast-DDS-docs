<a id="api-pim-readcondition"></a>

<a id="readcondition"></a>

# ReadCondition

### class ReadCondition : public eprosima::fastdds::dds::[Condition](../core/condition/condition.md#_CPPv4N8eprosima7fastdds3dds9ConditionE)

A [Condition](../core/condition/condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) specifically dedicated to read operations and attached to one [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

[ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) objects allow an application to specify the data samples it is interested in (by specifying the desired sample_states, view_states, and instance_states). The condition will only be triggered when suitable information is available. They are to be used in conjunction with a [WaitSet](../core/condition/waitset.md#classeprosima_1_1fastdds_1_1dds_1_1WaitSet) as normal conditions. More than one [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) may be attached to the same [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). 

### Public Functions

### virtual bool get_trigger_value() const noexcept override

Retrieves the trigger_value of the [Condition](../core/condition/condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition). 

* **Returns:**
  true if trigger_value is set to ‘true’, ‘false’ otherwise 

### [DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*get_datareader() const noexcept

Retrieves the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) associated with the [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition). 

Note that there is exactly one [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) associated with each [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition).

* **Returns:**
  pointer to the [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) associated with this [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition). 

### SampleStateMask get_sample_state_mask() const noexcept

Retrieves the set of sample_states taken into account to determine the trigger_value of this condition. 

* **Returns:**
  the sample_states specified when the [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) was created. 

### ViewStateMask get_view_state_mask() const noexcept

Retrieves the set of view_states taken into account to determine the trigger_value of this condition. 

* **Returns:**
  the view_states specified when the [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) was created. 

### InstanceStateMask get_instance_state_mask() const noexcept

Retrieves the set of instance_states taken into account to determine the trigger_value of this condition. 

* **Returns:**
  the instance_states specified when the [ReadCondition](#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition) was created.
