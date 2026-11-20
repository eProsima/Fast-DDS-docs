<a id="api-pim-condition"></a>

<a id="condition"></a>

# Condition

### class Condition

The [Condition](#classeprosima_1_1fastdds_1_1dds_1_1Condition) class is the root base class for all the conditions that may be attached to a [WaitSet](waitset.md#classeprosima_1_1fastdds_1_1dds_1_1WaitSet). 

Subclassed by [eprosima::fastdds::dds::GuardCondition](guardcondition.md#classeprosima_1_1fastdds_1_1dds_1_1GuardCondition), [eprosima::fastdds::dds::ReadCondition](../../subscriber/readcondition.md#classeprosima_1_1fastdds_1_1dds_1_1ReadCondition), [eprosima::fastdds::dds::StatusCondition](statuscondition.md#classeprosima_1_1fastdds_1_1dds_1_1StatusCondition)

### Public Functions

### inline virtual bool get_trigger_value() const

Retrieves the trigger_value of the [Condition](#classeprosima_1_1fastdds_1_1dds_1_1Condition). 

* **Returns:**
  true if trigger_value is set to ‘true’, ‘false’ otherwise
