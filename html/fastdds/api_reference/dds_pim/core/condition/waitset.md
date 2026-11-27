<a id="api-pim-waitset"></a>

<a id="wait-set"></a>

# Wait-set

### class WaitSet

The [WaitSet](#classeprosima_1_1fastdds_1_1dds_1_1WaitSet) class allows an application to wait until one or more of the attached [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) objects has a trigger_value of TRUE or until timeout expires. 

### Public Functions

### ReturnCode_t attach_condition(const [Condition](condition.md#_CPPv4N8eprosima7fastdds3dds9ConditionE) &cond)

Attaches a [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) to the Wait Set. 

* **Parameters:**
  **cond** – [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition)
* **Returns:**
  RETCODE_OK if attached correctly, error code otherwise 

### ReturnCode_t detach_condition(const [Condition](condition.md#_CPPv4N8eprosima7fastdds3dds9ConditionE) &cond)

Detaches a [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition) from the [WaitSet](#classeprosima_1_1fastdds_1_1dds_1_1WaitSet). 

* **Parameters:**
  **cond** – [Condition](condition.md#classeprosima_1_1fastdds_1_1dds_1_1Condition)
* **Returns:**
  RETCODE_OK if detached correctly, PRECONDITION_NOT_MET if condition was not attached 

### ReturnCode_t wait([ConditionSeq](conditionseq.md#_CPPv4N8eprosima7fastdds3dds12ConditionSeqE) &active_conditions, const dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) timeout) const

Allows an application thread to wait for the occurrence of certain conditions. If none of the conditions attached to the [WaitSet](#classeprosima_1_1fastdds_1_1dds_1_1WaitSet) have a trigger_value of true, the wait operation will block suspending the calling thread. 

* **Parameters:**
  * **active_conditions** – Reference to the collection of conditions which trigger_value are true 
  * **timeout** – Maximum time of the wait 
* **Returns:**
  RETCODE_OK if everything correct, PRECONDITION_NOT_MET if [WaitSet](#classeprosima_1_1fastdds_1_1dds_1_1WaitSet) already waiting, TIMEOUT if maximum time expired, error code otherwise 

### ReturnCode_t get_conditions([ConditionSeq](conditionseq.md#_CPPv4N8eprosima7fastdds3dds12ConditionSeqE) &attached_conditions) const

Retrieves the list of attached conditions. 

* **Parameters:**
  **attached_conditions** – Reference to the collection of attached conditions 
* **Returns:**
  RETCODE_OK if everything correct, error code otherwise
