<a id="entityid-t-operators"></a>

# EntityId_t Operators

### inline bool eprosima::fastdds::rtps::operator==([EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &id1, const std::uint32_t id2)

Entity Id comparison operator 

* **Parameters:**
  * **id1** – EntityId to compare 
  * **id2** – ID prefix to compare 
* **Returns:**
  True if equal 

### inline bool eprosima::fastdds::rtps::operator==(const [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &id1, const [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &id2)

Entity Id comparison operator 

* **Parameters:**
  * **id1** – First EntityId to compare 
  * **id2** – Second EntityId to compare 
* **Returns:**
  True if equal 

### inline bool eprosima::fastdds::rtps::operator!=(const [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &id1, const [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &id2)

Entity Id not equal operator 

* **Parameters:**
  * **id1** – First EntityId to compare 
  * **id2** – Second EntityId to compare 
* **Returns:**
  True if not equal 

### inline std::ostream &eprosima::fastdds::rtps::operator<<(std::ostream &output, const [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &enI)

### inline std::istream &eprosima::fastdds::rtps::operator>>(std::istream &input, [EntityId_t](EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &enP)
