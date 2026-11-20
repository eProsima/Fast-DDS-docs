<a id="guid-t-operators"></a>

# GUID_t Operators

### inline bool eprosima::fastdds::rtps::operator==(const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g1, const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g2)

GUID comparison operator 

* **Parameters:**
  * **g1** – First GUID to compare 
  * **g2** – Second GUID to compare 
* **Returns:**
  True if equal 

### inline bool eprosima::fastdds::rtps::operator!=(const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g1, const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g2)

GUID not equal operator 

* **Parameters:**
  * **g1** – First GUID to compare 
  * **g2** – Second GUID to compare 
* **Returns:**
  True if not equal 

### inline bool eprosima::fastdds::rtps::operator<(const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g1, const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &g2)

### inline std::ostream &eprosima::fastdds::rtps::operator<<(std::ostream &output, const [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid)

Stream operator, prints a GUID. 

* **Parameters:**
  * **output** – Output stream. 
  * **guid** – [GUID_t](GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) to print. 
* **Returns:**
  Stream operator. 

### inline std::istream &eprosima::fastdds::rtps::operator>>(std::istream &input, [GUID_t](GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid)

Stream operator, retrieves a GUID. 

* **Parameters:**
  * **input** – Input stream. 
  * **guid** – [GUID_t](GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) to print. 
* **Returns:**
  Stream operator.
