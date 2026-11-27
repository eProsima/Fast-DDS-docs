<a id="instancehandle-t-operators"></a>

# InstanceHandle_t Operators

### inline bool eprosima::fastdds::rtps::operator==(const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle1, const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle2) noexcept

Comparison operator 

* **Parameters:**
  * **ihandle1** – First [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare 
  * **ihandle2** – Second [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare 
* **Returns:**
  True if equal 

### inline bool eprosima::fastdds::rtps::operator!=(const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle1, const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle2) noexcept

Comparison operator. 

* **Parameters:**
  * **ihandle1** – First [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare 
  * **ihandle2** – Second [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare 
* **Returns:**
  True if not equal 

### inline bool eprosima::fastdds::rtps::operator<(const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &h1, const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &h2) noexcept

Comparison operator: checks if a [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) is less than another. 

* **Parameters:**
  * **h1** – First [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare. 
  * **h2** – Second [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to compare. 
* **Returns:**
  True if the first [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) is less than the second. 

### inline std::ostream &eprosima::fastdds::rtps::operator<<(std::ostream &output, const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &iHandle)

Stream operator: print an [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t).

* **Parameters:**
  * **output** – Output stream. 
  * **iHandle** – [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to print. 
* **Returns:**
  Stream operator. 

### inline std::istream &eprosima::fastdds::rtps::operator>>(std::istream &input, [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &iHandle)

Stream operator: retrieve an [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t).

* **Parameters:**
  * **input** – Input stream. 
  * **iHandle** – [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) that will receive the input as its new value. 
* **Returns:**
  Stream operator. 

### inline void eprosima::fastdds::rtps::iHandle2GUID([GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid, const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle) noexcept

Convert [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to GUID 

* **Parameters:**
  * **guid** – GUID to store the results 
  * **ihandle** – [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to copy 

### inline [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) eprosima::fastdds::rtps::iHandle2GUID(const [InstanceHandle_t](InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle) noexcept

Convert [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to GUID 

* **Parameters:**
  **ihandle** – [InstanceHandle_t](InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) to copy 
* **Returns:**
  [GUID_t](../Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t)
