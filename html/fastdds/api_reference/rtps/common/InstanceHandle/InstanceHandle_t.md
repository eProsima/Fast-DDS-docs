<a id="instancehandle-t"></a>

# InstanceHandle_t

### struct InstanceHandle_t

Struct [InstanceHandle_t](#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t), used to contain the key for WITH_KEY topics. 

### Public Functions

### [InstanceHandle_t](#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &operator=(const [InstanceHandle_t](#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &ihandle) noexcept = default

Assignment operator 

* **Parameters:**
  **ihandle** – Instance handle to copy the data from 

### inline [InstanceHandle_t](#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) &operator=(const [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid) noexcept

Assignment operator 

* **Parameters:**
  **guid** – GUID to copy the data from 

### inline bool isDefined() const noexcept

Know if the instance handle is defined 

* **Returns:**
  True if the values are not zero. 

### Public Members

### InstanceHandleValue_t value

Value.
