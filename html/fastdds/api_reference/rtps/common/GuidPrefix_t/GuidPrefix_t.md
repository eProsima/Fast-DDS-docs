<a id="guidprefix-t"></a>

# GuidPrefix_t

### struct GuidPrefix_t

Structure [GuidPrefix_t](#structeprosima_1_1fastdds_1_1rtps_1_1GuidPrefix__t), Guid Prefix of [GUID_t](../Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t). 

### Public Functions

### inline GuidPrefix_t()

Default constructor. Set the Guid prefix to 0. 

### bool is_on_same_host_as(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &other_guid_prefix) const

Checks whether this guid prefix is from an entity on the same host as another guid prefix.

#### NOTE
This method assumes the value of `other_guid_prefix` was originally assigned by Fast-DDS vendor.

* **Parameters:**
  **other_guid_prefix** – [GuidPrefix_t](#structeprosima_1_1fastdds_1_1rtps_1_1GuidPrefix__t) to compare to.
* **Returns:**
  true when this guid prefix is on the same host, false otherwise. 

### bool is_from_this_host() const

Checks whether this guid prefix is from a (Fast-DDS) entity created on this host (from where this method is called).

* **Returns:**
  true when this guid prefix is from a (Fast-DDS) entity created on this host, false otherwise. 

### bool is_on_same_process_as(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &other_guid_prefix) const

Checks whether this guid prefix is for an entity on the same host and process as another guid prefix.

#### NOTE
This method assumes the value of `other_guid_prefix` was originally assigned by Fast-DDS vendor.

* **Parameters:**
  **other_guid_prefix** – [GuidPrefix_t](#structeprosima_1_1fastdds_1_1rtps_1_1GuidPrefix__t) to compare to.
* **Returns:**
  true when this guid prefix is on the same host and process, false otherwise. 

### bool is_from_this_process() const

Checks whether this guid prefix is from a (Fast-DDS) entity created on this host and process (from where this method is called).

* **Returns:**
  true when this guid prefix is from a (Fast-DDS) entity created on this host and process, false otherwise. 

### inline bool operator==(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &prefix) const

Guid prefix comparison operator 

* **Parameters:**
  **prefix** – guid prefix to compare 
* **Returns:**
  True if the guid prefixes are equal 

### inline bool operator!=(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &prefix) const

Guid prefix not equal operator 

* **Parameters:**
  **prefix** – Second guid prefix to compare 
* **Returns:**
  True if the guid prefixes are not equal 

### inline bool operator<(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &prefix) const

Guid prefix minor operator 

* **Parameters:**
  **prefix** – Second guid prefix to compare 
* **Returns:**
  True if prefix is higher than this 

### Public Static Functions

### static inline int cmp(const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &prefix1, const [GuidPrefix_t](#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &prefix2)

Guid Prefix compare static method.

* **Parameters:**
  * **prefix1** – First guid prefix to compare 
  * **prefix2** – Second guid prefix to compare
* **Returns:**
  0 if `prefix1` is equal to `prefix2` . 
* **Returns:**
  < 0 if `prefix1` is lower than `prefix2` . 
* **Returns:**
  > 0 if `prefix1` is higher than `prefix2` .
