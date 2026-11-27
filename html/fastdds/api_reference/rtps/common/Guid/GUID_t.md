<a id="guid-t"></a>

# GUID_t

### struct GUID_t

Structure [GUID_t](#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t), entity identifier, unique in DDS-RTPS Domain. 

### Public Functions

### inline GUID_t() noexcept

Default constructor. 

Contructs an unknown GUID. 

### inline GUID_t(const [GuidPrefix_t](../GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &guid_prefix, std::uint32_t id) noexcept

Construct 

* **Parameters:**
  * **guid_prefix** – Guid prefix 
  * **id** – Entity id 

### inline GUID_t(const [GuidPrefix_t](../GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) &guid_prefix, const [EntityId_t](../EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entity_id) noexcept

* **Parameters:**
  * **guid_prefix** – Guid prefix 
  * **entity_id** – Entity id 

### inline bool is_on_same_host_as(const [GUID_t](#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &other_guid) const

Checks whether this guid is from an entity on the same host as another guid.

#### NOTE
This method assumes the value of `other_guid` was originally assigned by Fast-DDS vendor.

* **Parameters:**
  **other_guid** – [GUID_t](#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) to compare to.
* **Returns:**
  true when this guid is on the same host, false otherwise. 

### inline bool is_from_this_host() const

Checks whether this guid is from a (Fast-DDS) entity created on this host (from where this method is called).

* **Returns:**
  true when this guid is from a (Fast-DDS) entity created on this host, false otherwise. 

### inline bool is_on_same_process_as(const [GUID_t](#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &other_guid) const

Checks whether this guid is for an entity on the same host and process as another guid.

#### NOTE
This method assumes the value of `other_guid` was originally assigned by Fast-DDS vendor.

* **Parameters:**
  **other_guid** – [GUID_t](#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t) to compare to.
* **Returns:**
  true when this guid is on the same host and process, false otherwise. 

### inline bool is_from_this_process() const

Checks whether this guid is from a (Fast-DDS) entity created on this process (from where this method is called).

* **Returns:**
  true when this guid is from a (Fast-DDS) entity created on this process, false otherwise. 

### inline bool is_builtin() const

Checks whether this guid corresponds to a builtin entity.

* **Returns:**
  true when this guid corresponds to a builtin entity, false otherwise. 

### Public Members

### [GuidPrefix_t](../GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) guidPrefix

Guid prefix. 

### [EntityId_t](../EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) entityId

Entity id.
