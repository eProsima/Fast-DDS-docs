<a id="entityid-t"></a>

# EntityId_t

### struct EntityId_t

Structure [EntityId_t](#structeprosima_1_1fastdds_1_1rtps_1_1EntityId__t), entity id part of [GUID_t](../Guid/GUID_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GUID__t). 

### Public Functions

### inline EntityId_t()

Default constructor. Unknown entity. 

### inline EntityId_t(std::uint32_t id)

Main constructor. 

* **Parameters:**
  **id** – Entity id 

### inline EntityId_t(const [EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_t10EntityId_tERK10EntityId_t) &id)

Copy constructor. 

### inline EntityId_t([EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_t10EntityId_tERR10EntityId_t) &&id)

Move constructor. 

### inline [EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &operator=(std::uint32_t id)

Assignment operator. 

* **Parameters:**
  **id** – Entity id to copy 

### inline std::uint32_t to_uint32() const

conversion to uint32_t 

* **Returns:**
  uint32_t representation 

### inline bool operator<(const [EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &other) const

Entity Id minor operator 

* **Parameters:**
  **other** – Second entity id to compare 
* **Returns:**
  True if `other` is higher than this 

### Public Static Functions

### static inline int cmp(const [EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entity1, const [EntityId_t](#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE) &entity2)

Entity Id compare static method.

* **Parameters:**
  * **entity1** – First entity id to compare 
  * **entity2** – Second entity id to compare
* **Returns:**
  0 if `entity1` is equal to `entity2` . 
* **Returns:**
  < 0 if `entity1` is lower than `entity2` . 
* **Returns:**
  > 0 if `entity1` is higher than `entity2` .
