<a id="writerhistory"></a>

# WriterHistory

### class WriterHistory : public eprosima::fastdds::rtps::[History](History_class.md#_CPPv4N8eprosima7fastdds4rtps7HistoryE)

Class [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory), container of the different CacheChanges of a writer 

### Public Functions

### WriterHistory(const [HistoryAttributes](../attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) &att)

Construct a [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

* **Parameters:**
  **att** – Attributes configuring the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

### WriterHistory(const [HistoryAttributes](../attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) &att, const std::shared_ptr<[IPayloadPool](IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> &payload_pool)

Construct a [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory) with a custom payload pool. 

* **Parameters:**
  * **att** – Attributes configuring the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 
  * **payload_pool** – Pool of payloads to be used by the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

### WriterHistory(const [HistoryAttributes](../attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) &att, const std::shared_ptr<[IPayloadPool](IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> &payload_pool, const std::shared_ptr<[IChangePool](IChangePool.md#_CPPv4N8eprosima7fastdds4rtps11IChangePoolE)> &change_pool)

Construct a [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory) with custom payload and change pools. 

* **Parameters:**
  * **att** – Attributes configuring the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 
  * **payload_pool** – Pool of payloads to be used by the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 
  * **change_pool** – Pool of changes to be used by the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

### const std::shared_ptr<[IPayloadPool](IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE)> &get_payload_pool() const

Get the payload pool used by this history. 

* **Returns:**
  Reference to the payload pool used by this history. 

### const std::shared_ptr<[IChangePool](IChangePool.md#_CPPv4N8eprosima7fastdds4rtps11IChangePoolE)> &get_change_pool() const

Get the change pool used by this history. 

* **Returns:**
  Reference to the change pool used by this history. 

### [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*create_change([ChangeKind_t](../common/CacheChange/ChangeKind_t.md#_CPPv4N8eprosima7fastdds4rtps12ChangeKind_tE) change_kind, [InstanceHandle_t](../common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) handle = [c_InstanceHandle_Unknown](../common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE))

Create a new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) object. 

* **Parameters:**
  * **change_kind** – Kind of the change. 
  * **handle** – [InstanceHandle_t](../common/InstanceHandle/InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) of the change.
* **Returns:**
  Pointer to the new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) object.
* **Pre:**
  A writer has been associated with this history 

### [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*create_change(std::uint32_t payload_size, [ChangeKind_t](../common/CacheChange/ChangeKind_t.md#_CPPv4N8eprosima7fastdds4rtps12ChangeKind_tE) change_kind, [InstanceHandle_t](../common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) handle = [c_InstanceHandle_Unknown](../common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE))

Create a new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) object with a specific payload size. 

* **Parameters:**
  * **payload_size** – Size of the payload. 
  * **change_kind** – Kind of the change. 
  * **handle** – [InstanceHandle_t](../common/InstanceHandle/InstanceHandle_t.md#structeprosima_1_1fastdds_1_1rtps_1_1InstanceHandle__t) of the change.
* **Returns:**
  Pointer to the new [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) object.
* **Pre:**
  A writer has been associated with this history 

### bool add_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*a_change)

Add a [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

* **Parameters:**
  **a_change** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to be added. 
* **Returns:**
  True if added. 

### bool add_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*a_change, [WriteParams](../common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &wparams)

Add a [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to the [WriterHistory](#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory). 

* **Parameters:**
  * **a_change** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to be added. 
  * **wparams** – Extra write parameters. 
* **Returns:**
  True if added. 

### virtual iterator remove_change_nts(const_iterator removal, bool release = true) override

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the change for removal 
  * **release** – specifies if the change should be return to the pool 
* **Returns:**
  iterator to the next change if any 

### virtual iterator remove_change_nts(const_iterator removal, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time, bool release = true) override

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the change for removal 
  * **release** – specifies if the change should be return to the pool 
  * **max_blocking_time** – **[in]** Maximum time this method has to complete the task. 
* **Returns:**
  iterator to the next change if any 

### virtual bool matches_change(const [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*inner, [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*outer) override

Criteria to search a specific [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) on history 

* **Parameters:**
  * **inner** – change to compare 
  * **outer** – change for comparison 
* **Returns:**
  true if inner matches outer criteria 

### bool remove_min_change()

Remove the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) with the minimum sequenceNumber. 

* **Returns:**
  True if correctly removed. 

### bool remove_min_change(const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time)

Remove the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) with the minimum sequenceNumber. 

* **Parameters:**
  **max_blocking_time** – **[in]** Maximum time this method has to complete the task. 
* **Returns:**
  True if correctly removed. 

### bool release_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Release a change when it is not being used anymore.

* **Parameters:**
  **ch** – Pointer to the cache change to be released.
* **Returns:**
  whether the operation succeeded or not
* **Pre:**
  * A writer has been associated with this history 
  * `ch` is not `nullptr`
  * `ch` points to a cache change obtained from a call to `this->create_change`
* **Post:**
  memory pointed to by `ch` is not accessed 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Introduce base class method into scope. 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time)

Introduce base class method into scope. 

### inline iterator remove_change(const_iterator removal, bool release = true)

Introduce base class method into scope.
