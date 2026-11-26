<a id="history"></a>

# History

### class History

Class [History](#classeprosima_1_1fastdds_1_1rtps_1_1History), container of the different CacheChanges and the methods to access them. 

Subclassed by [eprosima::fastdds::rtps::ReaderHistory](ReaderHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory), [eprosima::fastdds::rtps::WriterHistory](WriterHistory.md#classeprosima_1_1fastdds_1_1rtps_1_1WriterHistory)

### Public Functions

### inline bool isFull()

Check if the history is full 

* **Returns:**
  true if the [History](#classeprosima_1_1fastdds_1_1rtps_1_1History) is full. 

### inline std::size_t getHistorySize()

Get the [History](#classeprosima_1_1fastdds_1_1rtps_1_1History) size. 

* **Returns:**
  Size of the history. 

### const_iterator find_change_nts([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Find a specific change in the history using the matches_change method criteria. No Thread Safe 

* **Parameters:**
  **ch** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to search for. 
* **Returns:**
  an iterator if a suitable change is found 

### virtual iterator remove_change_nts(const_iterator removal, bool release = true)

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to remove. 
  * **release** – defaults to true and hints if the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) should return to the pool 
* **Returns:**
  iterator to the next [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) or end iterator. 

### virtual iterator remove_change_nts(const_iterator removal, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time, bool release = true)

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to remove. 
  * **max_blocking_time** – **[in]** Maximum time this method has to complete the task. 
  * **release** – defaults to true and hints if the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) should return to the pool 
* **Returns:**
  iterator to the next [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) or end iterator. 

### bool remove_all_changes()

Remove all changes from the [History](#classeprosima_1_1fastdds_1_1rtps_1_1History) 

* **Returns:**
  True if everything was correctly removed. 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Remove a specific change from the history. 

* **Parameters:**
  **ch** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 
* **Returns:**
  True if removed. 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time)

Remove a specific change from the history. 

* **Parameters:**
  * **ch** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 
  * **max_blocking_time** – **[in]** Maximum time this method has to complete the task. 
* **Returns:**
  True if removed. 

### inline const_iterator find_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Find a specific change in the history using the matches_change method criteria. 

* **Parameters:**
  **ch** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to search for. 
* **Returns:**
  an iterator if a suitable change is found 

### virtual bool matches_change(const [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch_inner, [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch_outer)

Verifies if an element of the changes collection matches a given change Derived classes have more info on how to identify univocally a change and should override. 

* **Parameters:**
  * **ch_inner** – element of the collection to compare with the given change 
  * **ch_outer** – Pointer to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to identify. 
* **Returns:**
  true if the iterator identifies this change. 

### inline iterator remove_change(const_iterator removal, bool release = true)

Remove a specific change from the history. 

* **Parameters:**
  * **removal** – iterator to the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to remove. 
  * **release** – defaults to true and hints if the [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) should return to the pool 
* **Returns:**
  iterator to the next [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) or end iterator. 

### inline iterator changesBegin()

Get the beginning of the changes history iterator. 

* **Returns:**
  Iterator to the beginning of the vector. 

### inline iterator changesEnd()

Get the end of the changes history iterator. 

* **Returns:**
  Iterator to the end of the vector. 

### bool get_min_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*\*min_change)

Get the minimum [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 

* **Parameters:**
  **min_change** – Pointer to pointer to the minimum change. 
* **Returns:**
  True if correct. 

### bool get_max_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*\*max_change)

Get the maximum [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t). 

* **Parameters:**
  **max_change** – Pointer to pointer to the maximum change. 
* **Returns:**
  True if correct. 

### inline std::uint32_t getTypeMaxSerialized()

Get the maximum serialized payload size 

* **Returns:**
  Maximum serialized payload size 

### inline RecursiveTimedMutex \*getMutex() const

Get the mutex. 

* **Returns:**
  Mutex 

### bool get_earliest_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*\*change)

A method to get the change with the earliest timestamp. 

* **Parameters:**
  **change** – Pointer to pointer to earliest change 
* **Returns:**
  True on success 

### Public Members

### [HistoryAttributes](../attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) m_att

Attributes of the [History](#classeprosima_1_1fastdds_1_1rtps_1_1History).
