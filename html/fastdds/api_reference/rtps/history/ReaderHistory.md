<a id="readerhistory"></a>

# ReaderHistory

### class ReaderHistory : public eprosima::fastdds::rtps::[History](History_class.md#_CPPv4N8eprosima7fastdds4rtps7HistoryE)

Class [ReaderHistory](#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory), container of the different CacheChanges of a reader 

### Public Functions

### ReaderHistory(const [HistoryAttributes](../attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) &att)

Constructor of the [ReaderHistory](#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). It needs a [HistoryAttributes](../attributes/HistoryAttributes.md#classeprosima_1_1fastdds_1_1rtps_1_1HistoryAttributes). 

### virtual bool can_change_be_added_nts(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, std::uint32_t total_payload_size, std::size_t unknown_missing_changes_up_to, bool &will_never_be_accepted) const

Check if a new change can be added to this history.

* **Parameters:**
  * **writer_guid** – **[in]** GUID of the writer where the change came from. 
  * **total_payload_size** – **[in]** Total payload size of the incoming change. 
  * **unknown_missing_changes_up_to** – **[in]** The number of changes from the same writer with a lower sequence number that could potentially be received in the future. 
  * **will_never_be_accepted** – **[out]** When the method returns `false`, this parameter will inform whether the change could be accepted in the future or not.
* **Pre:**
  change should not be present in the history
* **Returns:**
  Whether a call to received_change will succeed when called with the same arguments. 

### virtual bool received_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*change, std::size_t unknown_missing_changes_up_to)

Virtual method that is called when a new change is received. In this implementation this method just calls add_change. The user can overload this method in case he needs to perform additional checks before adding the change. 

* **Parameters:**
  * **change** – Pointer to the change 
  * **unknown_missing_changes_up_to** – The number of changes from the same writer with a lower sequence number that could potentially be received in the future. 
* **Returns:**
  True if added. 

### inline virtual bool received_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*change, std::size_t unknown_missing_changes_up_to, fastdds::dds::[SampleRejectedStatusKind](../../dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) &rejection_reason)

Virtual method that is called when a new change is received. In this implementation this method just calls add_change. The user can overload this method in case he needs to perform additional checks before adding the change. 

* **Parameters:**
  * **change** – **[in]** Pointer to the change 
  * **unknown_missing_changes_up_to** – **[in]** The number of changes from the same writer with a lower sequence number that could potentially be received in the future. 
  * **rejection_reason** – **[out]** In case of been rejected the sample, it will contain the reason of the rejection. 
* **Returns:**
  True if added. 

### inline virtual bool completed_change(rtps::[CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*change)

Called when a fragmented change is received completely by the Subscriber. Will find its instance and store it. 

* **Parameters:**
  **change** – **[in]** The received change 
* **Pre:**
  Change should be already present in the history. 
* **Returns:**

### inline virtual bool completed_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*change, std::size_t unknown_missing_changes_up_to, fastdds::dds::[SampleRejectedStatusKind](../../dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) &rejection_reason)

Called when a fragmented change is received completely by the Subscriber. Will find its instance and store it. 

* **Parameters:**
  * **change** – **[in]** The received change 
  * **unknown_missing_changes_up_to** – **[in]** Number of missing changes before this one 
  * **rejection_reason** – **[out]** In case of been rejected the sample, it will contain the reason of the rejection. 
* **Pre:**
  Change should be already present in the history. 
* **Returns:**

### bool add_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*a_change)

Add a [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) to the [ReaderHistory](#classeprosima_1_1fastdds_1_1rtps_1_1ReaderHistory). 

* **Parameters:**
  **a_change** – Pointer to the CacheChange to add. 
* **Returns:**
  True if added. 

### virtual iterator remove_change_nts(const_iterator removal, bool release = true) override

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the change for removal 
  * **release** – specifies if the change must be returned to the pool 
* **Returns:**
  iterator to the next change if any 

### virtual iterator remove_change_nts(const_iterator removal, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time, bool release = true) override

Remove a specific change from the history. No Thread Safe 

* **Parameters:**
  * **removal** – iterator to the change for removal 
  * **max_blocking_time** – **[in]** Maximum time this method has to complete the task. 
  * **release** – specifies if the change must be returned to the pool 
* **Returns:**
  iterator to the next change if any 

### virtual bool matches_change(const [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*inner, [CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*outer) override

Criteria to search a specific [CacheChange_t](../common/CacheChange/CacheChange_t.md#structeprosima_1_1fastdds_1_1rtps_1_1CacheChange__t) on history 

* **Parameters:**
  * **inner** – change to compare 
  * **outer** – change for comparison 
* **Returns:**
  true if inner matches outer criteria 

### bool remove_changes_with_guid(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &a_guid)

Remove all changes from the [History](History_class.md#classeprosima_1_1fastdds_1_1rtps_1_1History) that have a certain guid. 

* **Parameters:**
  **a_guid** – Pointer to the target guid to search for. 
* **Returns:**
  True if successful, even if no changes have been removed. 

### bool remove_fragmented_changes_until(const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &seq_num, const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid)

Remove all fragmented changes from certain writer up to certain sequence number. 

* **Parameters:**
  * **seq_num** – First [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#structeprosima_1_1fastdds_1_1rtps_1_1SequenceNumber__t) not to be removed. 
  * **writer_guid** – GUID of the writer for which changes should be looked for. 
* **Returns:**
  True if successful, even if no changes have been removed. 

### virtual void writer_unmatched(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, const [SequenceNumber_t](../common/SequenceNumber/SequenceNumber_t.md#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &last_notified_seq)

Called when a writer is unmatched from the reader holding this history.

This method will remove all the changes on the history that came from the writer being unmatched and which have not yet been notified to the user.

* **Parameters:**
  * **writer_guid** – GUID of the writer being unmatched. 
  * **last_notified_seq** – Last sequence number from the specified writer that was notified to the user. 

### inline virtual void writer_update_its_ownership_strength_nts(const [GUID_t](../common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &writer_guid, const std::uint32_t ownership_strength)

This function should be called by reader if a writer updates its ownership strength. 

* **Parameters:**
  * **writer_guid** – **[in]** Guid of the writer which changes its ownership strength. 
  * **ownership_strength** – **[out]** New value of the writer’s Ownership strength. 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch)

Introduce base class method into scope. 

### bool remove_change([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*ch, const std::chrono::time_point<std::chrono::steady_clock> &max_blocking_time)

Introduce base class method into scope. 

### inline iterator remove_change(const_iterator removal, bool release = true)

Introduce base class method into scope.
