<a id="locatorselector"></a>

# LocatorSelector

### class LocatorSelector

A class used for the efficient selection of locators when sending data to multiple entities.

Algorithm:
* Entries are added/removed with add_entry/remove_entry when matched/unmatched.
* When data is to be sent:
  * A reference to this object is passed to the message group
  * For each submessage:
    * A call to reset is performed
    * A call to enable is performed per desired destination
    * If [state_has_changed()](#classeprosima_1_1fastdds_1_1rtps_1_1LocatorSelector_1af0197ceacaa40677291c979f02d8d446) returns true:
      * the message group is flushed
      * selection_start is called
      * for each transport:
        * transport_starts is called
        * transport handles the selection state of each entry
        * select may be called
    * Submessage is added to the message group 

### Public Functions

### inline LocatorSelector(const ResourceLimitedContainerConfig &entries_allocation)

Construct a [LocatorSelector](#classeprosima_1_1fastdds_1_1rtps_1_1LocatorSelector).

* **Parameters:**
  **entries_allocation** – Allocation configuration regarding the number of remote entities. 

### inline void clear()

Clears all internal data. 

### inline bool add_entry([LocatorSelectorEntry](../LocatorSelectorEntry/LocatorSelectorEntry.md#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntryE) \*entry)

Add an entry to this selector.

* **Parameters:**
  **entry** – Pointer to the [LocatorSelectorEntry](../LocatorSelectorEntry/LocatorSelectorEntry.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry) to add. 

### inline bool remove_entry(const [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid)

Remove an entry from this selector. 

* **Parameters:**
  **guid** – Identifier of the entry to be removed. 

### inline void reset(bool enable_all)

Reset the enabling state of the selector.

* **Parameters:**
  **enable_all** – Indicates whether entries should be initially enabled. 

### inline void enable(const [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid)

Enable an entry given its GUID.

* **Parameters:**
  **guid** – GUID of the entry to enable. 

### inline bool state_has_changed() const

Check if enabling state has changed.

* **Returns:**
  true if the enabling state has changed, false otherwise. 

### inline void selection_start()

Reset the selection state of the selector. 

### inline ResourceLimitedVector<[LocatorSelectorEntry](../LocatorSelectorEntry/LocatorSelectorEntry.md#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntryE)\*> &transport_starts()

Called when the selection algorithm starts for a specific transport.

Will set the temporary transport_should_process flag for all enabled entries.

* **Returns:**
  a reference to the entries collection. 

### inline void select(std::size_t index)

Marks an entry as selected.

* **Parameters:**
  **index** – The index of the entry to mark as selected. 

### inline void unselect(std::size_t index)

 **[Pro]** 

### inline std::size_t selected_size() const

Count the number of selected locators.

* **Returns:**
  the number of selected locators. 

### inline bool is_selected(const [Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) locator) const

Check if a locator is present in the selections of this object.

* **Parameters:**
  **locator** – The locator to be checked.
* **Returns:**
  True if the locator has been selected, false otherwise. 

### template<class UnaryPredicate>inline void for_each([UnaryPredicate](#_CPPv4I0ENK8eprosima7fastdds4rtps15LocatorSelector8for_eachEv14UnaryPredicate) action) const

Performs an action on each selected locator.

* **Parameters:**
  **action** – Unary function that accepts a locator as argument. The function shall not modify its argument. This can either be a function pointer or a function object. 

### template<class UnaryPredicate>inline bool for_every_entry([UnaryPredicate](#_CPPv4I0ENK8eprosima7fastdds4rtps15LocatorSelector15for_every_entryEb14UnaryPredicate) action) const

 **[Pro]** 

### inline [LocatorSelectorEntry](../LocatorSelectorEntry/LocatorSelectorEntry.md#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntryE) \*get_entry_by_guid(const [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &guid) const

 **[Pro]** 

### inline void initial_allow_to_send(bool value)

 **[Pro]** 

### inline bool initial_allow_to_send() const

 **[Pro]** 

### class iterator : public eprosima::fastdds::rtps::[LocatorsIterator](../Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE)

### Public Functions

### inline virtual [iterator](#_CPPv4N8eprosima7fastdds4rtps15LocatorSelector8iteratorE) &operator++()

Increment operator. 

* **Returns:**
  [LocatorsIterator](../Locator/LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator)& reference to the next [LocatorsIterator](../Locator/LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator). 

### inline virtual bool operator==(const [LocatorsIterator](../Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const

Equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](../Locator/LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if equal. 
* **Returns:**
  false otherwise. 

### inline virtual bool operator!=(const [LocatorsIterator](../Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const

Not equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](../Locator/LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if not equal. 
* **Returns:**
  false otherwise. 

### inline virtual reference operator\*() const

Dereference operator. 

* **Returns:**
  const Locator& Reference to the locator pointed by the [LocatorsIterator](../Locator/LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator). 

### struct IteratorIndex
