<a id="locatorsiterator"></a>

# LocatorsIterator

### struct LocatorsIterator

Provides a Locator’s iterator interface that can be used by different Locator’s containers 

Subclassed by [eprosima::fastdds::rtps::LocatorSelector::iterator](../LocatorSelector/LocatorSelector.md#classeprosima_1_1fastdds_1_1rtps_1_1LocatorSelector_1_1iterator), [eprosima::fastdds::rtps::Locators](Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators)

### Public Functions

### virtual [LocatorsIterator](#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &operator++() = 0

Increment operator. 

* **Returns:**
  [LocatorsIterator](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator)& reference to the next [LocatorsIterator](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator). 

### virtual bool operator==(const [LocatorsIterator](#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const = 0

Equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if equal. 
* **Returns:**
  false otherwise. 

### virtual bool operator!=(const [LocatorsIterator](#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const = 0

Not equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if not equal. 
* **Returns:**
  false otherwise. 

### virtual const Locator &operator\*() const = 0

Dereference operator. 

* **Returns:**
  const Locator& Reference to the locator pointed by the [LocatorsIterator](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator).
