<a id="locators"></a>

# Locators

### class Locators : public eprosima::fastdds::rtps::[LocatorsIterator](LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE)

Adapter class that provides a [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) interface from a LocatorListConstIterator 

### Public Functions

### inline Locators(const [LocatorListConstIterator](LocatorListConstIterator.md#_CPPv4N8eprosima7fastdds4rtps24LocatorListConstIteratorE) &it)

Constructor. 

### inline Locators(const [Locators](#_CPPv4N8eprosima7fastdds4rtps8Locators8LocatorsERK8Locators) &other)

Copy constructor. 

### inline virtual [LocatorsIterator](LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &operator++()

Increment operator. 

* **Returns:**
  [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator)& reference to the next [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator). 

### inline virtual bool operator==(const [LocatorsIterator](LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const

Equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if equal. 
* **Returns:**
  false otherwise. 

### inline virtual bool operator!=(const [LocatorsIterator](LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) &other) const

Not equal to operator. 

* **Parameters:**
  **other** – [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator) to compare. 
* **Returns:**
  true if not equal. 
* **Returns:**
  false otherwise. 

### inline virtual const Locator &operator\*() const

Dereference operator. 

* **Returns:**
  const Locator& Reference to the locator pointed by the [LocatorsIterator](LocatorsIterator.md#structeprosima_1_1fastdds_1_1rtps_1_1LocatorsIterator).
