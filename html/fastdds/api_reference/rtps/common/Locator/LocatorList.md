<a id="locatorlist"></a>

# LocatorList

### class LocatorList

Class [LocatorList](#classeprosima_1_1fastdds_1_1rtps_1_1LocatorList), a Locator vector that doesn’t allow duplicates. 

### Public Functions

### inline LocatorList()

Constructor. 

### inline ~LocatorList()

Destructor. 

### inline LocatorList(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorList11LocatorListERK11LocatorList) &list)

Copy constructor. 

### inline LocatorList([LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorList11LocatorListERR11LocatorList) &&list)

Move constructor. 

### inline [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &operator=(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &list)

Copy assignment. 

### inline [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &operator=([LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &&list)

Move assignment. 

### inline bool operator==(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locator_list) const

Equal to operator. 

### inline bool operator!=(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locator_list) const

Not equal to operator. 

### inline [LocatorListIterator](LocatorListIterator.md#_CPPv4N8eprosima7fastdds4rtps19LocatorListIteratorE) begin()

Return an iterator to the beginning. 

* **Returns:**
  LocatorListIterator iterator to the first locator. 

### inline [LocatorListIterator](LocatorListIterator.md#_CPPv4N8eprosima7fastdds4rtps19LocatorListIteratorE) end()

Return an iterator to the end. 

* **Returns:**
  LocatorListIterator iterator to the element following the last element. 

### inline [LocatorListConstIterator](LocatorListConstIterator.md#_CPPv4N8eprosima7fastdds4rtps24LocatorListConstIteratorE) begin() const

Return a constant iterator to the beginning. 

* **Returns:**
  LocatorListConstIterator iterator to the first locator. 

### inline [LocatorListConstIterator](LocatorListConstIterator.md#_CPPv4N8eprosima7fastdds4rtps24LocatorListConstIteratorE) end() const

Return a constant iterator to the end. 

* **Returns:**
  LocatorListConstIterator iterator to the element following the last element. 

### inline std::size_t size() const

Return the number of locators. 

* **Returns:**
  size_t The number of locators in the container. 

### inline [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &assign(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &list)

Replace the contents of the container. 

* **Parameters:**
  **list** – New content to be saved into the container. 
* **Returns:**
  [LocatorList](#classeprosima_1_1fastdds_1_1rtps_1_1LocatorList)& reference to the container with the replaced content. 

### inline void clear()

Erase all locators from the container. 

### inline void reserve(std::size_t num)

Reserve storage increasing the capacity of the vector. 

* **Parameters:**
  **num** – new capacity of the vector, in number of elements. 

### inline void resize(std::size_t num)

Resize the container to contain `num` locators. If the current size is greater than `num`, the container is reduced to its first `num` locators. If the current size is less than count, additional default-inserted locators are appended. 

* **Parameters:**
  **num** – new size of the container. 

### inline void push_back(const Locator &loc)

Add locator to the end if not found within the list. 

* **Parameters:**
  **loc** – locator to be appended. 

### inline void push_back(const [LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locList)

Add several locators to the end if not already present within the list. 

* **Parameters:**
  **locList** – [LocatorList](#classeprosima_1_1fastdds_1_1rtps_1_1LocatorList) with the locators to be appended. 

### inline bool empty() const

Check that the container has no locators. 

* **Returns:**
  true if the container is empty. False otherwise. 

### inline void erase(const Locator &loc)

Erase the specified locator from the container. 

* **Parameters:**
  **loc** – Locator to be removed. 

### inline bool isValid() const

Check that every locator contained in the list is not LOCATOR_KIND_INVALID. 

* **Returns:**
  true if all locators are valid. False otherwise. 

### inline void swap([LocatorList](#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locatorList)

exchange the content of the container. 

* **Parameters:**
  **locatorList** – container to exchange the contents with.
