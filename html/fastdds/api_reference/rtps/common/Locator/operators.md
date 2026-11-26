<a id="locator-operators"></a>

# Locator Operators

### inline bool eprosima::fastdds::rtps::operator<(const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc1, const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc2)

Less than operator. 

* **Parameters:**
  * **loc1** – Left hand side locator being compared. 
  * **loc2** – Right hand side locator being compared. 
* **Returns:**
  true if `loc1` is less than `loc2`. 
* **Returns:**
  false otherwise. 

### inline bool eprosima::fastdds::rtps::operator==(const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc1, const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc2)

Equal to operator. 

* **Parameters:**
  * **loc1** – Left hand side locator being compared. 
  * **loc2** – Right hand side locator being compared. 
* **Returns:**
  true if `loc1` is equal to `loc2`. 
* **Returns:**
  false otherwise. 

### inline bool eprosima::fastdds::rtps::operator!=(const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc1, const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc2)

Not equal to operator. 

* **Parameters:**
  * **loc1** – Left hand side locator being compared. 
  * **loc2** – Right hand side locator being compared. 
* **Returns:**
  true if `loc1` is not equal to `loc2`. 
* **Returns:**
  false otherwise. 

### inline std::ostream &eprosima::fastdds::rtps::operator<<(std::ostream &output, const [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc)

Insertion operator: serialize a locator The serialization format is kind:[address]:port `kind` must be one of the following: 

* UDPv4
* UDPv6
* TCPv4
* TCPv6
* SHM
* ETHERNET `address` IP address unless `kind` is SHM `port` number

* **Parameters:**
  * **output** – Output stream where the serialized locator is appended. 
  * **loc** – Locator to be serialized/inserted. 
* **Returns:**
  `std::ostream&` Reference to the output stream with the serialized locator appended. 

### inline std::istream &eprosima::fastdds::rtps::operator>>(std::istream &input, [Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc)

Extraction operator: deserialize a locator The deserialization format is kind:[address]:port `kind` must be one of the following: 

* UDPv4
* UDPv6
* TCPv4
* TCPv6
* SHM
* ETHERNET `address` must be either a name which can be resolved by DNS or the IP address unless `kind` is SHM `port` number

* **Parameters:**
  * **input** – Input stream where the locator to be deserialized is located. 
  * **loc** – Locator where the deserialized locator is saved. 
* **Returns:**
  `std::istream&` Reference to the input stream after extracting the locator. 

### inline std::ostream &eprosima::fastdds::rtps::operator<<(std::ostream &output, const [LocatorList](LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locList)

Insertion operator: serialize a locator list. The deserialization format is [locator1,locator2,…,locatorN]. Each individual locator within the list must follow the serialization format explained in the locator insertion operator. 

* **Parameters:**
  * **output** – Output stream where the serialized locator list is appended. 
  * **locList** – Locator list to be serialized/inserted. 
* **Returns:**
  `std::ostream&` Reference to the output stream with the serialized locator list appended. 

### inline std::istream &eprosima::fastdds::rtps::operator>>(std::istream &input, [LocatorList](LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locList)

Extraction operator: deserialize a list of locators. The serialization format is [locator1,locator2,…,locatorN]. Each individual locator within the list must follow the deserialization format explained in the locator extraction operator. 

* **Parameters:**
  * **input** – Input stream where the locator list to be deserialized is located. 
  * **locList** – Locator list where the deserialized locators are saved. 
* **Returns:**
  `std::istream&` Reference to the input stream after extracting the locator list. 

### static inline bool eprosima::fastdds::rtps::operator==(const ResourceLimitedVector<[Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> &lhs, const ResourceLimitedVector<[Locator_t](Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> &rhs)

Equal to operator to compare two locator lists. 

* **Parameters:**
  * **lhs** – Locator list to be compared. 
  * **rhs** – Other locator list to be compared. 
* **Returns:**
  true if the list are equal. 
* **Returns:**
  false otherwise.
