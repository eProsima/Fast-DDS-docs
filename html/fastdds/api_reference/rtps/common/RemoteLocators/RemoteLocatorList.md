<a id="remotelocatorlist"></a>

# RemoteLocatorList

### struct RemoteLocatorList

Holds information about the locators of a remote entity. 

### Public Functions

### inline RemoteLocatorList()

Default constructor of [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) for deserialize. 

### inline RemoteLocatorList(std::size_t max_unicast_locators, std::size_t max_multicast_locators)

Construct a [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList).

* **Parameters:**
  * **max_unicast_locators** – Maximum number of unicast locators to hold. 
  * **max_multicast_locators** – Maximum number of multicast locators to hold. 

### inline RemoteLocatorList(const [RemoteLocatorList](#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorList17RemoteLocatorListERK17RemoteLocatorList) &other)

Copy-construct a [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList).

* **Parameters:**
  **other** – [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) to copy data from. 

### inline [RemoteLocatorList](#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) &operator=(const [RemoteLocatorList](#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) &other)

Assign locator values from other [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList).

**Remark**

Using the assignment operator is different from copy-constructing as in the first case the configuration with the maximum number of locators is not copied. This means that, for two lists with different maximum number of locators, the expression `(a = b) == b` may not be true. 

* **Parameters:**
  **other** – [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) to copy data from.

### inline const ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> &get_unicast() const

 **[Pro]**  Get the unicast locators list.

* **Returns:**
  Reference to the unicast locators list. 

### inline const ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> &get_multicast() const

 **[Pro]**  Get the multicast locators list.

* **Returns:**
  Reference to the multicast locators list. 

### inline bool is_unicast_list_equal_to(const [RemoteLocatorList](#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) &other) const

 **[Pro]**  Check equality between two [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) Unicast list objects without considering order and assuming no duplicates exist.

* **Parameters:**
  **other** – [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) to compare with. 
* **Returns:**
  true if both Unicast lists contains the same locators. False otherwise. 

### inline bool is_multicast_list_equal_to(const [RemoteLocatorList](#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) &other) const

 **[Pro]**  Check equality between two [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) Multicast list objects without considering order and assuming no duplicates exist.

* **Parameters:**
  **other** – [RemoteLocatorList](#structeprosima_1_1fastdds_1_1rtps_1_1RemoteLocatorList) to compare with. 
* **Returns:**
  true if both Multicast lists contains the same locators. False otherwise. 

### inline void add_unicast_locator(const [Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator)

Adds a locator to the unicast list.

If the locator already exists in the unicast list, or the maximum number of unicast locators has been reached, the new locator is silently discarded.

* **Parameters:**
  **locator** – Unicast locator to be added. 

### inline void add_multicast_locator(const [Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator)

Adds a locator to the multicast list.

If the locator already exists in the multicast list, or the maximum number of multicast locators has been reached, the new locator is silently discarded.

* **Parameters:**
  **locator** – Multicast locator to be added. 

### Public Members

### ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> unicast

List of unicast locators. 

### ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> multicast

List of multicast locators.
