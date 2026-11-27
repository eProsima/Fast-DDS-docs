<a id="locatorselectorentry"></a>

# LocatorSelectorEntry

### struct LocatorSelectorEntry

An entry for the [LocatorSelector](../LocatorSelector/LocatorSelector.md#classeprosima_1_1fastdds_1_1rtps_1_1LocatorSelector).

This class holds the locators of a remote endpoint along with data required for the locator selection algorithm. 

### Public Functions

### inline LocatorSelectorEntry(std::size_t max_unicast_locators, std::size_t max_multicast_locators)

Construct a [LocatorSelectorEntry](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry).

* **Parameters:**
  * **max_unicast_locators** – Maximum number of unicast locators to hold. 
  * **max_multicast_locators** – Maximum number of multicast locators to hold. 

### inline void enable(bool should_enable)

Set the enabled value.

* **Parameters:**
  **should_enable** – Whether this entry should be enabled. 

### inline void reset()

Reset the selections. 

### Public Members

### [GUID_t](../Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) remote_guid

GUID of the remote entity. 

### ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> unicast

List of unicast locators to send data to the remote entity. 

### ResourceLimitedVector<[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)> multicast

List of multicast locators to send data to the remote entity. 

### [EntryState](#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntry10EntryStateE) state

State of the entry. 

### bool enabled = {false}

Indicates whether this entry should be taken into consideration. 

### bool allowed_to_send = {true}

Indicates whether this entry is allowed to send data. 

### bool transport_should_process = {false}

A temporary value for each transport to help optimizing some use cases. 

### struct EntryState

Holds the selection state of the locators held by a [LocatorSelectorEntry](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry)

### Public Functions

### inline EntryState(std::size_t max_unicast_locators, std::size_t max_multicast_locators)

Construct an [EntryState](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry_1_1EntryState) object.

* **Parameters:**
  * **max_unicast_locators** – Maximum number of unicast locators to held by parent [LocatorSelectorEntry](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry). 
  * **max_multicast_locators** – Maximum number of multicast locators to held by parent [LocatorSelectorEntry](#structeprosima_1_1fastdds_1_1rtps_1_1LocatorSelectorEntry). 

### Public Members

### ResourceLimitedVector<std::size_t> unicast

Unicast locators selection state. 

### ResourceLimitedVector<std::size_t> multicast

Multicast locators selection state.
