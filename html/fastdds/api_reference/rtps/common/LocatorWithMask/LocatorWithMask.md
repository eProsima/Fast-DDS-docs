<a id="locatorwithmask"></a>

# LocatorWithMask

### class LocatorWithMask : public eprosima::fastdds::rtps::[Locator_t](../Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE)

A Locator with a mask that defines the number of significant bits of its address. 

### Public Functions

### std::uint8_t mask() const

Get the number of significant bits on the address of this locator.

* **Returns:**
  number of significant bits on the address of this locator. 

### void mask(std::uint8_t mask)

Set the number of significant bits on the address of this locator.

* **Parameters:**
  **mask** – number of significant bits on the address of this locator. 

### bool matches(const Locator &loc) const

Check whether the given locator is from the same network as this locator.

* **Parameters:**
  **loc** – locator to check if belonging to the same network as this locator.
* **Returns:**
  true if the two locators are from the same network, false otherwise. 

### [LocatorWithMask](#_CPPv4N8eprosima7fastdds4rtps15LocatorWithMaskE) &operator=(const Locator &loc)

Copy assignment.
