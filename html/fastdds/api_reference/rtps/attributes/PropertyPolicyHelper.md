<a id="propertypolicyhelper"></a>

# PropertyPolicyHelper

### class PropertyPolicyHelper

### Public Static Functions

### static [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) get_properties_with_prefix(const [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) &property_policy, const std::string &prefix)

Returns only the properties whose name starts with the prefix. 

Prefix is removed in returned properties. 

* **Parameters:**
  * **property_policy** – [PropertyPolicy](PropertyPolicy.md#classeprosima_1_1fastdds_1_1rtps_1_1PropertyPolicy) where properties will be searched. 
  * **prefix** – Prefix used to search properties. 
* **Returns:**
  A copy of properties whose name starts with the prefix. 

### static std::size_t length(const [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) &property_policy)

Get the length of the property_policy. 

### static std::string \*find_property([PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) &property_policy, const std::string &name)

Look for a property_policy by name. 

### static const std::string \*find_property(const [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) &property_policy, const std::string &name)

Retrieves a property_policy by name. 

### static const [Property](../common/Property/Property.md#_CPPv4N8eprosima7fastdds4rtps8PropertyE) \*get_property(const [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) &property_policy, const std::string &name)

Retrieves a property by name. 

* **Parameters:**
  * **property_policy** – [PropertyPolicy](PropertyPolicy.md#classeprosima_1_1fastdds_1_1rtps_1_1PropertyPolicy) where the property will be searched. 
  * **name** – Name of the property to be searched. 
* **Returns:**
  A pointer to the property if found, nullptr otherwise.
