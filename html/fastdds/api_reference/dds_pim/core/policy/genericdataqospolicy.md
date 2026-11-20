<a id="api-pim-genericdataqospolicy"></a>

<a id="genericdataqospolicy"></a>

# GenericDataQosPolicy

### class GenericDataQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE), public eprosima::fastdds::ResourceLimitedVector<fastdds::rtps::[octet](../../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)>

Class [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy), base class to transmit user data during the discovery phase. 

Subclassed by [eprosima::fastdds::dds::GroupDataQosPolicy](groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy), [eprosima::fastdds::dds::TopicDataQosPolicy](topicdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDataQosPolicy), [eprosima::fastdds::dds::UserDataQosPolicy](userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

### Public Functions

### inline GenericDataQosPolicy(const [GenericDataQosPolicy](#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicy20GenericDataQosPolicyERK20GenericDataQosPolicy) &data)

Construct from another [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy).

The resulting [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy) will have the same size limits as the input attribute

* **Parameters:**
  **data** – data to copy in the newly created object 

### inline GenericDataQosPolicy(ParameterId_t pid, const collection_type &data)

Construct from underlying collection type.

Useful to easy integration on old APIs where a traditional container was used. The resulting [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy) will always be unlimited in size

* **Parameters:**
  * **pid** – Id of the parameter 
  * **data** – data to copy in the newly created object 

### inline [GenericDataQosPolicy](#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE) &operator=(const collection_type &b)

Copies data from underlying collection type.

Useful to easy integration on old APIs where a traditional container was used. The resulting [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy) will keep the current size limit. If the input data is larger than the current limit size, the elements exceeding that maximum will be silently discarded.

* **Parameters:**
  **b** – object to be copied 
* **Returns:**
  reference to the current object. 

### inline [GenericDataQosPolicy](#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE) &operator=(const [GenericDataQosPolicy](#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE) &b)

Copies another [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy).

The resulting [GenericDataQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy) will have the same size limit as the input parameter, so all data in the input will be copied.

* **Parameters:**
  **b** – object to be copied 
* **Returns:**
  reference to the current object. 

### inline void set_max_size(std::size_t size)

Set the maximum size of the user data and reserves memory for that much.

* **Parameters:**
  **size** – new maximum size of the user data. Zero for unlimited size 

### inline const collection_type &dataVec() const

* **Returns:**
  const reference to the internal raw data. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### inline const collection_type &data_vec() const

Returns raw data vector.

* **Returns:**
  raw data as vector of octets. 

### inline collection_type &data_vec()

Returns raw data vector.

* **Returns:**
  raw data as vector of octets. 

### inline void data_vec(const collection_type &vec)

Sets raw data vector.

* **Parameters:**
  **vec** – raw data to set. 

### inline const collection_type &getValue() const

Returns raw data vector.

* **Returns:**
  raw data as vector of octets. 

### inline void setValue(const collection_type &vec)

Sets raw data vector.

* **Parameters:**
  **vec** – raw data to set.
