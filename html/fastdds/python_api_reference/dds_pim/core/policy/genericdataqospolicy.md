<a id="python-api-pim-genericdataqospolicy"></a>

<a id="genericdataqospolicy"></a>

# GenericDataQosPolicy

### *class* fastdds.GenericDataQosPolicy(\*args)

Class GenericDataQosPolicy, base class to transmit user data during the discovery phase.

#### clear()

Clears the QosPolicy object

#### dataVec()

* **Return type:**
  eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type
* **Returns:**
  const reference to the internal raw data.

#### data_vec(\*args)

*Overload 1:*

Returns raw data vector.

* **Return type:**
  eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type
* **Returns:**
  raw data as vector of octets.

<br/>

*Overload 2:*

Returns raw data vector.

* **Return type:**
  eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type
* **Returns:**
  raw data as vector of octets.

<br/>

*Overload 3:*

Sets raw data vector.

* **Parameters:**
  **vec** (*eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type*) – raw data to set.

#### getValue()

Returns raw data vector.

* **Return type:**
  eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type
* **Returns:**
  raw data as vector of octets.

#### resize(new_size)

#### setValue(vec)

Sets raw data vector.

* **Parameters:**
  **vec** (*eprosima::fastdds::ResourceLimitedVector< unsigned char >::collection_type*) – raw data to set.

#### set_max_size(size)

Set the maximum size of the user data and reserves memory for that much.

* **Parameters:**
  **size** (*std::size_t*) – new maximum size of the user data. Zero for unlimited size

#### *property* thisown

The membership flag
