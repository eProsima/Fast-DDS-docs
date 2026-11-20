<a id="python-api-pim-partitionqospolicy"></a>

<a id="partitionqospolicy"></a>

# PartitionQosPolicy

### *class* fastdds.PartitionQosPolicy(\*args)

Set of strings that introduces a logical partition among the topics visible by the Publisher and Subscriber.
A DataWriter within a Publisher only communicates with a DataReader in a Subscriber if (in addition to matching the
Topic and having compatible QoS) the Publisher and Subscriber have a common partition name string.

The empty string (“”) is considered a valid partition that is matched with other partition names using the same rules of
string matching and regular-expression matching used for any other partition name.

Notes: Mutable Qos Policy

#### begin()

Getter for the first position of the partition list

* **Return type:**
  eprosima::fastdds::dds::PartitionQosPolicy::const_iterator
* **Returns:**
  const_iterator

#### clear()

Clears list of partition names

#### empty()

Check if the set is empty

* **Return type:**
  int
* **Returns:**
  true if it is empty, false otherwise

#### end()

Getter for the end of the partition list

* **Return type:**
  eprosima::fastdds::dds::PartitionQosPolicy::const_iterator
* **Returns:**
  const_iterator

#### getNames()

Returns partition names.

* **Return type:**
  std::vector< std::string,std::allocator< std::string > >
* **Returns:**
  Vector of partition name strings.

#### max_size()

Getter for the maximum size (in bytes)

* **Return type:**
  int
* **Returns:**
  uint32_t with the maximum size

#### names(\*args)

*Overload 1:*

Returns partition names.

* **Return type:**
  std::vector< std::string,std::allocator< std::string > >
* **Returns:**
  Vector of partition name strings.

<br/>

*Overload 2:*

Overrides partition names

* **Parameters:**
  **nam** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Vector of partition name strings.

#### push_back(name)

Appends a name to the list of partition names.

* **Parameters:**
  **name** (*string*) – Name to append.

#### setNames(nam)

Overrides partition names

* **Parameters:**
  **nam** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Vector of partition name strings.

#### set_max_size(size)

Setter for the maximum size reserved for partitions (in bytes)

* **Parameters:**
  **size** (*int*) – Size to be set

#### size()

Getter for the number of partitions

* **Return type:**
  int
* **Returns:**
  uint32_t with the size

#### *property* thisown

The membership flag
