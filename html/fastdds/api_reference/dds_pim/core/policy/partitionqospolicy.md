<a id="api-pim-partitionqospolicy"></a>

<a id="partitionqospolicy"></a>

# PartitionQosPolicy

### class PartitionQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Set of strings that introduces a logical partition among the topics visible by the [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) and [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). A [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) within a [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) only communicates with a [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) in a [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) if (in addition to matching the [Topic](../../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) and having compatible QoS) the [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) and [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber) have a common partition name string.

The empty string (“”) is considered a valid partition that is matched with other partition names using the same rules of string matching and regular-expression matching used for any other partition name.

#### NOTE
Mutable Qos Policy 

### Public Functions

### inline PartitionQosPolicy()

Constructor without parameters. 

### inline PartitionQosPolicy(std::uint16_t in_length)

Constructor using Parameter length. 

* **Parameters:**
  **in_length** – Length of the parameter 

### inline PartitionQosPolicy(const [PartitionQosPolicy](#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicy18PartitionQosPolicyERK18PartitionQosPolicy) &b)

Copy constructor. 

* **Parameters:**
  **b** – Another [PartitionQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy) instance 

### virtual ~PartitionQosPolicy() = default

Destructor. 

### inline [const_iterator](#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicy14const_iteratorE) begin() const

Getter for the first position of the partition list. 

* **Returns:**
  [const_iterator](#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy_1_1const__iterator)

### inline [const_iterator](#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicy14const_iteratorE) end() const

Getter for the end of the partition list. 

* **Returns:**
  [const_iterator](#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy_1_1const__iterator)

### inline std::uint32_t size() const

Getter for the number of partitions. 

* **Returns:**
  uint32_t with the size 

### inline std::uint32_t empty() const

Check if the set is empty. 

* **Returns:**
  true if it is empty, false otherwise 

### inline void set_max_size(std::uint32_t size)

Setter for the maximum size reserved for partitions (in bytes) 

* **Parameters:**
  **size** – Size to be set 

### inline std::uint32_t max_size() const

Getter for the maximum size (in bytes) 

* **Returns:**
  uint32_t with the maximum size 

### inline void push_back(const char \*name)

Appends a name to the list of partition names.

* **Parameters:**
  **name** – Name to append. 

### inline virtual void clear() override

Clears list of partition names 

### inline const std::vector<std::string> getNames() const

Returns partition names.

* **Returns:**
  Vector of partition name strings. 

### inline void setNames(std::vector<std::string> &nam)

Overrides partition names

* **Parameters:**
  **nam** – Vector of partition name strings. 

### inline const std::vector<std::string> names() const

Returns partition names.

* **Returns:**
  Vector of partition name strings. 

### inline void names(std::vector<std::string> &nam)

Overrides partition names

* **Parameters:**
  **nam** – Vector of partition name strings. 

### class const_iterator

### Public Functions

### inline const_iterator(const fastdds::rtps::[octet](../../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*ptr)

Constructor using a pointer. 

* **Parameters:**
  **ptr** – Pointer to be set
