<a id="api-pim-datarepresentationqospolicy"></a>

<a id="datarepresentationqospolicy"></a>

# DataRepresentationQosPolicy

### class DataRepresentationQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

With multiple standard data Representations available, and vendor-specific extensions possible, DataWriters and DataReaders must be able to negotiate which data representation(s) to use. This negotiation shall occur based on [DataRepresentationQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1DataRepresentationQosPolicy).

#### NOTE
Immutable Qos Policy 

#### WARNING
If a writer’s offered representation is contained within a reader’s sequence, the offer satisfies the request and the policies are compatible. Otherwise, they are incompatible.

### Public Functions

### inline DataRepresentationQosPolicy()

Constructor. 

### virtual ~DataRepresentationQosPolicy() override = default

Destructor. 

### inline bool operator==(const [DataRepresentationQosPolicy](#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE) &b) const

Compares the given policy to check if it’s equal.

* **Parameters:**
  **b** – QoS Policy. 
* **Returns:**
  True if the policy is equal. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### std::vector<DataRepresentationId_t> m_value

List of [DataRepresentationId](datarepresentationid.md#namespaceeprosima_1_1fastdds_1_1dds_1a0700dd1b55200c30d8e673dd5ad7c3f8)

.

By default, empty list.
