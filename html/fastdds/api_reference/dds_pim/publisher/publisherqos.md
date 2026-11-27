<a id="api-pim-publisherqos"></a>

<a id="publisherqos"></a>

# PublisherQos

### class PublisherQos

Class [PublisherQos](#classeprosima_1_1fastdds_1_1dds_1_1PublisherQos), containing all the possible Qos that can be set for a determined [Publisher](publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher). Although these values can be set and are transmitted during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library. Please consult each of them to check for implementation details and default values. 

### Public Functions

### inline PublisherQos()

Constructor. 

### virtual ~PublisherQos() = default

Destructor. 

### inline const [PresentationQosPolicy](../core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicyE) &presentation() const

Getter for [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy)

* **Returns:**
  [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy) reference 

### inline [PresentationQosPolicy](../core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicyE) &presentation()

Getter for [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy)

* **Returns:**
  [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy) reference 

### inline void presentation(const [PresentationQosPolicy](../core/policy/presentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds21PresentationQosPolicyE) &presentation)

Setter for [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy)

* **Parameters:**
  **presentation** – [PresentationQosPolicy](../core/policy/presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy)

### inline const [PartitionQosPolicy](../core/policy/partitionqospolicy.md#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicyE) &partition() const

Getter for [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy)

* **Returns:**
  [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy) reference 

### inline [PartitionQosPolicy](../core/policy/partitionqospolicy.md#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicyE) &partition()

Getter for [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy)

* **Returns:**
  [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy) reference 

### inline void partition(const [PartitionQosPolicy](../core/policy/partitionqospolicy.md#_CPPv4N8eprosima7fastdds3dds18PartitionQosPolicyE) &partition)

Setter for [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy)

* **Parameters:**
  **partition** – [PartitionQosPolicy](../core/policy/partitionqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PartitionQosPolicy)

### inline const [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18GroupDataQosPolicyE) &group_data() const

Getter for [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy)

* **Returns:**
  [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy) reference 

### inline [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18GroupDataQosPolicyE) &group_data()

Getter for [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy)

* **Returns:**
  [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy) reference 

### inline void group_data(const [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds18GroupDataQosPolicyE) &group_data)

Setter for [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy)

* **Parameters:**
  **group_data** – [GroupDataQosPolicy](../core/policy/groupdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GroupDataQosPolicy)

### inline const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory() const

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory()

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline void entity_factory(const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory)

Setter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Parameters:**
  **entity_factory** – [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

### const [PublisherQos](#_CPPv4N8eprosima7fastdds3dds12PublisherQosE) eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT
