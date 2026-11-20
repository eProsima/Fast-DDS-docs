<a id="api-pim-durabilityqospolicy"></a>

<a id="durabilityqospolicy"></a>

# DurabilityQosPolicy

### class DurabilityQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

This policy expresses if the data should ‘outlive’ their writing time.

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline DurabilityQosPolicy()

Constructor. 

### virtual ~DurabilityQosPolicy() = default

Destructor. 

### inline fastdds::rtps::[DurabilityKind_t](../../../rtps/common/Types/DurabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps16DurabilityKind_tE) durabilityKind() const

Translates kind to rtps layer equivalent

* **Returns:**
  fastdds::rtps::DurabilityKind_t

### inline void durabilityKind(const fastdds::rtps::[DurabilityKind_t](../../../rtps/common/Types/DurabilityKind_t.md#_CPPv4N8eprosima7fastdds4rtps16DurabilityKind_tE) new_kind)

Set kind passing the rtps layer equivalent kind

* **Parameters:**
  **new_kind** – fastdds::rtps::DurabilityKind_t

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### DurabilityQosPolicyKind_t kind

DurabilityQosPolicyKind.

By default the value for DataReaders: VOLATILE_DURABILITY_QOS, for DataWriters TRANSIENT_LOCAL_DURABILITY_QOS.
