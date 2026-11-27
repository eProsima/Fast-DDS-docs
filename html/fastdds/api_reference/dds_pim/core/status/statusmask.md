<a id="api-pim-statusmask"></a>

<a id="statusmask"></a>

# StatusMask

### class StatusMask : public std::bitset<FASTDDS_STATUS_COUNT>

[StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) is a bitmap or bitset field. 

This bitset is used to:
* determine which listener functions to call
* set conditions in dds::core::cond::StatusCondition
* indicate status changes when calling dds::core::Entity::status_changes 

### Public Types

### typedef std::bitset<FASTDDS_STATUS_COUNT> MaskType

Convenience typedef for std::bitset<FASTDDS_STATUS_COUNT>. 

### Public Functions

### inline StatusMask()

Construct an [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) with no flags set. 

### inline explicit StatusMask(std::uint32_t mask)

Construct an [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) with an uint32_t bit mask.

* **Parameters:**
  **mask** – the bit array to initialize the bitset with 

### inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &operator<<(const [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Add given [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) bits into this [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) bitset.

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) this 

### inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &operator>>(const [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Remove given [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) bits into this [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) bitset.

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) this 

### inline bool is_active([StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) status) const

Checks if the status passed as parameter is 1 in the actual [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask). 

* **Parameters:**
  **status** – Status that need to be checked 
* **Returns:**
  true if the status is active and false if not 

### Public Static Functions

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) all()

Get all StatusMasks

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) all 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) none()

Get no StatusMasks

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) none 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) inconsistent_topic()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::InconsistentTopicStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) inconsistent_topic 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) offered_deadline_missed()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::OfferedDeadlineMissedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) offered_deadline_missed 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) requested_deadline_missed()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::RequestedDeadlineMissedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) requested_deadline_missed 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) offered_incompatible_qos()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::OfferedIncompatibleQosStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) offered_incompatible_qos 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) requested_incompatible_qos()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::RequestedIncompatibleQosStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) requested_incompatible_qos 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) sample_lost()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::SampleLostStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) sample_lost 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) sample_rejected()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::SampleRejectedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) sample_rejected 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) data_on_readers()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::data_on_readers

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) data_on_readers 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) data_available()

get the statusmask associated with dds::core::status::data_available

* **Returns:**
  statusmask data_available 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) liveliness_lost()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::LivelinessLostStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) liveliness_lost 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) liveliness_changed()

Get the [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) associated with dds::core::status::LivelinessChangedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) liveliness_changed 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) publication_matched()

Get the statusmask associated with dds::core::status::PublicationMatchedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) publication_matched 

### static inline [StatusMask](#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) subscription_matched()

Get the statusmask associated with dds::core::status::SubscriptionMatchedStatus

* **Returns:**
  [StatusMask](#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) subscription_matched 

### FASTDDS_STATUS_COUNT 16u

Number of status bits (16). An unsigned literal rather than std::size_t(16), which SWIG cannot parse.
