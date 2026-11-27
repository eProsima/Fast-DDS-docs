<a id="python-api-pim-statusmask"></a>

<a id="statusmask"></a>

# StatusMask

### *class* fastdds.StatusMask(\*args)

StatusMask is a bitmap or bitset field.

This bitset is used to:
- determine which listener functions to call
- set conditions in dds::core::cond::StatusCondition
- indicate status changes when calling dds::core::Entity::status_changes

#### *static* all()

Get all StatusMasks

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask all

#### *static* data_available()

get the statusmask associated with dds::core::status::data_available

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  statusmask data_available

#### *static* data_on_readers()

Get the StatusMask associated with dds::core::status::data_on_readers

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask data_on_readers

#### *static* inconsistent_topic()

Get the StatusMask associated with dds::core::status::InconsistentTopicStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask inconsistent_topic

#### is_active(status)

Checks if the status passed as parameter is 1 in the actual StatusMask
:type status: [`StatusMask`](#fastdds.StatusMask)
:param status: Status that need to be checked
:rtype: boolean
:return: true if the status is active and false if not

#### *static* liveliness_changed()

Get the StatusMask associated with dds::core::status::LivelinessChangedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask liveliness_changed

#### *static* liveliness_lost()

Get the StatusMask associated with dds::core::status::LivelinessLostStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask liveliness_lost

#### *static* none()

Get no StatusMasks

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask none

#### *static* offered_deadline_missed()

Get the StatusMask associated with dds::core::status::OfferedDeadlineMissedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask offered_deadline_missed

#### *static* offered_incompatible_qos()

Get the StatusMask associated with dds::core::status::OfferedIncompatibleQosStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask offered_incompatible_qos

#### *static* publication_matched()

Get the statusmask associated with dds::core::status::PublicationMatchedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask publication_matched

#### *static* requested_deadline_missed()

Get the StatusMask associated with dds::core::status::RequestedDeadlineMissedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask requested_deadline_missed

#### *static* requested_incompatible_qos()

Get the StatusMask associated with dds::core::status::RequestedIncompatibleQosStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask requested_incompatible_qos

#### *static* sample_lost()

Get the StatusMask associated with dds::core::status::SampleLostStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask sample_lost

#### *static* sample_rejected()

Get the StatusMask associated with dds::core::status::SampleRejectedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask sample_rejected

#### *static* subscription_matched()

Get the statusmask associated with dds::core::status::SubscriptionMatchedStatus

* **Return type:**
  [`StatusMask`](#fastdds.StatusMask)
* **Returns:**
  StatusMask subscription_matched

#### *property* thisown

The membership flag
