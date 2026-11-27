<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-core-status"></a>

<a id="status"></a>

# Status

Each [Entity](../entity/entity.md#dds-layer-core-entity) is associated with a set of `Status` objects whose values represent
the *communication status* of that Entity.
Changes on the status values occur due to communication events related to each of the entities,
e.g., when new data arrives, a new participant is discovered, or a remote endpoint is lost.
The status is decomposed into several status objects, each concerning a different aspect of the communication,
so that each of these status objects can vary independently of the others.

Changes on a status object trigger the corresponding [Listener](../entity/entity.md#dds-layer-core-entity-commonchars-listener) callbacks
that allow the Entity to inform the application about the event.
For a given status object with name `fooStatus`, the entity listener interface defines a callback
function `on_foo()` that will be called when the status changes.
Beware that some statuses have data members that are reset every time the corresponding listener is called.
The only exception to this rule is when the entity has no listener attached, so the callback cannot be called.
See the documentation of each status for details.

[Conditions and Wait-sets](../waitsets/waitsets.md#dds-layer-core-waitsets) provide the application with an alternative mechanism to make it aware of changes
on status objects, by means of a [StatusCondition](../entity/entity.md#dds-layer-core-entity-commonchars-statuscondition).
The advantage of this mechanism is that the application can wait for changes on several entities at the same time.
It will also help the determinism of your system, as the notification is not processed on an internal thread, as
it is done when using listeners.

The entities expose functions to access the value of its statuses.
For a given status with name `fooStatus`, the entity exposes a member function `get_foo()` to
access the data in its `fooStatus`.
The only exceptions are [DataOnReaders](#dds-layer-core-status-dataonreaders) and [DataAvailable](#dds-layer-core-status-dataavailable).
These getter functions return a read-only struct where all data members are public and accessible to the application.
Beware that some statuses have data members that are reset every time the getter function is called by the application.
See the documentation of each status for details.

The following subsections describe each of the status objects, their data members, and to which
Entity type they concern.
The next table offers a quick reference as well as the corresponding bit for each status in the [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE).

| Status Name                                                                             | Entity                                                                                  | Listener callback                                                                                                                                                                                                                    | Accessor                                                                                                                                                                                                               |   Bit |
|-----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
| [InconsistentTopicStatus](#dds-layer-core-status-inconsistenttopicstatus)               | [Topic](../../topic/topic.md#dds-layer-topic)                                           | [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus)                                               | [`get_inconsistent_topic_status()`](../../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5Topic29get_inconsistent_topic_statusER23InconsistentTopicStatus)                                  |     0 |
| [OfferedDeadlineMissedStatus](#dds-layer-core-status-offereddeadlinemissedstatus)       | [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)   | [`on_offered_deadline_missed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener26on_offered_deadline_missedEP10DataWriterRK27OfferedDeadlineMissedStatus)           | [`get_offered_deadline_missed_status()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter34get_offered_deadline_missed_statusER27OfferedDeadlineMissedStatus)           |     1 |
| [RequestedDeadlineMissedStatus](#dds-layer-core-status-requesteddeadlinemissedstatus)   | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_requested_deadline_missed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus)    | [`get_requested_deadline_missed_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader36get_requested_deadline_missed_statusER29RequestedDeadlineMissedStatus)    |     2 |
| [OfferedIncompatibleQosStatus](#dds-layer-core-status-offeredincompatibleqosstatus)     | [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)   | [`on_offered_incompatible_qos()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener27on_offered_incompatible_qosEP10DataWriterRK28OfferedIncompatibleQosStatus)        | [`get_offered_incompatible_qos_status()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter35get_offered_incompatible_qos_statusER28OfferedIncompatibleQosStatus)        |     5 |
| [RequestedIncompatibleQosStatus](#dds-layer-core-status-requestedincompatibleqosstatus) | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_requested_incompatible_qos()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener29on_requested_incompatible_qosEP10DataReaderRK30RequestedIncompatibleQosStatus) | [`get_requested_incompatible_qos_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader37get_requested_incompatible_qos_statusER30RequestedIncompatibleQosStatus) |     6 |
| [SampleLostStatus](#dds-layer-core-status-sampleloststatus)                             | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_sample_lost()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener14on_sample_lostEP10DataReaderRK16SampleLostStatus)                                             | [`get_sample_lost_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader22get_sample_lost_statusER16SampleLostStatus)                                            |     7 |
| [SampleRejectedStatus](#dds-layer-core-status-samplerejectedstatus)                     | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_sample_rejected()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener18on_sample_rejectedEP10DataReaderRK20SampleRejectedStatus)                                 | [`get_sample_rejected_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader26get_sample_rejected_statusER20SampleRejectedStatus)                                |     8 |
| [DataOnReaders](#dds-layer-core-status-dataonreaders)                                   | [Subscriber](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) | [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber)                                                         | N/A                                                                                                                                                                                                                    |     9 |
| [DataAvailable](#dds-layer-core-status-dataavailable)                                   | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader)                                                           | N/A                                                                                                                                                                                                                    |    10 |
| [LivelinessLostStatus](#dds-layer-core-status-livelinessloststatus)                     | [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)   | [`on_liveliness_lost()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener18on_liveliness_lostEP10DataWriterRK20LivelinessLostStatus)                                  | [`get_liveliness_lost_status()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter26get_liveliness_lost_statusER20LivelinessLostStatus)                                  |    11 |
| [LivelinessChangedStatus](#dds-layer-core-status-livelinesschangedstatus)               | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_liveliness_changed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener21on_liveliness_changedEP10DataReaderRK23LivelinessChangedStatus)                        | [`get_liveliness_changed_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader29get_liveliness_changed_statusER23LivelinessChangedStatus)                       |    12 |
| [PublicationMatchedStatus](#dds-layer-core-status-publicationmatchedstatus)             | [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)   | [`on_publication_matched()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus)                      | [`get_publication_matched_status()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4NK8eprosima7fastdds3dds10DataWriter30get_publication_matched_statusER24PublicationMatchedStatus)                     |    13 |
| [SubscriptionMatchedStatus](#dds-layer-core-status-subscriptionmatchedstatus)           | [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | [`on_subscription_matched()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE)    | [`get_subscription_matched_status()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader31get_subscription_matched_statusER25SubscriptionMatchedStatus)                 |    14 |

<a id="dds-layer-core-status-inconsistenttopicstatus"></a>

<a id="inconsistenttopicstatus"></a>

## InconsistentTopicStatus

This status changes every time an inconsistent remote Topic is discovered,
that is, one with the same name but different characteristics than the current Topic.
See [`InconsistentTopicStatus`](../../../api_reference/dds_pim/core/status/inconsistenttopicstatus.md#_CPPv4N8eprosima7fastdds3dds23InconsistentTopicStatusE).

List of status data members:

| Data Member Name                                                                                                                               | Type      |
|------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE)               | `int32_t` |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE) | `int32_t` |
* [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE):
  Total cumulative count of inconsistent Topics discovered
  since the creation of the current Topic.
  An inconsistent remote endpoint is counted only once, until it is removed.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE) since
  the last time [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus) was called or the status was read.

#### NOTE
**[Pro]** This status is only supported in *Fast DDS* **[Pro]**.
In *Fast DDS*, trying to access this status returns `NOT_SUPPORTED` and the corresponding listener is never
called.

<a id="dds-layer-core-status-dataonreaders"></a>

<a id="dataonreaders"></a>

## DataOnReaders

This status becomes active every time there is new data available for the application on any
DataReader belonging to the current Subscriber.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber) callback on the listener attached to the
DataReader.

<a id="dds-layer-core-status-dataavailable"></a>

<a id="dataavailable"></a>

## DataAvailable

This status becomes active every time there is new data available for the application on the
DataReader.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader) callback on the listener attached to the
DataReader.

<a id="dds-layer-core-status-livelinesschangedstatus"></a>

<a id="livelinesschangedstatus"></a>

## LivelinessChangedStatus

This status changes every time the liveliness status of a matched DataWriter has changed.
Either because a DataWriter that was *inactive* has become *active* or the other way around.
See [`LivelinessChangedStatus`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                                   | Type                                                                                                                                         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [`alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus11alive_countE)                         | `int32_t`                                                                                                                                    |
| [`not_alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus15not_alive_countE)                 | `int32_t`                                                                                                                                    |
| [`alive_count_change`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus18alive_count_changeE)           | `int32_t`                                                                                                                                    |
| [`not_alive_count_change`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus22not_alive_count_changeE)   | `int32_t`                                                                                                                                    |
| [`last_publication_handle`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus23last_publication_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) |
* [`alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus11alive_countE):
  Total number of currently active DataWriters.
  This count increases every time a newly matched DataWriter asserts its
  liveliness or a DataWriter that was considered not alive reasserts its
  liveliness.
  It decreases every time an active DataWriter becomes not alive, either
  because it failed to assert its liveliness or because it was deleted for any reason.
* [`not_alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus15not_alive_countE):
  Total number of matched DataWriters
  that are currently considered not alive.
  This count increases every time an active DataWriter becomes not alive
  because it fails to assert its liveliness.
  It decreases every time a DataWriter that was considered not alive
  reasserts its liveliness.
  Normal matching and unmatching of DataWriters
  does not affect this count.
* [`alive_count_change`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus18alive_count_changeE):
  The change in [`alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus11alive_countE) since
  the last time [`on_liveliness_changed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener21on_liveliness_changedEP10DataReaderRK23LivelinessChangedStatus) was called or the status was read.
  It can have positive or negative values.
* [`not_alive_count_change`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus22not_alive_count_changeE):
  The change in [`not_alive_count`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus15not_alive_countE) since
  the last time [`on_liveliness_changed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener21on_liveliness_changedEP10DataReaderRK23LivelinessChangedStatus) was called or the status was read.
  It can have positive or negative values.
* [`last_publication_handle`](../../../api_reference/dds_pim/core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatus23last_publication_handleE):
  Handle to the last DataWriter
  whose liveliness status was changed.
  If no liveliness has ever changed, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).

<a id="dds-layer-core-status-requesteddeadlinemissedstatus"></a>

<a id="requesteddeadlinemissedstatus"></a>

## RequestedDeadlineMissedStatus

This status changes every time the DataReader does not receive
data within the deadline period configured on its [DataReaderQos](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos).
See [`RequestedDeadlineMissedStatus`](../../../api_reference/dds_pim/core/status/requesteddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds29RequestedDeadlineMissedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                       | Type                                                                                                                                         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE)                   | `int32_t`                                                                                                                                    |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus18total_count_changeE)     | `int32_t`                                                                                                                                    |
| [`last_instance_handle`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus20last_instance_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE):
  Total cumulative count of missed deadlines for any instance read by the
  current DataReader.
  As the deadline period applies to each instance of the Topic independently,
  the count will will be incremented by one for each instance for which data
  was not received in the deadline period.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE) since
  the last time [`on_requested_deadline_missed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus) was called or the status was read.
  It can only have zero or positive values.
* [`last_instance_handle`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus20last_instance_handleE):
  Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).

<a id="dds-layer-core-status-requestedincompatibleqosstatus"></a>

<a id="requestedincompatibleqosstatus"></a>

## RequestedIncompatibleQosStatus

This status changes every time the DataReader finds a
DataWriter that matches the Topic and has
a common partition, but with a QoS configuration incompatible with the one defined on the
DataReader.
See [`RequestedIncompatibleQosStatus`](../../../api_reference/dds_pim/core/status/requestedicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds30RequestedIncompatibleQosStatusE).

List of status data members:

| Data Member Name                                                                                                                                                     | Type                                                                                                                                    |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE)               | `int32_t`                                                                                                                               |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus18total_count_changeE) | `int32_t`                                                                                                                               |
| [`last_policy_id`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus14last_policy_idE)         | [`QosPolicyId_t`](../../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE)             |
| [`policies`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus8policiesE)                      | [`QosPolicyCountSeq`](../../../api_reference/dds_pim/core/status/qospolicycountseq.md#_CPPv4N8eprosima7fastdds3dds17QosPolicyCountSeqE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE):
  Total cumulative count of DataWriters found
  matching the Topic and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the DataReader.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE) since
  the last time [`on_requested_incompatible_qos()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener29on_requested_incompatible_qosEP10DataReaderRK30RequestedIncompatibleQosStatus) was called or the status was read.
  It can only have zero or positive values.
* [`last_policy_id`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus14last_policy_idE):
  The policy ID of one of the policies that was found to be incompatible with the
  current DataReader.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.
* [`policies`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus8policiesE):
  A collection that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one offered by a remote DataWriter that
  matched the Topic and with a common partition.
  See [QosPolicyCountSeq](#dds-layer-core-status-qospolicycountseq) and [QosPolicyCount](#dds-layer-core-status-qospolicycount)
  for more information the information that is stored for each policy.

<a id="dds-layer-core-status-qospolicycountseq"></a>

<a id="qospolicycountseq"></a>

### QosPolicyCountSeq

Holds a [QosPolicyCount](#dds-layer-core-status-qospolicycount) for each [Policy](../policy/policy.md#dds-layer-core-policy),
indexed by its [`QosPolicyId_t`](../../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE). Therefore, the Qos Policy with ID `N` will be at position `N` in the sequence.
See [`QosPolicyCountSeq`](../../../api_reference/dds_pim/core/status/qospolicycountseq.md#_CPPv4N8eprosima7fastdds3dds17QosPolicyCountSeqE).

```c++
DataReader* reader_ =
        subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT);

// Get how many times ReliabilityQosPolicy was not compatible with a remote writer
RequestedIncompatibleQosStatus status;
reader_->get_requested_incompatible_qos_status(status);
uint32_t incompatible_reliability_count = status.policies[RELIABILITY_QOS_POLICY_ID].count;
```

<a id="dds-layer-core-status-qospolicycount"></a>

<a id="qospolicycount"></a>

### QosPolicyCount

This structure holds a counter for a policy.
See [`QosPolicyCount`](../../../api_reference/dds_pim/core/status/qospolicycount.md#_CPPv4N8eprosima7fastdds3dds14QosPolicyCountE).

List of data members:

| Data Member Name                                                                                                                    | Type                                                                                                                        |
|-------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| [`policy_id`](../../../api_reference/dds_pim/core/status/qospolicycount.md#_CPPv4N8eprosima7fastdds3dds14QosPolicyCount9policy_idE) | [`QosPolicyId_t`](../../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) |
| [`count`](../../../api_reference/dds_pim/core/status/qospolicycount.md#_CPPv4N8eprosima7fastdds3dds14QosPolicyCount5countE)         | `int32_t`                                                                                                                   |
* [`policy_id`](../../../api_reference/dds_pim/core/status/qospolicycount.md#_CPPv4N8eprosima7fastdds3dds14QosPolicyCount9policy_idE): The ID of the policy.
* [`count`](../../../api_reference/dds_pim/core/status/qospolicycount.md#_CPPv4N8eprosima7fastdds3dds14QosPolicyCount5countE): The counter value for the policy.

<a id="dds-layer-core-status-sampleloststatus"></a>

<a id="sampleloststatus"></a>

## SampleLostStatus

This status changes every time a new data sample is lost and will never be received.
See [`SampleLostStatus`](../../../api_reference/dds_pim/core/status/sampleloststatus.md#_CPPv4N8eprosima7fastdds3dds16SampleLostStatusE).

There are two different criteria for considering a sample as lost depending on the [`reliability()`](../../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos11reliabilityEv):

* When using [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE), a not yet received sample is considered lost whenever a sample with a
  greater sequence number is received.
* When using [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE), a not yet received sample is considered lost whenever the
  `DataWriter` informs, through an RTPS `HEARTBEAT` submessage, that the sample is not available anymore.

List of status data members:

| Data Member Name                                                                                                                               | Type      |
|------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE)               | `int32_t` |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE) | `int32_t` |
* [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE): Total cumulative count of lost samples under the Topic
  of the current DataReader.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE): The change in [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE) since
  the last time [`on_sample_lost()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener14on_sample_lostEP10DataReaderRK16SampleLostStatus) was called or the status was read.
  It can only be positive or zero.

<a id="dds-layer-core-status-samplerejectedstatus"></a>

<a id="samplerejectedstatus"></a>

## SampleRejectedStatus

This status changes every time an incoming data sample is rejected by the DataReader.
The reason for the rejection is defined by [SampleRejectedStatusKind](#dds-layer-core-status-samplerejectedstatuskind).
For further information see [`SampleRejectedStatus`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                       | Type                                                                                                                                                         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus11total_countE)                   | `int32_t`                                                                                                                                                    |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus18total_count_changeE)     | `int32_t`                                                                                                                                                    |
| [`last_reason`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus11last_reasonE)                   | [`SampleRejectedStatusKind`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) |
| [`last_instance_handle`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus20last_instance_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE)                 |
* [`total_count`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus11total_countE):
  Total cumulative count of rejected samples under the Topic
  of the current DataReader.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus11total_countE) since
  the last time [`on_sample_rejected()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener18on_sample_rejectedEP10DataReaderRK20SampleRejectedStatus) was called or the status was read.
  It can only be positive or zero.
* [`last_reason`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus11last_reasonE):
  The reason for rejecting the last rejected sample.
  If no sample was ever rejected, it will have value [`NOT_REJECTED`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKind12NOT_REJECTEDE).
  See [SampleRejectedStatusKind](#dds-layer-core-status-samplerejectedstatuskind) for further details.
* [`last_instance_handle`](../../../api_reference/dds_pim/core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatus20last_instance_handleE):
  Handle to the last instance whose sample was rejected.
  If no sample was ever rejected, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).

<a id="dds-layer-core-status-samplerejectedstatuskind"></a>

<a id="samplerejectedstatuskind"></a>

### SampleRejectedStatusKind

In Fast DDS, samples can be rejected due to resource limit reasons.
However, the fact that the samples are rejected does not imply that they are lost, i.e. a rejected sample may be
accepted in the future.

[`SampleRejectedStatusKind`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKindE) specifies the reason of the rejection:

* [`NOT_REJECTED`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKind12NOT_REJECTEDE) specifies that the samples were not rejected.
* [`REJECTED_BY_SAMPLES_LIMIT`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKind25REJECTED_BY_SAMPLES_LIMITE) specifies that the samples were rejected because there were not enough resources to
  stored them.
  This can happen even when there are free resources if those resources must be guaranteed to be available for
  other samples.
  This situation, which arises in the RTPS layer, occurs when there are yet to be received samples with lower
  sequence number and there is not enough resources for all of them (because [`max_samples`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy11max_samplesE) has been
  reached).
* [`REJECTED_BY_INSTANCES_LIMIT`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKind27REJECTED_BY_INSTANCES_LIMITE) specifies that the samples were rejected because there were not enough resources
  to allocate the samples’ instances.
  This situation, which arises in the DDS layer, more precisely in the in the `DataReader`’s history,
  occurs when the sample corresponds to a new instance for which the middleware should reserve resources but the
  history’s number of instances has already reached [`max_instances`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy13max_instancesE).
* [`REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT`](../../../api_reference/dds_pim/core/status/samplerejectedstatuskind.md#_CPPv4N8eprosima7fastdds3dds24SampleRejectedStatusKind38REJECTED_BY_SAMPLES_PER_INSTANCE_LIMITE) specifies that the samples were rejected because there were not
  enough resources within their instance to stored them.
  This situation, which arises in the DDS layer, more precisely in the `DataReader`’s history, occurs
  when the `DataReader` is configured with [`KEEP_ALL_HISTORY_QOS`](../../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE) and the instance’s number of samples
  has reached [`max_samples_per_instance`](../../../api_reference/dds_pim/core/policy/durabilityserviceqospolicy.md#_CPPv4N8eprosima7fastdds3dds26DurabilityServiceQosPolicy24max_samples_per_instanceE).

<a id="dds-layer-core-status-subscriptionmatchedstatus"></a>

<a id="subscriptionmatchedstatus"></a>

## SubscriptionMatchedStatus

This status changes every time the DataReader finds a DataWriter
that matches the Topic and has a common partition and a compatible QoS,
or has ceased to be matched with a DataWriter that was previously considered to be matched.
See [`SubscriptionMatchedStatus`](../../../api_reference/dds_pim/core/status/subscriptionmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds25SubscriptionMatchedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                                       | Type                                                                                                                                         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE)                                                 | `int32_t`                                                                                                                                    |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus18total_count_changeE)                                   | `int32_t`                                                                                                                                    |
| [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE)                                             | `int32_t`                                                                                                                                    |
| [`current_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus20current_count_changeE)                               | `int32_t`                                                                                                                                    |
| [`last_publication_handle`](../../../api_reference/dds_pim/core/status/subscriptionmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds25SubscriptionMatchedStatus23last_publication_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE):
  Total cumulative count of remote DataWriters
  that have been discovered publishing on the same Topic and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE) since
  the last time [`on_subscription_matched()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE) was called or the status was read.
  It can only have zero or positive values.
* [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE):
  The number of remote DataWriters
  currently matched to the DataReader.
* [`current_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus20current_count_changeE):
  The change in [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE) since
  the last time [`on_subscription_matched()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE) was called or the status was read.
  It can have positive or negative values.
* [`last_publication_handle`](../../../api_reference/dds_pim/core/status/subscriptionmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds25SubscriptionMatchedStatus23last_publication_handleE):
  Handle to the last DataWriter
  that matched the DataReader.
  If no matching ever happened, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).

<a id="dds-layer-core-status-livelinessloststatus"></a>

<a id="livelinessloststatus"></a>

## LivelinessLostStatus

This status changes every time the DataWriter failed to assert its liveliness
during the period configured on its [DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).
This means that matched DataReader entities will consider the
DataWriter as no longer *alive*.
See [`LivelinessLostStatus`](../../../api_reference/dds_pim/core/status/rtpslivelinessloststatus.md#_CPPv4N8eprosima7fastdds3dds20LivelinessLostStatusE).

List of status data members:

| Data Member Name                                                                                                                               | Type      |
|------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE)               | `int32_t` |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE) | `int32_t` |
* [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE):
  Total cumulative count of times that the DataWriter
  failed to assert its liveliness during the period configured on its [DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos),
  becoming considered not *alive*.
  This count does not change when the DataWriter is already considered not *alive* and
  simply remains not *alive* for another liveliness period.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/basestatus.md#_CPPv4N8eprosima7fastdds3dds10BaseStatus11total_countE) since
  the last time [`on_liveliness_lost()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener18on_liveliness_lostEP10DataWriterRK20LivelinessLostStatus) was called or the status was read.
  It can only have zero or positive values.

<a id="dds-layer-core-status-offereddeadlinemissedstatus"></a>

<a id="offereddeadlinemissedstatus"></a>

## OfferedDeadlineMissedStatus

This status changes every time the DataWriter fails to provide
data within the deadline period configured on its [DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).
See [`OfferedDeadlineMissedStatus`](../../../api_reference/dds_pim/core/status/offereddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds27OfferedDeadlineMissedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                       | Type                                                                                                                                         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE)                   | `int32_t`                                                                                                                                    |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus18total_count_changeE)     | `int32_t`                                                                                                                                    |
| [`last_instance_handle`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus20last_instance_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE):
  Total cumulative count of missed deadlines for any instance written by the
  current DataWriter.
  As the deadline period applies to each instance of the Topic independently,
  the count will be incremented by one for each instance for which data
  was not sent in the deadline period.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus11total_countE) since
  the last time [`on_offered_deadline_missed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener26on_offered_deadline_missedEP10DataWriterRK27OfferedDeadlineMissedStatus) was called or the status was read.
  It can only have zero or positive values.
* [`last_instance_handle`](../../../api_reference/dds_pim/core/status/deadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds20DeadlineMissedStatus20last_instance_handleE):
  Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).

<a id="dds-layer-core-status-offeredincompatibleqosstatus"></a>

<a id="offeredincompatibleqosstatus"></a>

## OfferedIncompatibleQosStatus

This status changes every time the DataWriter finds a
DataReader that matches the Topic and has
a common partition, but with a QoS configuration that is incompatible with the one defined on the
DataWriter.
See [`OfferedIncompatibleQosStatus`](../../../api_reference/dds_pim/core/status/offeredicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds28OfferedIncompatibleQosStatusE).

List of status data members:

| Data Member Name                                                                                                                                                     | Type                                                                                                                                    |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE)               | `int32_t`                                                                                                                               |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus18total_count_changeE) | `int32_t`                                                                                                                               |
| [`last_policy_id`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus14last_policy_idE)         | [`QosPolicyId_t`](../../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE)             |
| [`policies`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus8policiesE)                      | [`QosPolicyCountSeq`](../../../api_reference/dds_pim/core/status/qospolicycountseq.md#_CPPv4N8eprosima7fastdds3dds17QosPolicyCountSeqE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE):
  Total cumulative count of DataReaders found
  matching the Topic and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the DataWriter.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus11total_countE) since
  the last time [`on_offered_incompatible_qos()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener27on_offered_incompatible_qosEP10DataWriterRK28OfferedIncompatibleQosStatus) was called or the status was read.
  It can only have zero or positive values.
* [`last_policy_id`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus14last_policy_idE):
  The policy ID of one of the policies that was found to be incompatible with the
  current DataWriter.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.
* [`policies`](../../../api_reference/dds_pim/core/status/incompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds21IncompatibleQosStatus8policiesE):
  A collection that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one requested by a remote DataReader that
  matched the Topic and with a common partition.
  See [QosPolicyCountSeq](#dds-layer-core-status-qospolicycountseq) and [QosPolicyCount](#dds-layer-core-status-qospolicycount)
  for more information the information that is stored for each policy.

<a id="dds-layer-core-status-publicationmatchedstatus"></a>

<a id="publicationmatchedstatus"></a>

## PublicationMatchedStatus

This status changes every time the DataWriter finds a DataReader
that matches the Topic and has a common partition and a compatible QoS,
or has ceased to be matched with a DataReader that was previously considered to be matched.
See [`PublicationMatchedStatus`](../../../api_reference/dds_pim/core/status/publicationmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds24PublicationMatchedStatusE).

List of status data members:

| Data Member Name                                                                                                                                                                       | Type                                                                                                                                         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE)                                                 | `int32_t`                                                                                                                                    |
| [`total_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus18total_count_changeE)                                   | `int32_t`                                                                                                                                    |
| [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE)                                             | `int32_t`                                                                                                                                    |
| [`current_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus20current_count_changeE)                               | `int32_t`                                                                                                                                    |
| [`last_subscription_handle`](../../../api_reference/dds_pim/core/status/publicationmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds24PublicationMatchedStatus24last_subscription_handleE) | [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE) |
* [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE):
  Total cumulative count of remote DataReaders
  that have been discovered publishing on the same Topic and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.
* [`total_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus18total_count_changeE):
  The change in [`total_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus11total_countE) since
  the last time [`on_publication_matched()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus) was called or the status was read.
  It can only have zero or positive values.
* [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE):
  The number of remote DataReaders
  currently matched to the DataWriter.
* [`current_count_change`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus20current_count_changeE):
  The change in [`current_count`](../../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatus13current_countE) since
  the last time [`on_publication_matched()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus) was called or the status was read.
  It can have positive or negative values.
* [`last_subscription_handle`](../../../api_reference/dds_pim/core/status/publicationmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds24PublicationMatchedStatus24last_subscription_handleE):
  Handle to the last DataReader
  that matched the DataWriter.
  If no matching ever happened, it will have value [`c_InstanceHandle_Unknown`](../../../api_reference/rtps/common/InstanceHandle/c_InstanceHandle_Unknown.md#_CPPv4N8eprosima7fastdds4rtps24c_InstanceHandle_UnknownE).
