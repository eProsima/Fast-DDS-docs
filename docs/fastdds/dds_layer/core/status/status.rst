.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_core_status:

Status
======

Each :ref:`dds_layer_core_entity` is associated with a set of :class:`Status` objects whose values represent
the *communication status* of that Entity.
Changes on the status values occur due to communication events related to each of the entities,
e.g., when new data arrives, a new participant is discovered, or a remote endpoint is lost.
The status is decomposed into several status objects, each concerning a different aspect of the communication,
so that each of these status objects can vary independently of the others.

Changes on a status object trigger the corresponding :ref:`dds_layer_core_entity_commonchars_listener` callbacks
that allow the Entity to inform the application about the event.
For a given status object with name :class:`fooStatus`, the entity listener interface defines a callback
function :func:`on_foo` that will be called when the status changes.
Beware that some statuses have data members that are reset every time the corresponding listener is called.
The only exception to this rule is when the entity has no listener attached, so the callback cannot be called.
See the documentation of each status for details.

The entities expose functions to access the value of its statuses.
For a given status with name :class:`fooStatus`, the entity exposes a member function :func:`get_foo` to
access the data in its :class:`fooStatus`.
The only exceptions are :ref:`dds_layer_core_status_dataOnReaders` and :ref:`dds_layer_core_status_dataAvailable`.
These getter functions return a read-only struct where all data members are public and accessible to the application.
Beware that some statuses have data members that are reset every time the getter function is called by the application.
See the documentation of each status for details.

The following subsections describe each of the status objects, their data members, and to which
Entity type they concern.
The next table can be used as a quick reference too.

.. list-table::
   :header-rows: 1

   * - Status Name
     - Entity
     - Listener callback
     - Accessor
   * - |InconsistentTopicStatus|
     - |Topic|
     - |TopicListener::on_inconsistent_topic-api|
     - |Topic::get_inconsistent_topic_status-api|
   * - |DataOnReaders|
     - |Subscriber|
     - |SubscriberListener::on_data_on_readers-api|
     - N/A
   * - |DataAvailable|
     - |DataReader|
     - |DataReaderListener::on_data_available-api|
     - N/A
   * - |LivelinessChangedStatus|
     - |DataReader|
     - |DataReaderListener::on_liveliness_changed-api|
     - |DataReader::get_liveliness_changed_status-api|
   * - |RequestedDeadlineMissedStatus|
     - |DataReader|
     - |DataReaderListener::on_requested_deadline_missed-api|
     - |DataReader::get_requested_deadline_missed_status-api|
   * - |RequestedIncompatibleQosStatus|
     - |DataReader|
     - |DataReaderListener::on_requested_incompatible_qos-api|
     - |DataReader::get_requested_incompatible_qos_status-api|
   * - |SampleLostStatus|
     - |DataReader|
     - |DataReaderListener::on_sample_lost-api|
     - |DataReader::get_sample_lost_status-api|
   * - |SampleRejectedStatus|
     - |DataReader|
     - |DataReaderListener::on_sample_rejected-api|
     - |DataReader::get_sample_rejected_status-api|
   * - |SubscriptionMatchedStatus|
     - |DataReader|
     - |DataReaderListener::on_subscription_matched-api|
     - |DataReader::get_subscription_matched_status-api|
   * - |LivelinessLostStatus|
     - |DataWriter|
     - |DataWriterListener::on_liveliness_lost-api|
     - |DataWriter::get_liveliness_lost_status-api|
   * - |OfferedDeadlineMissedStatus|
     - |DataWriter|
     - |DataWriterListener::on_offered_deadline_missed-api|
     - |DataWriter::get_offered_deadline_missed_status-api|
   * - |OfferedIncompatibleQosStatus|
     - |DataWriter|
     - |DataWriterListener::on_offered_incompatible_qos-api|
     - |DataWriter::get_offered_incompatible_qos_status-api|
   * - |PublicationMatchedStatus|
     - |DataWriter|
     - |DataWriterListener::on_publication_matched-api|
     - |DataWriter::get_publication_matched_status-api|


.. _dds_layer_core_status_inconsistentTopicStatus:

InconsistentTopicStatus
-----------------------

This status changes every time an inconsistent remote Topic is discovered,
that is, one with the same name but different characteristics than the current Topic.
See |InconsistentTopicStatus-api|.

List of status data members:

+--------------------------------------------------------------------------------------------------------+-------------+
| Data Member Name                                                                                       | Type        |
+========================================================================================================+=============+
| |BaseStatus::total_count-api|                                                                          | ``int32_t`` |
+--------------------------------------------------------------------------------------------------------+-------------+
| |BaseStatus::total_count_change-api|                                                                   | ``int32_t`` |
+--------------------------------------------------------------------------------------------------------+-------------+

* |BaseStatus::total_count-api|:
  Total cumulative count of inconsistent Topics discovered
  since the creation of the current Topic.

* |BaseStatus::total_count_change-api|:
  The change in |BaseStatus::total_count-api| since
  the last time |TopicListener::on_inconsistent_topic-api| was called or the status was read.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_dataOnReaders:

DataOnReaders
-------------

This status becomes active every time there is new data available for the application on any
DataReader belonging to the current Subscriber.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the |SubscriberListener::on_data_on_readers-api| callback on the listener attached to the
DataReader.



.. _dds_layer_core_status_dataAvailable:

DataAvailable
-------------

This status becomes active every time there is new data available for the application on the
DataReader.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the |DataReaderListener::on_data_available-api| callback on the listener attached to the
DataReader.


.. _dds_layer_core_status_livelinessChangedStatus:

LivelinessChangedStatus
-----------------------

This status changes every time the liveliness status of a matched DataWriter has changed.
Either because a DataWriter that was *inactive* has become *active* or the other way around.
See |LivelinessChangedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |LivelinessChangedStatus::alive_count-api|                                 | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |LivelinessChangedStatus::not_alive_count-api|                             | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |LivelinessChangedStatus::alive_count_change-api|                          | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |LivelinessChangedStatus::not_alive_count_change-api|                      | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |LivelinessChangedStatus::last_publication_handle-api|                     | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |LivelinessChangedStatus::alive_count-api|:
  Total number of currently active DataWriters.
  This count increases every time a newly matched DataWriter asserts its
  liveliness or a DataWriter that was considered not alive reasserts its
  liveliness.
  It decreases every time an active DataWriter becomes not alive, either
  because it failed to asserts its liveliness or because it was deleted for any reason.

* |LivelinessChangedStatus::not_alive_count-api|:
  Total number of matched DataWriters
  that are currently considered not alive.
  This count increases every time an active DataWriter becomes not alive
  because it fails to assert its liveliness.
  It decreases every time a DataWriter that was considered not alive
  reasserts its liveliness.
  Normal matching and unmatching of DataWriters
  does not affect this count.

* |LivelinessChangedStatus::alive_count_change-api|:
  The change in |LivelinessChangedStatus::alive_count-api| since
  the last time |DataReaderListener::on_liveliness_changed-api| was called or the status was read.
  It can have positive or negative values.

* |LivelinessChangedStatus::not_alive_count_change-api|:
  The change in |LivelinessChangedStatus::not_alive_count-api| since
  the last time |DataReaderListener::on_liveliness_changed-api| was called or the status was read.
  It can have positive or negative values.

* |LivelinessChangedStatus::last_publication_handle-api|:
  Handle to the last DataWriter
  whose liveliness status was changed.
  If no liveliness has ever changed, it will have value ``c_InstanceHandle_Unknown``.


.. _dds_layer_core_status_requestedDeadlineMissedStatus:

RequestedDeadlineMissedStatus
-----------------------------

This status changes every time the DataReader does not receive
data within the deadline period configured on its :ref:`dds_layer_subscriber_dataReaderQos`.
See |RequestedDeadlineMissedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |DeadlineMissedStatus::total_count-api|                                    | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |DeadlineMissedStatus::total_count_change-api|                             | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |DeadlineMissedStatus::last_instance_handle-api|                           | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |DeadlineMissedStatus::total_count-api|:
  Total cumulative count of missed deadlines for any instance read by the
  current DataReader.
  As the deadline period applies to each instance of the Topic independently,
  the count will will be incremented by one for each instance for which data
  was not received in the deadline period.

* |DeadlineMissedStatus::total_count_change-api|:
  The change in |DeadlineMissedStatus::total_count-api| since
  the last time |DataReaderListener::on_requested_deadline_missed-api| was called or the status was read.
  It can only have zero or positive values.

* |DeadlineMissedStatus::last_instance_handle-api|:
  Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value ``c_InstanceHandle_Unknown``.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_requestedIncompatibleQosStatus:

RequestedIncompatibleQosStatus
------------------------------

This status changes every time the DataReader finds a
DataWriter that matches the Topic and has
a common partition, but with a QoS configuration incompatible with the one defined on the
DataReader.
See |RequestedIncompatibleQosStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |IncompatibleQosStatus::total_count-api|                                   | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::total_count_change-api|                            | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::last_policy_id-api|                                | |QosPolicyId_t-api|                     |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::policies-api|                                      | |QosPolicyCountSeq-api|                 |
+----------------------------------------------------------------------------+-----------------------------------------+

* |IncompatibleQosStatus::total_count-api|:
  Total cumulative count of DataWriters found
  matching the Topic and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the DataReader.

* |IncompatibleQosStatus::total_count_change-api|:
  The change in |IncompatibleQosStatus::total_count-api| since
  the last time |DataReaderListener::on_requested_incompatible_qos-api| was called or the status was read.
  It can only have zero or positive values.

* |IncompatibleQosStatus::last_policy_id-api|:
  The policy ID of one of the policies that was found to be incompatible with the
  current DataReader.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.

* |IncompatibleQosStatus::policies-api|:
  A collection that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one offered by a remote DataWriter that
  matched the Topic and with a common partition.
  See :ref:`dds_layer_core_status_qosPolicyCountSeq` and :ref:`dds_layer_core_status_qosPolicyCount`
  for more information the information that is stored for each policy.


.. _dds_layer_core_status_qosPolicyCountSeq:

QosPolicyCountSeq
^^^^^^^^^^^^^^^^^

Holds a :ref:`dds_layer_core_status_qosPolicyCount` for each :ref:`dds_layer_core_policy`,
indexed by its |QosPolicyId_t-api|. Therefore, the Qos Policy with ID ``N`` will be at position ``N`` in the sequence.
See |QosPolicyCountSeq-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_QOS_POLICY_COUNT_SEQ
   :end-before: //!


.. _dds_layer_core_status_qosPolicyCount:

QosPolicyCount
^^^^^^^^^^^^^^

This structure holds a counter for a policy.
See |QosPolicyCount-api|.

List of data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |QosPolicyCount::policy_id-api|                                            | |QosPolicyId_t-api|                     |
+----------------------------------------------------------------------------+-----------------------------------------+
| |QosPolicyCount::count-api|                                                | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+

* |QosPolicyCount::policy_id-api|: The ID of the policy.

* |QosPolicyCount::count-api|: The counter value for the policy.


.. _dds_layer_core_status_sampleLostStatus:

SampleLostStatus
----------------

This status changes every time a new data sample is lost and will never be received.
See |SampleLostStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |BaseStatus::total_count-api|                                              | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |BaseStatus::total_count_change-api|                                       | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+

* |BaseStatus::total_count-api|: Total cumulative count of lost samples under the Topic
  of the current DataReader.

* |BaseStatus::total_count_change-api|: The change in |BaseStatus::total_count-api| since
  the last time |DataReaderListener::on_sample_lost-api| was called or the status was read.
  It can only be positive or zero.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_sampleRejectedStatus:

SampleRejectedStatus
--------------------

This status changes every time an incoming data sample is rejected by the DataReader.
The reason for the rejection is stored as a :ref:`dds_layer_core_status_sampleRejectedStatusKind`.
See |SampleRejectedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |SampleRejectedStatus::total_count-api|                                    | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |SampleRejectedStatus::total_count_change-api|                             | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |SampleRejectedStatus::last_reason-api|                                    | |SampleRejectedStatusKind-api|          |
+----------------------------------------------------------------------------+-----------------------------------------+
| |SampleRejectedStatus::last_instance_handle-api|                           | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |SampleRejectedStatus::total_count-api|:
  Total cumulative count of rejected samples under the Topic
  of the current DataReader.

* |SampleRejectedStatus::total_count_change-api|:
  The change in |SampleRejectedStatus::total_count-api| since
  the last time |DataReaderListener::on_sample_rejected-api| was called or the status was read.
  It can only be positive or zero.

* |SampleRejectedStatus::last_reason-api|:
  The reason for rejecting the last rejected sample.
  If no sample was ever rejected, it will have value |NOT_REJECTED|.
  See :ref:`dds_layer_core_status_sampleRejectedStatusKind` for further details.

* |SampleRejectedStatus::last_instance_handle-api|:
  Handle to the last instance whose sample was rejected.
  If no sample was ever rejected, it will have value ``c_InstanceHandle_Unknown``.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_sampleRejectedStatusKind:

SampleRejectedStatusKind
^^^^^^^^^^^^^^^^^^^^^^^^

There are four possible values (see |SampleRejectedStatusKind-api|):

* |NOT_REJECTED|:
  It means there have been no rejections so far on this DataReader.
  The rejection reason will have this value only while the total count of rejections is zero.
* |REJECTED_BY_INSTANCES_LIMIT|:
  The sample was rejected because the
  :ref:`max_instances<resourcelimitsqospolicy>` limit was reached.
* |REJECTED_BY_SAMPLES_LIMIT|:
  The sample was rejected because the
  :ref:`max_samples<resourcelimitsqospolicy>` limit was reached.
* |REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT|:
  The sample was rejected because the
  :ref:`max_samples_per_instance<resourcelimitsqospolicy>` limit was reached.


.. _dds_layer_core_status_subscriptionMatchedStatus:

SubscriptionMatchedStatus
-------------------------

This status changes every time the DataReader finds a DataWriter
that matches the Topic and has a common partition and a compatible QoS,
or has ceased to be matched with a DataWriter that was previously considered to be matched.
See |SubscriptionMatchedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |MatchedStatus::total_count-api|                                           | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::total_count_change-api|                                    | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::current_count-api|                                         | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::current_count_change-api|                                  | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |SubscriptionMatchedStatus::last_publication_handle-api|                   | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |MatchedStatus::total_count-api|:
  Total cumulative count of remote DataWriters
  that have been discovered publishing on the same Topic and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.

* |MatchedStatus::total_count_change-api|:
  The change in |MatchedStatus::total_count-api| since
  the last time |DataReaderListener::on_subscription_matched-api| was called or the status was read.
  It can only have zero or positive values.

* |MatchedStatus::current_count-api|:
  The number of remote DataWriters
  currently matched to the DataReader.

* |MatchedStatus::current_count_change-api|:
  The change in |MatchedStatus::current_count-api| since
  the last time |DataReaderListener::on_subscription_matched-api| was called or the status was read.
  It can have positive or negative values.

* |SubscriptionMatchedStatus::last_publication_handle-api|:
  Handle to the last DataWriter
  that matched the DataReader.
  If no matching ever happened, it will have value |c_InstanceHandle_Unknown-api|.


.. _dds_layer_core_status_livelinessLostStatus:

LivelinessLostStatus
--------------------

This status changes every time the DataWriter failed to assert its liveliness
during the period configured on its :ref:`dds_layer_publisher_dataWriterQos`.
This means that matched DataReader entities will consider the
DataWriter as no longer *alive*.
See |LivelinessLostStatus-api|.


List of status data members:

+--------------------------------------------------------------------------------------------------------+-------------+
| Data Member Name                                                                                       | Type        |
+========================================================================================================+=============+
| |BaseStatus::total_count-api|                                                                          | ``int32_t`` |
+--------------------------------------------------------------------------------------------------------+-------------+
| |BaseStatus::total_count_change-api|                                                                   | ``int32_t`` |
+--------------------------------------------------------------------------------------------------------+-------------+

* |BaseStatus::total_count-api|:
  Total cumulative count of times that the DataWriter
  failed to assert its liveliness during the period configured on its :ref:`dds_layer_publisher_dataWriterQos`,
  becoming considered not *alive*.
  This count does not change when the DataWriter is already considered not *alive* and
  simply remains not *alive* for another liveliness period.

* |BaseStatus::total_count_change-api|:
  The change in |BaseStatus::total_count-api| since
  the last time |DataWriterListener::on_liveliness_lost-api| was called or the status was read.
  It can only have zero or positive values.


.. _dds_layer_core_status_offeredDeadlineMissedStatus:

OfferedDeadlineMissedStatus
---------------------------

This status changes every time the DataWriter fails to provide
data within the deadline period configured on its :ref:`dds_layer_publisher_dataWriterQos`.
See |OfferedDeadlineMissedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |DeadlineMissedStatus::total_count-api|                                    | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |DeadlineMissedStatus::total_count_change-api|                             | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |DeadlineMissedStatus::last_instance_handle-api|                           | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |DeadlineMissedStatus::total_count-api|:
  Total cumulative count of missed deadlines for any instance written by the
  current DataWriter.
  As the deadline period applies to each instance of the Topic independently,
  the count will will be incremented by one for each instance for which data
  was not sent in the deadline period.

* |DeadlineMissedStatus::total_count_change-api|:
  The change in |DeadlineMissedStatus::total_count-api| since
  the last time |DataWriterListener::on_offered_deadline_missed-api| was called or the status was read.
  It can only have zero or positive values.

* |DeadlineMissedStatus::last_instance_handle-api|:
  Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value |c_InstanceHandle_Unknown-api|.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_offeredIncompatibleQosStatus:

OfferedIncompatibleQosStatus
----------------------------

This status changes every time the DataWriter finds a
DataReader that matches the Topic and has
a common partition, but with a QoS configuration that is incompatible with the one defined on the
DataWriter.
See |OfferedIncompatibleQosStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |IncompatibleQosStatus::total_count-api|                                   | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::total_count_change-api|                            | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::last_policy_id-api|                                | |QosPolicyId_t-api|                     |
+----------------------------------------------------------------------------+-----------------------------------------+
| |IncompatibleQosStatus::policies-api|                                      | |QosPolicyCountSeq-api|                 |
+----------------------------------------------------------------------------+-----------------------------------------+

* |IncompatibleQosStatus::total_count-api|:
  Total cumulative count of DataReaders found
  matching the Topic and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the DataWriter.

* |IncompatibleQosStatus::total_count_change-api|:
  The change in |IncompatibleQosStatus::total_count-api| since
  the last time |DataWriterListener::on_offered_incompatible_qos-api| was called or the status was read.
  It can only have zero or positive values.

* |IncompatibleQosStatus::last_policy_id-api|:
  The policy ID of one of the policies that was found to be incompatible with the
  current DataWriter.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.

* |IncompatibleQosStatus::policies-api|:
  A collection that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one requested by a remote DataReader that
  matched the Topic and with a common partition.
  See :ref:`dds_layer_core_status_qosPolicyCountSeq` and :ref:`dds_layer_core_status_qosPolicyCount`
  for more information the information that is stored for each policy.


.. _dds_layer_core_status_publicationMatchedStatus:

PublicationMatchedStatus
------------------------

This status changes every time the DataWriter finds a DataReader
that matches the Topic and has a common partition and a compatible QoS,
or has ceased to be matched with a DataReader that was previously considered to be matched.
See |PublicationMatchedStatus-api|.

List of status data members:

+----------------------------------------------------------------------------+-----------------------------------------+
| Data Member Name                                                           | Type                                    |
+============================================================================+=========================================+
| |MatchedStatus::total_count-api|                                           | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::total_count_change-api|                                    | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::current_count-api|                                         | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |MatchedStatus::current_count_change-api|                                  | ``int32_t``                             |
+----------------------------------------------------------------------------+-----------------------------------------+
| |PublicationMatchedStatus::last_subscription_handle-api|                   | |InstanceHandle_t-api|                  |
+----------------------------------------------------------------------------+-----------------------------------------+

* |MatchedStatus::total_count-api|:
  Total cumulative count of remote DataReaders
  that have been discovered publishing on the same Topic and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.

* |MatchedStatus::total_count_change-api|:
  The change in |MatchedStatus::total_count-api| since
  the last time |DataWriterListener::on_publication_matched-api| was called or the status was read.
  It can only have zero or positive values.

* |MatchedStatus::current_count-api|:
  The number of remote DataReaders
  currently matched to the DataWriter.

* |MatchedStatus::current_count_change-api|:
  The change in |MatchedStatus::current_count-api| since
  the last time |DataWriterListener::on_publication_matched-api| was called or the status was read.
  It can have positive or negative values.

* |PublicationMatchedStatus::last_subscription_handle-api|:
  Handle to the last DataReader
  that matched the DataWriter.
  If no matching ever happened, it will have value |c_InstanceHandle_Unknown-api|.



