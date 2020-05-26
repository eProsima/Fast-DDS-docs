.. _dds_layer_core_status:

Status
======

Each :ref:`dds_layer_core_entity` is associated with a set of :class:`Status` objects whose values represent
the *communication status* of that :ref:`dds_layer_core_entity`.
Changes on the status values occur due to communication events related to each of the entities,
e.g., when new data arrives, a new participant is discovered, or a remote endpoint is lost.
The status is decomposed into several status objects, each concerning a different aspect of the communication,
so that each of these status objects can vary independently of the others.

Changes on a status object trigger the corresponding :ref:`dds_layer_core_entity_commonchars_listener` callbacks
that allow the :ref:`dds_layer_core_entity` to inform the application about the event.
For a given status object with name :class:`fooStatus`, the entity listener interface defines a callback
function :func:`on_foo` that will be called when the status changes.
The only exceptions are :ref:`dds_layer_core_status_dataOnReaders` and :ref:`dds_layer_core_status_dataAvailable`.
Beware that some statuses have data members that are reset every time the corresponding listener is called.
The only exception to this rule is when the entity has no listener attached, so the callback cannot be called.
See the documentation of each status for details.

The entities expose functions to access the value of its statuses.
For a given status with name :class:`fooStatus`, the entity exposes a member function :func:`get_foo` to
access the data in its :class:`fooStatus`.
These getter functions return a read-only struct where all data members are public and accessible to the application.
Beware that some statuses have data members that are reset every time the getter function is called by the application.
See the documentation of each status for details.

The following subsections describe each of the status objects, their data members, and to which
:ref:`dds_layer_core_entity` type they concern.
The next table can be used as a quick reference too.

.. |InconsistentTopicStatus| replace:: :ref:`dds_layer_core_status_inconsistentTopicStatus`
.. |DataOnReaders| replace:: :ref:`dds_layer_core_status_dataOnReaders`
.. |DataAvailable| replace:: :ref:`dds_layer_core_status_dataAvailable`
.. |LivelinessChangedStatus| replace:: :ref:`dds_layer_core_status_livelinessChangedStatus`
.. |RequestedDeadlineMissedStatus| replace:: :ref:`dds_layer_core_status_requestedDeadlineMissedStatus`
.. |RequestedIncompatibleQosStatus| replace:: :ref:`dds_layer_core_status_requestedIncompatibleQosStatus`
.. |SampleLostStatus| replace:: :ref:`dds_layer_core_status_sampleLostStatus`
.. |SampleRejectedStatus| replace:: :ref:`dds_layer_core_status_sampleRejectedStatus`
.. |SubscriptionMatchedStatus| replace:: :ref:`dds_layer_core_status_subscriptionMatchedStatus`
.. |LivelinessLostStatus| replace:: :ref:`dds_layer_core_status_livelinessLostStatus`
.. |OfferedDeadlineMissedStatus| replace:: :ref:`dds_layer_core_status_offeredDeadlineMissedStatus`
.. |OfferedIncompatibleQosStatus| replace:: :ref:`dds_layer_core_status_offeredIncompatibleQosStatus`
.. |PublicationMatchedStatus| replace:: :ref:`dds_layer_core_status_publicationMatchedStatus`

.. |Topic| replace:: :ref:`dds_layer_topic_topic`
.. |Participant| replace:: :ref:`_ds_layer_domainParticipant`
.. |Subscriber| replace:: :ref:`dds_layer_subscriber_subscriber`
.. |Publisher| replace:: :ref:`dds_layer_publisher_publisher`
.. |DataReader| replace:: :ref:`dds_layer_subscriber_dataReader`
.. |DataWriter| replace:: :ref:`dds_layer_publisher_dataWriter`

.. |on_inconsistent_topic| replace:: :cpp:func:`on_inconsistent_topic<eprosima::fastdds::dds::TopicListener::on_inconsistent_topic>`
.. |on_data_on_readers| replace:: :cpp:func:`on_data_on_readers<eprosima::fastdds::dds::SubscriberListener::on_data_on_readers>`
.. |on_data_available| replace:: :cpp:func:`on_data_available<eprosima::fastdds::dds::DataReaderListener::on_data_available>`
.. |on_liveliness_changed| replace:: :cpp:func:`on_liveliness_changed<eprosima::fastdds::dds::DataReaderListener::on_liveliness_changed>`
.. |on_requested_deadline_missed| replace:: :cpp:func:`on_requested_deadline_missed<eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`
.. |on_requested_incompatible_qos| replace:: :cpp:func:`on_requested_incompatible_qos<eprosima::fastdds::dds::DataReaderListener::on_requested_incompatible_qos>`
.. |on_sample_lost| replace:: :cpp:func:`on_sample_lost<eprosima::fastdds::dds::DataReaderListener::on_sample_lost>`
.. |on_sample_rejected| replace:: :cpp:func:`on_sample_rejected<eprosima::fastdds::dds::DataReaderListener::on_sample_rejected>`
.. |on_subscription_matched| replace:: :cpp:func:`on_suscription_matched<eprosima::fastdds::dds::DataReaderListener::on_subscription_matched>`
.. |on_liveliness_lost| replace:: :cpp:func:`on_liveliness_lost<eprosima::fastdds::dds::DataWriterListener::on_liveliness_lost>`
.. |on_offered_deadline_missed| replace:: :cpp:func:`on_offered_deadline_missed<eprosima::fastdds::dds::DataWriterListener::on_offered_deadline_missed>`
.. |on_offered_incompatible_qos| replace:: :cpp:func:`on_offered_incompatible_qos<eprosima::fastdds::dds::DataWriterListener::on_offered_incompatible_qos>`
.. |on_publication_matched| replace:: :cpp:func:`on_publication_matched<eprosima::fastdds::dds::DataWriterListener::on_publication_matched>`

+----------------------------------+--------------+---------------------------------+
| Status Name                      | Entity       | Listener callback               |
+==================================+==============+=================================+
| |InconsistentTopicStatus|        | |Topic|      | |on_inconsistent_topic|         |
+----------------------------------+--------------+---------------------------------+
| |DataOnReaders|                  | |Subscriber| | |on_data_on_readers|            |
+----------------------------------+--------------+---------------------------------+
| |DataAvailable|                  | |DataReader| | |on_data_available|             |
+----------------------------------+--------------+---------------------------------+
| |LivelinessChangedStatus|        | |DataReader| | |on_liveliness_changed|         |
+----------------------------------+--------------+---------------------------------+
| |RequestedDeadlineMissedStatus|  | |DataReader| | |on_requested_deadline_missed|  |
+----------------------------------+--------------+---------------------------------+
| |RequestedIncompatibleQosStatus| | |DataReader| | |on_requested_incompatible_qos| |
+----------------------------------+--------------+---------------------------------+
| |SampleLostStatus|               | |DataReader| | |on_sample_lost|                |
+----------------------------------+--------------+---------------------------------+
| |SampleRejectedStatus|           | |DataReader| | |on_sample_rejected|            |
+----------------------------------+--------------+---------------------------------+
| |SubscriptionMatchedStatus|      | |DataReader| | |on_subscription_matched|       |
+----------------------------------+--------------+---------------------------------+
| |LivelinessLostStatus|           | |DataWriter| | |on_liveliness_lost|            |
+----------------------------------+--------------+---------------------------------+
| |OfferedDeadlineMissedStatus|    | |DataWriter| | |on_offered_deadline_missed|    |
+----------------------------------+--------------+---------------------------------+
| |OfferedIncompatibleQosStatus|   | |DataWriter| | |on_offered_incompatible_qos|   |
+----------------------------------+--------------+---------------------------------+
| |PublicationMatchedStatus|       | |DataWriter| | |on_publication_matched|        |
+----------------------------------+--------------+---------------------------------+



.. _dds_layer_core_status_inconsistentTopicStatus:

InconsistentTopicStatus
-----------------------

This status changes every time an inconsistent remote :ref:`dds_layer_topic_topic` is discovered,
that is, one with the same name but different characteristics than the current :ref:`dds_layer_topic_topic`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of inconsistent :ref:`Topics<dds_layer_topic_topic>` discovered
  since the creation of the current :ref:`dds_layer_topic_topic`.

* **total_count_change**: The change in **total_count** since
  the last time |on_inconsistent_topic| was called or the status was read.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_dataOnReaders:

DataOnReaders
-------------

This status becomes active every time there is new data available for the application on any
:ref:`dds_layer_subscriber_dataReader` belonging to the current :ref:`dds_layer_subscriber_subscriber`.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the |on_data_on_readers| callback on the listener attached to the
:ref:`dds_layer_subscriber_dataReader`.



.. _dds_layer_core_status_dataAvailable:

DataAvailable
-------------

This status becomes active every time there is new data available for the application on the
:ref:`dds_layer_subscriber_dataReader`.
There is no getter function to access this status, as it does not keep track of any information related to the
data itself.
Its only purpose is to trigger the |on_data_available| callback on the listener attached to the
:ref:`dds_layer_subscriber_dataReader`.


.. _dds_layer_core_status_livelinessChangedStatus:

LivelinessChangedStatus
-----------------------

This status changes every time the liveliness status of a matched :ref:`dds_layer_publisher_dataWriter` has changed.
Either because a :ref:`dds_layer_publisher_dataWriter` that was *inactive* has become *active* or the other way around.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| alive_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| not_alive_count          | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| alive_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| not_alive_count_change   | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_publication_handle  | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **alive_count**: Total number of currently active :ref:`DataWriters<dds_layer_publisher_dataWriter>`.
  This count increases every time a newly matched :ref:`dds_layer_publisher_dataWriter` asserts its
  liveliness or a :ref:`dds_layer_publisher_dataWriter` that was considered not alive reasserts its
  liveliness.
  It decreases every time an active :ref:`dds_layer_publisher_dataWriter` becomes not alive, either
  because it failed to asserts its liveliness or because it was deleted for any reason.

* **not_alive_count**: Total number of matched :ref:`DataWriters<dds_layer_publisher_dataWriter>`
  that are currently considered not alive.
  This count increases every time an active :ref:`dds_layer_publisher_dataWriter` becomes not alive
  because it fails to assert its liveliness.
  It decreases every time a :ref:`dds_layer_publisher_dataWriter` that was considered not alive
  reasserts its liveliness.
  Normal matching and unmatching of :ref:`DataWriters<dds_layer_publisher_dataWriter>`
  does not affect this count.

* **alive_count_change**: The change in **alive_count** since
  the last time |on_liveliness_changed| was called or the status was read.
  It can have positive or negative values.

* **not_alive_count_change**: The change in **not_alive_count** since
  the last time |on_liveliness_changed| was called or the status was read.
  It can have positive or negative values.

* **last_publication_handle**: Handle to the last :ref:`dds_layer_publisher_dataWriter`
  whose liveliness status was changed.
  If no liveliness has ever changed, it will have value ``c_InstanceHandle_Unknown``.


.. _dds_layer_core_status_requestedDeadlineMissedStatus:

RequestedDeadlineMissedStatus
-----------------------------

This status changes every time the :ref:`dds_layer_subscriber_dataReader` does not receive
data within the deadline period configured on its :ref:`dds_layer_subscriber_dataReaderQos`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_instance_handle     | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of missed deadlines for any instance read by the
  current :ref:`dds_layer_subscriber_dataReader`.
  As the deadline period applies to each instance of the :ref:`dds_layer_topic_topic` independently,
  the count will will be incremented by one for each instance for which data
  was not received in the deadline period.

* **total_count_change**: The change in **total_count** since
  the last time |on_requested_deadline_missed| was called or the status was read.
  It can only have zero or positive values.

* **last_instance_handle**: Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value ``c_InstanceHandle_Unknown``.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_requestedIncompatibleQosStatus:

RequestedIncompatibleQosStatus
------------------------------

This status changes every time the :ref:`dds_layer_subscriber_dataReader` finds a
:ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
a common partition, but with a QoS configuration incompatible with the one defined on the
:ref:`dds_layer_subscriber_dataReader`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_policy_id           | ``uint32_t``                            |
+--------------------------+-----------------------------------------+
| policies                 | ``std::vector<QosPolicyCount>``         |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of :ref:`DataWriters<dds_layer_publisher_dataWriter>` found
  matching the :ref:`dds_layer_topic_topic` and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the :ref:`dds_layer_subscriber_dataReader`.

* **total_count_change**: The change in **total_count** since
  the last time |on_requested_incompatible_qos| was called or the status was read.
  It can only have zero or positive values.

* **last_policy_id**: The policy ID of one of the policies that was found to be incompatible with the
  current :ref:`dds_layer_subscriber_dataReader`.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.

* **policies**: A list that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one offered by a remote :ref:`dds_layer_publisher_dataWriter` that
  matched the :ref:`dds_layer_topic_topic` and with a common partition.
  See :ref:`dds_layer_core_status_qosPolicyCount` for more information the information that is stored for each policy.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_qosPolicyCount:

QosPolicyCount
^^^^^^^^^^^^^^

This structure holds a counter for a policy.

List of data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| policy_id                | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| count                    | ``int32_t``                             |
+--------------------------+-----------------------------------------+

* **policy_id**: The ID of the policy.

* **count**: The counter value for the policy.


.. _dds_layer_core_status_sampleLostStatus:

SampleLostStatus
----------------

This status changes every time a new data sample is lost and will never be received.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of lost samples under the :ref:`dds_layer_topic_topic`
  of the current :ref:`dds_layer_subscriber_dataReader`.

* **total_count_change**: The change in **total_count** since
  the last time |on_sample_lost| was called or the status was read.
  It can only be positive or zero.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_sampleRejectedStatus:

SampleRejectedStatus
--------------------

This status changes every time an incoming data sample is rejected by the :ref:`dds_layer_subscriber_dataReader`.
The reason for the rejection is stored as a :ref:`dds_layer_core_status_sampleRejectedStatusKind`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_reason              | ``SampleRejectedStatusKind``            |
+--------------------------+-----------------------------------------+
| last_instance_handle     | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of rejected samples under the :ref:`dds_layer_topic_topic`
  of the current :ref:`dds_layer_subscriber_dataReader`.

* **total_count_change**: The change in **total_count** since
  the last time |on_sample_rejected| was called or the status was read.
  It can only be positive or zero.

* **last_reason**: The reason for rejecting the last rejected sample.
  If no sample was ever rejected, it will have value ``NOT_REJECTED``.
  See :ref:`dds_layer_core_status_sampleRejectedStatusKind` for further details.

* **last_instance_handle**: Handle to the last instance whose sample was rejected.
  If no sample was ever rejected, it will have value ``c_InstanceHandle_Unknown``.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_sampleRejectedStatusKind:

SampleRejectedStatusKind
^^^^^^^^^^^^^^^^^^^^^^^^

There are four possible values:

* ``NOT_REJECTED``: It means there have been no rejections so far on this :ref:`dds_layer_subscriber_dataReader`.
  The rejection reason will have this value only while the total count of rejections is zero.
* ``REJECTED_BY_INSTANCES_LIMIT``: The sample was rejected because the
  :ref:`max_instances<resourcelimitsqospolicy>` limit was reached.
* ``REJECTED_BY_SAMPLES_LIMIT``: The sample was rejected because the
  :ref:`max_samples<resourcelimitsqospolicy>` limit was reached.
* ``REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT``: The sample was rejected because the
  :ref:`max_samples_per_instance<resourcelimitsqospolicy>` limit was reached.


.. _dds_layer_core_status_subscriptionMatchedStatus:

SubscriptionMatchedStatus
-------------------------

This status changes every time the :ref:`dds_layer_subscriber_dataReader` finds a :ref:`dds_layer_publisher_dataWriter`
that matches the :ref:`dds_layer_topic_topic` and has a common partition and a compatible QoS,
or has ceased to be matched with a :ref:`dds_layer_publisher_dataWriter` that was previously considered to be matched.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| current_count            | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| current_count_change     | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_publication_handle  | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of remote :ref:`DataWriters<dds_layer_publisher_dataWriter>`
  that have been discovered publishing on the same :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.

* **total_count_change**: The change in **total_count** since
  the last time |on_subscription_matched| was called or the status was read.
  It can only have zero or positive values.

* **current_count**: The number of remote :ref:`DataWriters<dds_layer_publisher_dataWriter>`
  currently matched to the :ref:`dds_layer_subscriber_dataReader`.

* **current_count_change**: The change in **current_count** since
  the last time |on_subscription_matched| was called or the status was read.
  It can have positive or negative values.

* **last_publication_handle**: Handle to the last :ref:`dds_layer_publisher_dataWriter`
  that matched the :ref:`dds_layer_subscriber_dataReader`.
  If no matching ever happened, it will have value ``c_InstanceHandle_Unknown``.


.. _dds_layer_core_status_livelinessLostStatus:

LivelinessLostStatus
--------------------

This status changes every time the :ref:`dds_layer_publisher_dataWriter` failed to assert its liveliness
during the period configured on its :ref:`dds_layer_publisher_dataWriterQos`.
This means that matched :ref:`dds_layer_subscriber_dataReader` entities will consider the
:ref:`dds_layer_publisher_dataWriter` as no longer *alive*.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of times that the :ref:`dds_layer_publisher_dataWriter`
  failed to assert its liveliness during the period configured on its :ref:`dds_layer_publisher_dataWriterQos`,
  becoming considered not *alive*.
  This count does not change when the :ref:`dds_layer_publisher_dataWriter` is already considered not *alive* and
  simply remains not *alive* for another liveliness period.

* **total_count_change**: The change in **total_count** since
  the last time |on_liveliness_lost| was called or the status was read.
  It can only have zero or positive values.


.. _dds_layer_core_status_offeredDeadlineMissedStatus:

OfferedDeadlineMissedStatus
---------------------------

This status changes every time the :ref:`dds_layer_publisher_dataWriter` fails to provide
data within the deadline period configured on its :ref:`dds_layer_publisher_dataWriterQos`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_instance_handle     | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of missed deadlines for any instance written by the
  current :ref:`dds_layer_publisher_dataWriter`.
  As the deadline period applies to each instance of the :ref:`dds_layer_topic_topic` independently,
  the count will will be incremented by one for each instance for which data
  was not sent in the deadline period.

* **total_count_change**: The change in **total_count** since
  the last time |on_offered_deadline_missed| was called or the status was read.
  It can only have zero or positive values.

* **last_instance_handle**: Handle to the last instance that missed the deadline.
  If no deadline was ever missed, it will have value ``c_InstanceHandle_Unknown``.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_offeredIncompatibleQosStatus:

OfferedIncompatibleQosStatus
----------------------------

This status changes every time the :ref:`dds_layer_publisher_dataWriter` finds a
:ref:`dds_layer_subscriber_dataReader` that matches the :ref:`dds_layer_topic_topic` and has
a common partition, but with a QoS configuration that is incompatible with the one defined on the
:ref:`dds_layer_publisher_dataWriter`.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_policy_id           | ``uint32_t``                            |
+--------------------------+-----------------------------------------+
| policies                 | ``std::vector<QosPolicyCount>``         |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of :ref:`DataReaders<dds_layer_subscriber_dataReader>` found
  matching the :ref:`dds_layer_topic_topic` and with a common partition, but with a QoS configuration
  that is incompatible with the one defined on the :ref:`dds_layer_publisher_dataWriter`.

* **total_count_change**: The change in **total_count** since
  the last time |on_offered_incompatible_qos| was called or the status was read.
  It can only have zero or positive values.

* **last_policy_id**: The policy ID of one of the policies that was found to be incompatible with the
  current :ref:`dds_layer_publisher_dataWriter`.
  If more than one policy happens to be incompatible, only one of them will be reported in this member.

* **policies**: A list that holds, for each policy, the total number of times that the policy was
  found to be incompatible with the one requested by a remote :ref:`dds_layer_subscriber_dataReader` that
  matched the :ref:`dds_layer_topic_topic` and with a common partition.
  See :ref:`dds_layer_core_status_qosPolicyCount` for more information the information that is stored for each policy.

.. warning::

    Currently this status is not supported and will be implemented in future releases.
    As a result, trying to access this status will return ``NOT_SUPPORTED``
    and the corresponding listener will never be called.


.. _dds_layer_core_status_publicationMatchedStatus:

PublicationMatchedStatus
------------------------

This status changes every time the :ref:`dds_layer_publisher_dataWriter` finds a :ref:`dds_layer_subscriber_dataReader`
that matches the :ref:`dds_layer_topic_topic` and has a common partition and a compatible QoS,
or has ceased to be matched with a :ref:`dds_layer_subscriber_dataReader` that was previously considered to be matched.

List of status data members:

+--------------------------+-----------------------------------------+
| Data Member Name         | Type                                    |
+==========================+=========================================+
| total_count              | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| total_count_change       | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| current_count            | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| current_count_change     | ``int32_t``                             |
+--------------------------+-----------------------------------------+
| last_subscription_handle | ``InstanceHandle_t``                    |
+--------------------------+-----------------------------------------+

* **total_count**: Total cumulative count of remote :ref:`DataReaders<dds_layer_subscriber_dataReader>`
  that have been discovered publishing on the same :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS.
  They may not all be matched at the moment.

* **total_count_change**: The change in **total_count** since
  the last time |on_publication_matched| was called or the status was read.
  It can only have zero or positive values.

* **current_count**: The number of remote :ref:`DataReaders<dds_layer_subscriber_dataReader>`
  currently matched to the :ref:`dds_layer_publisher_dataWriter`.

* **current_count_change**: The change in **current_count** since
  the last time |on_publication_matched| was called or the status was read.
  It can have positive or negative values.

* **last_subscription_handle**: Handle to the last :ref:`dds_layer_subscriber_dataReader`
  that matched the :ref:`dds_layer_publisher_dataWriter`.
  If no matching ever happened, it will have value ``c_InstanceHandle_Unknown``.



