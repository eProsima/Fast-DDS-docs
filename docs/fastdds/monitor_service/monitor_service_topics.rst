.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service_topics:

Monitor Service Topics
======================

The following table depicts a summary of the available topics in the :ref:`monitor_service`:

+---------------------------------------+-------------------------+---------------------------------------+
|**Topic name**                         |**Topic Alias**          | **TopicDataType**                     |
+---------------------------------------+-------------------------+---------------------------------------+
| ``_fastdds_monitor_service_events``   |     MONITOR_EV_TOPIC    | :ref:`monitor_service_events_data`    |
+---------------------------------------+-------------------------+---------------------------------------+
|``_fastdds_monitor_service_request``   |     MONITOR_REQ_TOPIC   | :ref:`monitor_service_request_data`   |
+---------------------------------------+-------------------------+---------------------------------------+
| ``_fastdds_monitor_service_response`` |     MONITOR_RES_TOPIC   | :ref:`monitor_service_response_data`  |
+---------------------------------------+-------------------------+---------------------------------------+

.. _monitor_service_events_topic:

Monitor Service Events Topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Monitor Service Event's Topic carries information about all the local entities of a particular
|DomainParticipant|, storing a collection of |Guid_t-api| identifying each entity alongside with a
sequence of statuses' version (an incremental counter), each one identified by an index.
These statuses' indexes are described in the following table:

.. _monitoring_statuses:

+---------+-----------------------------+-----------------------------------------+
|**Index**|          **Name**           | **Description**                         |
+---------+-----------------------------+-----------------------------------------+
|  0      |     ``ProxyInfo``           | Collection of Parameters describing the |
|         |                             | ``Proxy Data`` of that entity           |
+---------+-----------------------------+-----------------------------------------+
|  1      |     ``ConnectionList``      | List of connections that this entity is |
|         |                             | using with its matched remote entities. |
+---------+-----------------------------+-----------------------------------------+
|  2      |  ``IncompatibleQoSInfo``    | Status of the Incompatible QoS          |
|         |                             | of that entity.                         |
+---------+-----------------------------+-----------------------------------------+
|  3      | ``InconsistentTopicInfo``   | Status of Inconsistent topics that the  |
|         |                             | topic of that entity has.               |
+---------+-----------------------------+-----------------------------------------+
|  4      |   ``DeadlineMissedInfo``    | The Status of the number of deadlines   |
|         |                             | missed for that entity                  |
+---------+-----------------------------+-----------------------------------------+
|  5      |   ``LivelinessLostInfo``    | Tracks the status of the number of times|
|         |                             | a writer lost liveliness                |
+---------+-----------------------------+-----------------------------------------+
|  6      |  ``LivelinessChangedInfo``  | Tracks the status of the number of times|
|         |                             | the liveliness changed in a reader.     |
+---------+-----------------------------+-----------------------------------------+
|  7      |     ``SampleLostInfo``      | Tracks the number of times this entity  |
|         |                             | lost samples.                           |
+---------+-----------------------------+-----------------------------------------+

The ``Monitor Service`` Event's Topic publishes new data when new updates are received from any
of the |DomainParticipant|'s local entities (on-event driven) at a certain limited rate.
In addition, it is in charge of notifying about any dispose or liveliness lost.

.. _monitor_service_events_data:

Monitor Service Event Data
--------------------------

The ``MonitorServiceEventData`` data structure comprises the following fields:

* *source_participant:* The GUID of the participant that knows the entity.
* *entity_guid:* Guid of a known entity.
* *status_info:* Most recent :ref:`statuses<monitoring_statuses>` available for that entity.

.. code-block:: bash

    GUID_t source_participant @key
    GUID_t entity_guid @key
    InfoVersionStatus status_info

.. note::

    The *source_participant* and *entity_guid* are keyed fields, hence making use of instances (see :ref:`dds_layer_topic_instances`).
    In this case, the pair <*source_participant, entity_guid*> conforms a unique instance.

Monitor Service RPC Topics
^^^^^^^^^^^^^^^^^^^^^^^^^^

A |DomainParticipant| with an enabled ``Monitor Service`` provides a RPC mechanism
for querying another |DomainParticipant| about its local entities.
This is achieved by means of the ``Monitor Service`` Request/Response topics,
in which the client side writes the request specifying the :ref:`statuses<monitoring_statuses>` of
interest (in a mask) and the ``Monitor Service`` of the requested |DomainParticipant| collects
the requested data to later return the corresponding response.

The actual data types and fields of the aforementioned topics are described below:

.. _monitor_service_request_data:

Monitor Service Request Data
----------------------------

The ``MonitorServiceRequestData`` data structure has the following fields:

* *requested_participant:* It refers to the |Guid_t-api| of the participant of interest.
* *entity_guid:* The particular entity_id to get the information from.
* *request_mask:* The mask indicating what ``Monitoring Information`` should be
  returned in the response.

.. code-block:: bash

    GUID_t requested_participant @Key
    GUID_t entity_guid @Key
    RequestMask request_mask

.. _monitor_service_response_data:

Monitor Service Response Data
-----------------------------

The ``MonitorServiceResponseData`` data structure consists on the following fields:

* *event_data:* The current event data of the queried entity, this should be congruent
  with last MonitorServiceEventData published in the Monitor Service Event's Topic.
  This field is always returned in the response.
* *entity_proxy:* Collection of the Quality of Service Parameters in the form of a ParameterList.
  This field can be optionally returned[^5].
* *connection_list:* Defines how is this entity communicating with its matched entities.
  Each of the elements is of Connection type (depicted below).
  This field can be optionally returned.

.. code-block:: bash

    Connection
      uint32_t mode //INTRAPROCESS, DATASHARING, TRANSPORT
      LocatorList announced_locators
      LocatorList used_locators

* *incompatible_qos_status:* Status of the Incompatible QoS of that entity.
  |DataWriter| Offered |DataReader| Requested.
  This field can be optionally returned.
* *inconsistent_topic_status:* Status of Inconsistent topics of the topic of that entity.
  Asked to the topic of the requested entity.
  This field can be optionally returned.
* *liveliness_lost_status:* Tracks the status of the number of times that liveliness was lost by a
  |DataWriter|.
  This field can be optionally returned.
* *liveliness_changed_status:* Tracks the status of the number of times that liveliness was lost
  by a |DataReader|.
  This field can be optionally returned.
* *deadline_status:* The Status of the number of deadlines missed that were registered in that entity.
  This field can be optionally returned.
* *sample_lost_status:* The number of samples that entity lost.
  This field can be optionally returned.

.. code-block:: bash

    MonitorServiceEventData event_data
    vector<uint8_t> entity_proxy
    vector<vector<Connection>> connection_list
    vector<IncompatibleQoSStatus> incompatible_qos_status
    vector<InconsistentTopicStatus> inconsistent_topic_status
    vector<LivelinessLostStatus> liveliness_lost_status
    vector<LivelinessChangedStatus> liveliness_changed_status
    vector<DeadlineMissedStatus> deadline_status
    vector<SampleLostStatus> sample_lost_status

.. note::

    Note that each optional field can be returned, or not.
    This depends on the :ref:`RequestMask<monitor_service_request_data>` and if
    the service is announced within a :ref:`DDS<dds_layer>` or :ref:`RTPS<rtps_layer>` |domain|.
    A vector with a maximum size of 1 is used to denote the optionality.
