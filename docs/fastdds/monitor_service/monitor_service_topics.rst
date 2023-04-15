.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service_topics:

Monitor Service Topics
======================

The following table depicts a summary of the availble topics in the :ref:`monitor_service`:

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

The Monitor Service Event's Topic carries information about all the entities known (|Guid_t-api|) by a particular
|DomainParticipant|.
The ``Monitor Service`` Event's Topic publishes new data when new updates are received from any
of the |DomainParticipant|'s known entities (on-event driven).
In addition, it is in charge of notifying about any dispose or liveliness lost.

.. _monitor_service_events_data:

Monitor Service Event Data
--------------------------

The ``MonitorServiceEventData`` data structure comprises the following fields:

* *source_participant:* The GUID of the participant that knows the entity.
* *entity_guid:* Guid of a known entity.
* *seq_num:* Most recent sequence number for that entity.
  Sequence number might be invalid {-1,0} when the  Monitor Service
  is replying to a request and the requested entity is not known by the Monitor Service.

.. code-block:: bash

    GUID_t source_participant @key
    GUID_t entity_guid @key
    SequenceNumber_t seq_num

.. note::

    The *source_participant* and *entity_guid* are keyed fields, hence making use of instances (see :ref:`dds_layer_topic_instances`).
    In this case, the pair <*source_participant, entity_guid*> conforms a unique instance.

Monitor Service RPC Topics
^^^^^^^^^^^^^^^^^^^^^^^^^^

A |DomainParticipant| with an enabled ``Monitor Service`` provides a RPC mechanism for querying another |DomainParticipant| about its known entities.
This is achieved by means of the ``Monitor Service`` Request/Response topics, in which the client side writes the request and the ``Monitor Service``
of the requested |DomainParticipant| replies with the corresponding response.

The actual data types and fields of the aforementioned topics are described below:

.. _monitor_service_request_data:

Monitor Service Request Data
----------------------------

The ``MonitorServiceRequestData`` data structure has the following fields:

* *requested_participant:* It refers to the |Guid_t-api| of the participant of interest.
* *entity_guid:* The particular entity_id to get the information from.

.. code-block:: bash

    GUID_t requested_participant
    GUID_t entity_guid

.. _monitor_service_response_data:

Monitor Service Response Data
-----------------------------

The ``MonitorServiceResponseData`` data structure consists on the following fields:


* *event_data:* The current event data of the queried entity, this should be congruent
  with last MonitorServiceEventData published in the :ref:`monitor_service_events_topic`.
  In this case, the second |Guid_t-api| can be a known or an unknown entity for the queried participant, reflected
  with an invalid {-1,0} SequenceNumber_t in case of being unknown.
* *param_list:* ParameterList consisting on a set of parameters for describing a WriterProxy, ReaderProxy or ParticipantProxy.
  See `DDS Interoperability Wire Protocol <https://www.omg.org/spec/DDSI-RTPS/>`_ for further information on these structures.
* *locator_list:* Current |LocatorList_t-api| in use with that entity.
  It is an additional informative field for the DDS Monitor to know which locators a particular entity is using for communicating with other
  entity from other participant.

.. code-block:: bash

    MonitorServiceEventData event_data
    ParameterList param_list
    LocatorList locator_list



