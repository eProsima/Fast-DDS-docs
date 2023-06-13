.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service_topics:

Monitor Service Topics
======================

The following table depicts the properties of the topics within the :ref:`monitor_service`:

+---------------------------------------+-------------------------+---------------------------------------+
|**Topic name**                         |**Topic Alias**          | **TopicDataType**                     |
+---------------------------------------+-------------------------+---------------------------------------+
| ``fastrtps_monitor_service_status``   | MONITOR_SERVICE_TOPIC   | :ref:`monitor_service_status_data`    |
+---------------------------------------+-------------------------+---------------------------------------+

.. _monitor_service_status_topic:

Monitor Service Status Topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``Monitor Service Status Topic`` carries information about the
:ref:`monitoring information <monitor_service_keywords>` of the local entities of a
particular |DomainParticipant|.
The :ref:`monitoring information <monitor_service_keywords>` can be divided into different statuses
identified by a ``StatusKind``.
The possible values are described in the following table:

.. _monitoring_statuses:

+---------+-----------------------------+-----------------------------------------+
|**Value**|          **Name**           | **Description**                         |
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
|  4      |   ``LivelinessLostInfo``    | Tracks the status of the number of times|
|         |                             | a writer lost liveliness.               |
+---------+-----------------------------+-----------------------------------------+
|  5      |   ``LivelinessChangedInfo`` | Tracks the status of the number of times|
|         |                             | the liveliness changed in a reader.     |
+---------+-----------------------------+-----------------------------------------+
|  6      |  ``DeadlineMissedInfo``     | The Status of the number of deadlines   |
|         |                             | missed of a sample for that entity.     |
+---------+-----------------------------+-----------------------------------------+
|  7      |     ``SampleLostInfo``      | Tracks the status of the number of times|
|         |                             | this entity lost samples.               |
+---------+-----------------------------+-----------------------------------------+

.. note::

    If the service is enabled in a :ref:`RTPS layer <rtps_layer>` context, :ref:`not all
    the statuses will be published <monitor_service_in_rtps_note>`,
    only the ``ProxyInfo`` and ``ConnectionList``.

The ``Monitor Service Status Topic`` publishes new data when new updates are received from any
of the |DomainParticipant|'s local entities (on-event driven) with a minimum waiting time between
publications.
In addition, it is in charge of notifying about any disposal or liveliness lost.

.. _monitor_service_status_data:

Monitor Service Status Data
---------------------------

The ``MonitorServiceStatusData`` data structure comprises the following fields:

* *local_entity:* |Guid_t-api| of the local entity.
* *status_kind:* :ref:`StatusKind <monitoring_statuses>` enumeration identifying the status.
* *value:* The value of the status.

.. code-block:: bash

    MonitorServiceStatusData
      @Key GUID local_entity
      @Key StatusKind status_kind
      Data value

.. note::

    The *local_entity* and *status_kind* are keyed fields, hence making use of instances (see :ref:`dds_layer_topic_instances`).
    In this case, the pair <*local_entity, status_kind*> identifies a unique instance.


Each of the ``StatusKind`` enumeration values maps to a corresponding ``Data`` value.
The actual field names for the different values are described below:

* *entity_proxy:* Collection of the serialized Quality of Service Parameters in the form of a ``ParameterList``.

* *connection_list:* Defines how is this entity communicating with its matched entities.
  Each of the elements is of Connection type (depicted below).

.. code-block:: bash

    Connection
      uint32_t mode //INTRAPROCESS, DATASHARING, TRANSPORT
      LocatorList announced_locators
      LocatorList used_locators

* *incompatible_qos_status:* Status of the Incompatible QoS of that entity.

  * |DataWriter| Incompatible QoS Offered.
  * |DataReader| Incompatible QoS Requested.

* *inconsistent_topic_status:* Status of Inconsistent topics of the topic of that entity.
  Asked to the topic of the requested entity.

* *liveliness_lost_status:* Tracks the status of the number of times that liveliness was lost by a
  |DataWriter|.

* *liveliness_changed_status:* Tracks the status of the number of times that liveliness was lost
  by a |DataReader|.

* *deadline_status:* The Status of the number of deadlines missed that were registered in that entity.

* *sample_lost_status:* The number of samples that entity lost.

The following table depicts the relation between each of the ``StatusKind`` values and the ``Data`` field:

+---------------------+---------------------------+---------------------------+---------------------------+
|**StatusKind Value** |     **StatusKind Name**   | **Data field Name**       | **IDL Data field Type**   |
+---------------------+---------------------------+---------------------------+---------------------------+
|  0                  |     ``ProxyInfo``         | entity_proxy              | sequence<octet>           |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  1                  |     ``ConnectionList``    | connection_list           | sequence<Connection>      |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  2                  |  ``IncompatibleQoSInfo``  | incompatible_qos_status   | IncompatibleQoSStatus     |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  3                  | ``InconsistentTopicInfo`` | inconsistent_topic_status | InconsistentTopicStatus   |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  4                  |   ``LivelinessLostInfo``  | liveliness_lost_status    | LivelinessLostStatus      |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  5                  |   ``LivelinessLostInfo``  | liveliness_changed_status | LivelinessChangedStatus   |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  6                  |  ``DeadlineMissedInfo``   | deadline_missed_status    | DeadlineMissedStatus      |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
|  7                  |     ``SampleLostInfo``    | sample_lost_status        | SampleLostStatus          |
|                     |                           |                           |                           |
+---------------------+---------------------------+---------------------------+---------------------------+
