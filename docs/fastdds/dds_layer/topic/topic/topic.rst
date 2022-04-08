.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_topic:

Topic
=====

A |Topic-api| is a specialization of the broader concept of :ref:`dds_layer_topic_topicDescription`.
A Topic represents a single data flow between :ref:`dds_layer_publisher_publisher`
and :ref:`dds_layer_subscriber_subscriber`, providing:

 * The name to identify the data flow.
 * The data type that is transmitted on that flow.
 * The QoS values related to the data itself.

The behavior of the Topic can be modified with the QoS values
specified on :ref:`dds_layer_topic_topicQos`.
The QoS values can be set at the creation of the Topic,
or modified later with the |Topic::set_qos-api| member function.

Like other Entities, Topic accepts a Listener that will be notified of
status changes on the Topic.

Please refer to :ref:`dds_layer_topic_creation` for more information about how to create a |Topic-api|.

.. _dds_layer_topic_topicQos:

TopicQos
--------

|TopicQos-api| controls the behavior of the Topic.
Internally it contains the following |QosPolicy-api| objects:

+----------------------------------------------+------------------------------------------------------------+----------+
| QosPolicy class                              | Accessor                                                   | Mutable  |
+==============================================+============================================================+==========+
| |TopicDataQosPolicy|                         | |TopicQos::topic_data-api|                                 | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |DurabilityQosPolicy|                        | |TopicQos::durability-api|                                 | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |DurabilityServiceQosPolicy|                 | |TopicQos::durability_service-api|                         | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |DeadlineQosPolicy|                          | |TopicQos::deadline-api|                                   | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |LatencyBudgetQosPolicy|                     | |TopicQos::latency_budget-api|                             | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |LivelinessQosPolicy|                        | |TopicQos::liveliness-api|                                 | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |ReliabilityQosPolicy|                       | |TopicQos::reliability-api|                                | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |DestinationOrderQosPolicy|                  | |TopicQos::destination_order-api|                          | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |HistoryQosPolicy|                           | |TopicQos::history-api|                                    | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |ResourceLimitsQosPolicy|                    | |TopicQos::resource_limits-api|                            | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |TransportPriorityQosPolicy|                 | |TopicQos::transport_priority-api|                         | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |LifespanQosPolicy|                          | |TopicQos::lifespan-api|                                   | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |OwnershipQosPolicy|                         | |TopicQos::ownership-api|                                  | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+
| |DataRepresentationQosPolicy|                | |TopicQos::representation-api|                             | Yes      |
+----------------------------------------------+------------------------------------------------------------+----------+

Refer to the detailed description of each QosPolicy-api class for more information about their usage and
default values.

The QoS value of a previously created Topic can be modified using the
|Topic::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultTopicQos:

Default TopicQos
^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_topic_topicQos` refers to the value returned by the
|DomainParticipant::get_default_topic_qos-api| member function on the :ref:`dds_layer_domainParticipant` instance.
The special value ``TOPIC_QOS_DEFAULT`` can be used as QoS argument on |DomainParticipant::create_topic-api|
or |Topic::set_qos-api| member functions to indicate that the current default TopicQos
should be used.

When the system starts, the default TopicQos is equivalent to the default constructed
value |TopicQos::TopicQos-api|.
The default TopicQos can be modified at any time using the
|DomainParticipant::get_default_topic_qos-api| member function on the DomainParticipant instance.
Modifying the default TopicQos will not affect already existing Topic
instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_TOPICQOS
   :end-before: //!
   :dedent: 8

|DomainParticipant::get_default_topic_qos-api| member function also accepts the value ``TOPIC_QOS_DEFAULT``
as input argument.
This will reset the current default TopicQos to default constructed
value |TopicQos::TopicQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``TOPIC_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On |DomainParticipant::create_topic-api| and |Topic::set_qos-api| it refers to the default TopicQos
     as returned by |DomainParticipant::get_default_topic_qos-api|.
   * On |DomainParticipant::get_default_topic_qos-api| it refers to the default constructed |TopicQos::TopicQos-api|.


