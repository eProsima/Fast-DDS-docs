.. _dds_layer_topic_topic:

Topic
=====

A :class:`Topic` is a specialization of the broader concept of :ref:`dds_layer_topic_topicDescription`.
A :class:`Topic` represents a single data flow between :ref:`dds_layer_publisher_publisher`
and :ref:`dds_layer_subscriber_subscriber`, providing:

 * The name to identify the data flow.
 * The data type that is transmitted on that flow.
 * The QoS values related to the data itself.

The behavior of the :class:`Topic` can be modified with the QoS values
specified on :ref:`dds_layer_topic_topicQos`.
The QoS values can be set at the creation of the :class:`Topic`,
or modified later with the :func:`set_qos` member function.

Like other Entities, :class:`Topic` accepts a Listener that will be notified of
status changes on the Topic.


.. _dds_layer_topic_topicQos:

TopicQos
--------

:class:`TopicQos` controls the behavior of the :ref:`dds_layer_topic_topic`.
Internally it contains the following :class:`QosPolicy` objects:

+------------------------------+----------------------------+----------+
| QosPolicy class              | Accessor                   | Mutable  |
+==============================+============================+==========+
| TopicDataQosPolicy           | :func:`topic_data`         | Yes      |
+------------------------------+----------------------------+----------+
| DurabilityQosPolicy          | :func:`durability`         | Yes      |
+------------------------------+----------------------------+----------+
| DurabilityServiceQosPolicy   | :func:`durability_service` | Yes      |
+------------------------------+----------------------------+----------+
| DeadlineQosPolicy            | :func:`deadline`           | Yes      |
+------------------------------+----------------------------+----------+
| LatencyBudgetQosPolicy       | :func:`latency_budget`     | Yes      |
+------------------------------+----------------------------+----------+
| LivelinessQosPolicy          | :func:`liveliness`         | Yes      |
+------------------------------+----------------------------+----------+
| ReliabilityQosPolicy         | :func:`reliability`        | Yes      |
+------------------------------+----------------------------+----------+
| DestinationOrderQosPolicy    | :func:`destination_order`  | Yes      |
+------------------------------+----------------------------+----------+
| HistoryQosPolicy             | :func:`history`            | Yes      |
+------------------------------+----------------------------+----------+
| ResourceLimitsQosPolicy      | :func:`resource_limits`    | Yes      |
+------------------------------+----------------------------+----------+
| TransportPriorityQosPolicy   | :func:`transport_priority` | Yes      |
+------------------------------+----------------------------+----------+
| LifespanQosPolicy            | :func:`lifespan`           | Yes      |
+------------------------------+----------------------------+----------+
| OwnershipQosPolicy           | :func:`ownership`          | Yes      |
+------------------------------+----------------------------+----------+
| DataRepresentationQosPolicy  | :func:`representation`     | Yes      |
+------------------------------+----------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

The QoS value of a previously created :ref:`dds_layer_topic_topic` can be modified using the
:func:`set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultTopicQos:

Default TopicQos
^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_topic_topicQos` refers to the value returned by the
:func:`get_default_topic_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
The special value ``TOPIC_QOS_DEFAULT`` can be used as QoS argument on :func:`create_topic`
or :func:`set_qos` member functions to indicate that the current default :ref:`dds_layer_topic_topicQos`
should be used.

When the system starts, the default :ref:`dds_layer_topic_topicQos` is equivalent to the default constructed
value :func:`TopicQos`.
The default :ref:`dds_layer_topic_topicQos` can be modified at any time using the
:func:`set_default_topict_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
Modifying the default :ref:`dds_layer_topic_topicQos` will not affect already existing :ref:`dds_layer_topic_topic`
instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_TOPICQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_topic_qos` member function also accepts the value ``TOPIC_QOS_DEFAULT``
as input argument.
This will reset the current default :ref:`dds_layer_topic_topicQos` to default constructed
value :func:`TopicQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``TOPIC_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On :func:`create_topic` and :func:`set_qos` it refers to the default :ref:`dds_layer_topic_topicQos`
     as returned by :func:`get_default_topic_qos`.
   * On :func:`set_default_topic_qos` it refers to the default constructed :func:`TopicQos`.


