.. _dds_layer_topic_topic:

Topic
=====

A Topic is a specialization of the broarder concept of :class:`TopicDescription`.
A Topic represents a single data flow between Publishers and Subscribers, providing:
 * Thw name to identify the data flow.
 * The data type that is transmitted on that flow.
 * The QoS values related to the data itself.

The behavior of the :class:`Topic` can be modified with the QoS values
specified on :ref:`dds_layer_topicQos`.
The QoS values can be set at the creation of the :class:`Topic`,
or modified later with the :func:`set_qos()` method.

Like other Entities, :class:`Topic` accepts a Listener that will be notified of
status changes on the :class:`Topic`.


.. _dds_layer_topic_creation:

Creating a Topic
----------------------------

A :class:`Topic` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a :class:`Topic` is done with the :func:`create_topic()` method on the
:ref:`dds_layer_domainParticipant` instance, that acts as a factory for the :class:`Topic`.

Mandatory parameters are:

 * A string with the name that identifies the :class:`Topic`.
 
 * The :ref:`dds_layer_topic_typeSupport` describing the data type transmitted.
 
 * The :ref:`dds_layer_TopicQos` describing the behavior of the :class:`Topic`.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultTopicQos` is used.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_topicParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :class:`Topic`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the Listener.
   By default all events are enabled.

:func:`create_topic()` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_TOPIC
   :end-before: //!


.. _dds_layer_topic_creation_profile:

Profile based creation of a Topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instead of using a :ref:`dds_layer_topicQos`, the name of a profile
can be used to create a Topic with the :func:`create_topic_with_profile()`
method on the :ref:`dds_layer_domainParticipant` instance.

Mandatory parameters are:

 * A string with the name that identifies the :class:`Topic`.
 
 * The :ref:`dds_layer_topic_typeSupport` describing the data type transmitted.
 
 * The name of the profile to be applied to the :class:`Topic`.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_topicParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :class:`Topic`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the Listener.
   By default all events are enabled.

:func:`create_topic_with_profile()` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_TOPIC
   :end-before: //!


.. _dds_layer_topic_deletion:

Deleting a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A :class:`Topic` can be deleted with the :func:`delete_topic()` method on the
:ref:`dds_layer_domainParticipant` instance where the :class:`Topic` was created..

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_TOPIC
   :end-before: //!

.. _dds_layer_topicQos:

TopicQos
--------

TopicQos controls the behavior of the :ref:`dds_layer_topic_topic`.
Internally it contains the following QosPolicy objects:

+--------------------------+------------------------------+----------+
| Name                     | QosPolicy class              | Mutable  |
+===============================+=========================+==========+
| ``topic_data_``          | TopicDataQosPolicy           | yes      |
+--------------------------+------------------------------+----------+
| ``durability_``          | DurabilityQosPolicy          | yes      |
+--------------------------+------------------------------+----------+
| ``durability_service_``  | DurabilityServiceQosPolicy   | yes      |
+--------------------------+------------------------------+----------+
| ``deadline_``            | DeadlineQosPolicy            | yes      |
+--------------------------+------------------------------+----------+
| ``latency_budget_``      | LatencyBudgetQosPolicy       | yes      |
+--------------------------+------------------------------+----------+
| ``liveliness_``          | LivelinessQosPolicy          | yes      |
+--------------------------+------------------------------+----------+
| ``reliability_``         | ReliabilityQosPolicy         | yes      |
+--------------------------+------------------------------+----------+
| ``destination_order_``   | DestinationOrderQosPolicy    | yes      |
+--------------------------+------------------------------+----------+
| ``history_``             | HistoryQosPolicy             | yes      |
+--------------------------+------------------------------+----------+
| ``resource_limits_``     | ResourceLimitsQosPolicy      | yes      |
+--------------------------+------------------------------+----------+
| ``transport_priority_``  | TransportPriorityQosPolicy   | yes      |
+--------------------------+------------------------------+----------+
| ``lifespan_``            | LifespanQosPolicy            | yes      |
+--------------------------+------------------------------+----------+
| ``ownership_``           | OwnershipQosPolicy           | yes      |
+--------------------------+------------------------------+----------+
| ``representation_``      | DataRepresentationQosPolicy  | yes      |
+--------------------------+------------------------------+----------+

Refer to the detailed description of each QosPolicy class for more information about their usage and
default values.

The QoS value of a previously created :class:`Topic` can be modified using the :func:`set_qos()` method.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS
   :end-before: //!


.. _dds_layer_defaultTopicQos:

Default TopicQos
^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_topicQos` refers to the value returned by the
:func:`get_default_topic_qos()` method on the :ref:`dds_layer_domainParticipant` instance.
The special symbol :class:`TOPIC_QOS_DEFAULT` can be used as QoS parameter on :func:`create_topic()`
or :func:`set_qos()` methods to indicate that the current default :ref:`dds_layer_topicQos` should be used.

When the system starts, the default :ref:`dds_layer_topicQos` is equivalent to the default constructed
value :func:`TopicQos()`.
The default :ref:`dds_layer_topicQos`can be modified at any time using the
:func:`set_default_topict_qos()` method on the :ref:`dds_layer_domainParticipant` instance.
Modifying the default :ref:`dds_layer_topicQos` will not affect already exsiting Topic
instances unless their QoS is modified with :func:`set_qos(TOPIC_QOS_DEFAULT)`.
value :func:`TopicQos()`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_TOPICQOS
   :end-before: //!

:func:`set_default_topic_qos()` method also accepts the symbol :class:`TOPIC_QOS_DEFAULT`
as input parameter.
This will reset the current default :ref:`dds_layer_topicQos` to default constructed value


.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_TOPICQOS_TO_DEFAULT
   :end-before: //!

