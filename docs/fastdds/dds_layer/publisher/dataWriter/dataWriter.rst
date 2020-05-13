.. _dds_layer_publisher_dataWriter:

DataWriter
==========

A DataWriter is attached to exactly one :ref:`dds_layer_publisher_publisher` that acts as a factory for it.
Additionally, each DataWriter is bound to a single :ref:`dds_layer_topic_topic` since its creation.
This :ref:`dds_layer_topic_topic` must exist prior to the DataWriter's creation, and must be bound
to the data type that the DataWriter wants to publish.

The effect of creating a new DataWriter in a :ref:`dds_layer_publisher_publisher` for a specific
:ref:`dds_layer_topic_topic` is to initiate a new publication with the name and data type described
by the :ref:`dds_layer_topic_topic`.

Once the DataWriter is created, the application can inform of changes in the data value using the
:func:`write` member function on the DataWriter.
These changes will be transmitted to all subscriptions matched with this publication.

.. _dds_layer_publisher_dataWriterQos:

DataWriterQos
-------------

DataWriterQos controls the behavior of the :ref:`dds_layer_publisher_dataWriter`.
Internally it contains the following QosPolicy objects:

+-----------------------------+--------------------------------+----------+
| Name                        | QosPolicy class                | Mutable  |
+=============================+================================+==========+
| ``durability_``             | DurabilityQosPolicy            | no       |
+-----------------------------+--------------------------------+----------+
| ``durability_service_``     | DurabilityServiceQosPolicy     | yes      |
+-----------------------------+--------------------------------+----------+
| ``deadline_``               | DeadlineQosPolicy              | yes      |
+-----------------------------+--------------------------------+----------+
| ``latency_budget_``         | LatencyBudgetQosPolicy         | yes      |
+-----------------------------+--------------------------------+----------+
| ``liveliness_``             | LivelinessQosPolicy            | no       |
+-----------------------------+--------------------------------+----------+
| ``reliability_``            | ReliabilityQosPolicy           | no (*)   |
+-----------------------------+--------------------------------+----------+
| ``destination_order_``      | DestinationOrderQosPolicy      | no       |
+-----------------------------+--------------------------------+----------+
| ``history_``                | HistoryQosPolicy               | yes      |
+-----------------------------+--------------------------------+----------+
| ``resource_limits_``        | ResourceLimitsQosPolicy        | yes      |
+-----------------------------+--------------------------------+----------+
| ``transport_priority_``     | TransportPriorityQosPolicy     | yes      |
+-----------------------------+--------------------------------+----------+
| ``lifespan_``               | LifespanQosPolicy              | yes      |
+-----------------------------+--------------------------------+----------+
| ``user_data_``              | UserDataQosPolicy              | yes      |
+-----------------------------+--------------------------------+----------+
| ``ownership_``              | OwnershipQosPolicy             | mo       |
+-----------------------------+--------------------------------+----------+
| ``ownership_strength_``     | OwnershipStrengthQosPolicy     | yes      |
+-----------------------------+--------------------------------+----------+
| ``writer_data_lifecycle_``  | WriterDataLifecycleQosPolicy   | yes      |
+-----------------------------+--------------------------------+----------+
| ``publish_mode_``           | PublishModeQosPolicy           | yes      |
+-----------------------------+--------------------------------+----------+
| ``representation_``         | DataRepresentationQosPolicy    | yes      |
+-----------------------------+--------------------------------+----------+
| ``properties_``             | PropertyPolicyQos              | yes      |
+-----------------------------+--------------------------------+----------+
| ``reliable_writer_qos_``    | RTPSReliableWriterQos          | yes      |
+-----------------------------+--------------------------------+----------+
| ``endpoint_``               | RTPSEndpointQos                | yes      |
+-----------------------------+--------------------------------+----------+
| ``writer_resource_limits_`` | WriterResourceLimitsQos        | yes      |
+-----------------------------+--------------------------------+----------+
| ``throughput_controller_``  | ThroughputControllerDescriptor | yes      |
+-----------------------------+--------------------------------+----------+

Refer to the detailed description of each QosPolicy class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the ``max_blocking_time`` data member of ReliabilityQosPolicy can be modified any time.

The QoS value of a previously created :ref:`dds_layer_publisher_dataWriter` can be modified using the
:func:`set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAWRITERQOS
   :end-before: //!


.. _dds_layer_defaultDataWriterQos:

Default DataWriterQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_publisher_dataWriterQos` refers to the value returned by the
:func:`get_default_dataWriter_qos` member function on the :ref:`dds_layer_publisher_publisher` instance.
The special value ``DATAWRITER_QOS_DEFAULT`` can be used as QoS parameter on :func:`create_datawriter`
or :func:`set_qos` member functions to indicate that the current default :ref:`dds_layer_publisher_dataWriterQos`
should be used.

When the system starts, the default :ref:`dds_layer_publisher_dataWriterQos` is equivalent to the default constructed
value :func:`DataWriterQos`.
The default :ref:`dds_layer_publisher_dataWriterQos` can be modified at any time using the
:func:`set_default_dataWriter_qos` member function on the :ref:`dds_layer_publisher_publisher` instance.
Modifying the default :ref:`dds_layer_publisher_dataWriterQos` will not affect already existing
:ref:`dds_layer_publisher_dataWriter` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAWRITERQOS
   :end-before: //!

:func:`set_default_dataWriter_qos` member function also accepts the special value ``DATAWRITER_QOS_DEFAULT``
as input parameter.
This will reset the current default :ref:`dds_layer_publisher_dataWriterQos` to default constructed
value :func:`DataWriterQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAWRITERQOS_TO_DEFAULT
   :end-before: //!


.. _dds_layer_publisher_datawriter_creation:

Creating a DataWriter
=====================

A :ref:`dds_layer_publisher_dataWriter` always belongs to a :ref:`dds_layer_publisher_publisher`.
Creation of a :ref:`dds_layer_publisher_dataWriter` is done with the :func:`create_datawriter` member function on the
:ref:`dds_layer_publisher_publisher` instance, that acts as a factory for the :ref:`dds_layer_publisher_dataWriter`.

Mandatory parameters are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * The :ref:`dds_layer_publisher_dataWriterQos` describing the behavior of the :ref:`dds_layer_publisher_dataWriter`.
   If the provided value is :class:`DATAWRITER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDataWriterQos` is used.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_publisher_dataWriterListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_dataWriter`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_dataWriterListener`.
   By default all events are enabled.

:func:`create_datawriter` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DATAWRITER
   :end-before: //!


.. _dds_layer_publisher_datawriter_creation_profile:

Profile based creation of a DataWriter
--------------------------------------

Instead of using a :ref:`dds_layer_publisher_dataWriterQos`, the name of a profile
can be used to create a :ref:`dds_layer_publisher_dataWriter` with the :func:`create_datawriter_with_profile`
member function on the :ref:`dds_layer_publisher_publisher` instance.

Mandatory parameters are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * A string with the name that identifies the :ref:`dds_layer_publisher_dataWriter`.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_publisher_dataWriterListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_dataWriter`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_dataWriterListener`.
   By default all events are enabled.

:func:`create_datawriter_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DATAWRITER
   :end-before: //!


.. _dds_layer_publisher_datawriter_deletion:

Deleting a DataWriter
---------------------

A :ref:`dds_layer_publisher_dataWriter` can be deleted with the :func:`delete_datawriter` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_publisher_dataWriter` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DATAWRITER
   :end-before: //!


.. _dds_layer_publisher_write:

Publishing data
===============

The user informs of a change in the value of a data instance with the :func:`write` member function on the
:ref:`dds_layer_publisher_dataWriter`. This change will then be communicated to every
:ref:`dds_layer_subscriber_subscriber` matched with the :ref:`dds_layer_publisher_dataWriter`.
As a side effect, this operation asserts liveliness on the :ref:`dds_layer_publisher_dataWriter` itself,
the :ref:`dds_layer_publisher_publisher` and the :ref:`dds_layer_domainParticipant`.

The function takes two arguments:

 * A pointer to the data instance with the new values.
 * The handler to the instance.

An empty (i.e., default constructed :func:`InstanceHandle_t`) instance handler can be used for the parameter handle.
This indicates that the identity of the instance should be automatically deduced from the key of the
instance data.
Alternatively, the member function :func:`write` is overloaded to take only the pointer to the data instance,
which will always deduced the identity from the key of the instance data.

If the handle is not empty, then it must correspond to the value obtained with the :func:`getKey` of the
:class:`TypeSupport` instance.
Otherwise the write function will fail with ``RETCODE_PRECONDITION_NOT_MET``.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_WRITE
   :end-before: //!


.. _dds_layer_publisher_write_blocking:

Blocking of the write operation
-------------------------------

If the reliability kind is set to ``RELIABLE`` on the :ref:`dds_layer_publisher_dataWriterQos`,
the :func:`write` operation may block.
Specifically, if the limits specified in the configured resource limits have been reached, the
:func:`write` operation will block waiting for space to become available.
Under these circumstances, the reliability ``max_blocking_time`` configures the maximum time
the write operation may block waiting.
If ``max_blocking_time`` elapses before the :ref:`dds_layer_publisher_dataWriter` is able to store
the modification without exceeding the limits, the write operation will fail and return ``TIMEOUT``.



