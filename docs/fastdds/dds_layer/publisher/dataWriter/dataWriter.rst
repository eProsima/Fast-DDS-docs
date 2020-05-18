.. _dds_layer_publisher_dataWriter:

DataWriter
==========

A :class:`DataWriter` is attached to exactly one :ref:`dds_layer_publisher_publisher` that acts as a factory for it.
Additionally, each :class:`DataWriter` is bound to a single :ref:`dds_layer_topic_topic` since its creation.
This :ref:`dds_layer_topic_topic` must exist prior to the creation of the :class:`DataWriter`,
and must be bound to the data type that the :class:`DataWriter` wants to publish.

The effect of creating a new :class:`DataWriter` in a :ref:`dds_layer_publisher_publisher` for a specific
:ref:`dds_layer_topic_topic` is to initiate a new publication with the name and data type described
by the :ref:`dds_layer_topic_topic`.

Once the :class:`DataWriter` is created, the application can inform of changes in the data value using the
:func:`write` member function on the :class:`DataWriter`.
These changes will be transmitted to all subscriptions matched with this publication.

.. _dds_layer_publisher_dataWriterQos:

DataWriterQos
-------------

:class:`DataWriterQos` controls the behavior of the :ref:`dds_layer_publisher_dataWriter`.
Internally it contains the following :class:`QosPolicy` objects:

+--------------------------------+------------------------------------+----------+
| QosPolicy class                | Accessor/Mutator                   | Mutable  |
+================================+====================================+==========+
| DurabilityQosPolicy            | :func:`durability`                 | No       |
+--------------------------------+------------------------------------+----------+
| DurabilityServiceQosPolicy     | :func:`durability_service`         | Yes      |
+--------------------------------+------------------------------------+----------+
| DeadlineQosPolicy              | :func:`deadline`                   | Yes      |
+--------------------------------+------------------------------------+----------+
| LatencyBudgetQosPolicy         | :func:`latency_budget`             | Yes      |
+--------------------------------+------------------------------------+----------+
| LivelinessQosPolicy            | :func:`liveliness`                 | No       |
+--------------------------------+------------------------------------+----------+
| ReliabilityQosPolicy           | :func:`reliability`                | No (*)   |
+--------------------------------+------------------------------------+----------+
| DestinationOrderQosPolicy      | :func:`destination_order`          | No       |
+--------------------------------+------------------------------------+----------+
| HistoryQosPolicy               | :func:`history`                    | Yes      |
+--------------------------------+------------------------------------+----------+
| ResourceLimitsQosPolicy        | :func:`resource_limits`            | Yes      |
+--------------------------------+------------------------------------+----------+
| TransportPriorityQosPolicy     | :func:`transport_priority`         | Yes      |
+--------------------------------+------------------------------------+----------+
| LifespanQosPolicy              | :func:`lifespan`                   | Yes      |
+--------------------------------+------------------------------------+----------+
| UserDataQosPolicy              | :func:`user_data`                  | Yes      |
+--------------------------------+------------------------------------+----------+
| OwnershipQosPolicy             | :func:`ownership`                  | No       |
+--------------------------------+------------------------------------+----------+
| OwnershipStrengthQosPolicy     | :func:`ownership_strength`         | Yes      |
+--------------------------------+------------------------------------+----------+
| WriterDataLifecycleQosPolicy   | :func:`writer_data_lifecycle`      | Yes      |
+--------------------------------+------------------------------------+----------+
| PublishModeQosPolicy           | :func:`publish_mode`               | Yes      |
+--------------------------------+------------------------------------+----------+
| DataRepresentationQosPolicy    | :func:`representation`             | Yes      |
+--------------------------------+------------------------------------+----------+
| PropertyPolicyQos              | :func:`properties`                 | Yes      |
+--------------------------------+------------------------------------+----------+
| RTPSReliableWriterQos          | :func:`reliable_writer_qos`        | Yes      |
+--------------------------------+------------------------------------+----------+
| RTPSEndpointQos                | :func:`endpoint`                   | Yes      |
+--------------------------------+------------------------------------+----------+
| WriterResourceLimitsQos        | :func:`writer_resource_limits`     | Yes      |
+--------------------------------+------------------------------------+----------+
| ThroughputControllerDescriptor | :func:`throughput_controller`      | Yes      |
+--------------------------------+------------------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
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
   :dedent: 8


.. _dds_layer_defaultDataWriterQos:

Default DataWriterQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_publisher_dataWriterQos` refers to the value returned by the
:func:`get_default_dataWriter_qos` member function on the :ref:`dds_layer_publisher_publisher` instance.
The special value ``DATAWRITER_QOS_DEFAULT`` can be used as QoS argument on :func:`create_datawriter`
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
   :dedent: 8

:func:`set_default_dataWriter_qos` member function also accepts the special value ``DATAWRITER_QOS_DEFAULT``
as input argument.
This will reset the current default :ref:`dds_layer_publisher_dataWriterQos` to default constructed
value :func:`DataWriterQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAWRITERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``DATAWRITER_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On :func:`create_datawriter` and :func:`set_qos` it refers to the default :ref:`dds_layer_publisher_dataWriterQos`
     as returned by :func:`get_default_dataWriter_qos`.
   * On :func:`set_default_dataWriter_qos` it refers to the default constructed :func:`DataWriterQos`.



