.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_dataWriter:

DataWriter
==========

A |DataWriter-api| is attached to exactly one :ref:`dds_layer_publisher_publisher` that acts as a factory for it.
Additionally, each DataWriter is bound to a single :ref:`dds_layer_topic_topic` since its creation.
This Topic must exist prior to the creation of the DataWriter,
and must be bound to the data type that the DataWriter wants to publish.

The effect of creating a new DataWriter in a Publisher for a specific
Topic is to initiate a new publication with the name and data type described
by the Topic.

Once the DataWriter is created, the application can inform of changes in the data value using the
|DataWriter::write-api| member function on the DataWriter.
These changes will be transmitted to all subscriptions matched with this publication.

.. _dds_layer_publisher_dataWriterQos:

DataWriterQos
-------------

|DataWriterQos-api| controls the behavior of the DataWriter.
Internally it contains the following |QosPolicy-api| objects:

+----------------------------------+-------------------------------------------------+----------+
| QosPolicy class                  | Accessor/Mutator                                | Mutable  |
+==================================+=================================================+==========+
| |DurabilityQosPolicy|            | |DataWriterQos::durability-api|                 | No       |
+----------------------------------+-------------------------------------------------+----------+
| |DurabilityServiceQosPolicy|     | |DataWriterQos::durability_service-api|         | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |DeadlineQosPolicy|              | |DataWriterQos::deadline-api|                   | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |LatencyBudgetQosPolicy|         | |DataWriterQos::latency_budget-api|             | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |LivelinessQosPolicy|            | |DataWriterQos::liveliness-api|                 | No       |
+----------------------------------+-------------------------------------------------+----------+
| |ReliabilityQosPolicy|           | |DataWriterQos::reliability-api|                | No (*)   |
+----------------------------------+-------------------------------------------------+----------+
| |DestinationOrderQosPolicy|      | |DataWriterQos::destination_order-api|          | No       |
+----------------------------------+-------------------------------------------------+----------+
| |HistoryQosPolicy|               | |DataWriterQos::history-api|                    | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |ResourceLimitsQosPolicy|        | |DataWriterQos::resource_limits-api|            | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |TransportPriorityQosPolicy|     | |DataWriterQos::transport_priority-api|         | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |LifespanQosPolicy|              | |DataWriterQos::lifespan-api|                   | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |UserDataQosPolicy|              | |DataWriterQos::user_data-api|                  | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |OwnershipQosPolicy|             | |DataWriterQos::ownership-api|                  | No       |
+----------------------------------+-------------------------------------------------+----------+
| |OwnershipStrengthQosPolicy|     | |DataWriterQos::ownership_strength-api|         | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |WriterDataLifecycleQosPolicy|   | |DataWriterQos::writer_data_lifecycle-api|      | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |PublishModeQosPolicy|           | |DataWriterQos::publish_mode-api|               | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |DataRepresentationQosPolicy|    | |DataWriterQos::representation-api|             | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |PropertyPolicyQos|              | |DataWriterQos::properties-api|                 | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |RTPSReliableWriterQos|          | |DataWriterQos::reliable_writer_qos-api|        | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |RTPSEndpointQos|                | |DataWriterQos::endpoint-api|                   | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |WriterResourceLimitsQos|        | |DataWriterQos::writer_resource_limits-api|     | Yes      |
+----------------------------------+-------------------------------------------------+----------+
| |DataSharingQosPolicy|           | |DataWriterQos::data_sharing-api|               | No       |
+----------------------------------+-------------------------------------------------+----------+

The following non-consolidated property-assigned QoS apply to DataWriters:

+----------------------------------+-------------------------------------------------+
| Property name                    | Non-consolidated QoS                            |
+==================================+=================================================+
| fastdds.push_mode                | :ref:`push_mode_qos_policy`                     |
+----------------------------------+-------------------------------------------------+
| partitions                       | :ref:`property_policies_partitions`             |
+----------------------------------+-------------------------------------------------+

Refer to the detailed description of each |QosPolicy-api| class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the ``max_blocking_time`` data member of |ReliabilityQosPolicy| can be modified any time.

The QoS value of a previously created DataWriter can be modified using the
|DataWriter::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAWRITERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDataWriterQos:

Default DataWriterQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_publisher_dataWriterQos` refers to the value returned by the
|Publisher::get_default_datawriter_qos-api| member function on the Publisher instance.
The special value :class:`DATAWRITER_QOS_DEFAULT` can be used as QoS argument on |Publisher::create_datawriter-api|
or |DataWriter::set_qos-api| member functions to indicate that the current default
DataWriterQos should be used.

When the system starts, the default DataWriterQos is equivalent to the default constructed
value |DataWriterQos::DataWriterQos-api|.
The default DataWriterQos can be modified at any time using the
|Publisher::set_default_datawriter_qos-api| member function on the Publisher instance.
Modifying the default DataWriterQos will not affect already existing
DataWriter instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAWRITERQOS
   :end-before: //!
   :dedent: 8

|Publisher::set_default_datawriter_qos-api| member function also accepts the special value ``DATAWRITER_QOS_DEFAULT``
as input argument.
This will reset the current default DataWriterQos to default constructed
value |DataWriterQos::DataWriterQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAWRITERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``DATAWRITER_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On |Publisher::create_datawriter-api| and |DataWriter::set_qos-api| it refers to the default DataWriterQos
     as returned by |Publisher::get_default_datawriter_qos-api|.
   * On |Publisher::set_default_datawriter_qos-api| it refers to the default constructed
     |DataWriterQos::DataWriterQos-api|.



