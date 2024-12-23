.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_dataReader:

DataReader
==========


A |DataReader-api| is attached to exactly one
:ref:`dds_layer_subscriber_subscriber` that acts as a factory for it.
Additionally, each DataReader is bound to a single
:ref:`dds_layer_topic_topic` since its creation.
This Topic must exist prior to the creation of the
DataReader,
and must be bound to the data type that the DataReader wants to publish.

The effect of creating a new DataReader in a
Subscriber for a specific
Topic is to initiate a new subscription with the name and data type described
by the Topic.

Once the DataReader is created, the application will be informed
when changes in the data value are received from remote publications.
These changes can then be retrieved using the |DataReader::read_next_sample-api| or |DataReader::take_next_sample-api|
member functions of the DataReader.

.. _dds_layer_subscriber_dataReaderQos:

DataReaderQos
-------------

|DatareaderQos-api| controls the behavior of the :ref:`dds_layer_subscriber_dataReader`.
Internally it contains the following |QosPolicy-api| objects:

+-----------------------------------------------+---------------------------------------------+----------+
| QosPolicy class                               | Accessor/Mutator                            | Mutable  |
+===============================================+=============================================+==========+
| |durabilityqospolicy|                         | |DataReaderQos::durability-api|             | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |durabilityserviceqospolicy|                  | |DataReaderQos::durability_service-api|     | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |deadlineqospolicy|                           | |DataReaderQos::deadline-api|               | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |latencybudgetqospolicy|                      | |DataReaderQos::latency_budget-api|         | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |livelinessqospolicy|                         | |DataReaderQos::liveliness-api|             | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |reliabilityqospolicy|                        | |DataReaderQos::reliability-api|            | No (*)   |
+-----------------------------------------------+---------------------------------------------+----------+
| |destinationorderqospolicy|                   | |DataReaderQos::destination_order-api|      | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |historyqospolicy|                            | |DataReaderQos::history-api|                | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |resourcelimitsqospolicy|                     | |DataReaderQos::resource_limits-api|        | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |lifespanqospolicy|                           | |DataReaderQos::lifespan-api|               | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |userdataqospolicy|                           | |DataReaderQos::user_data-api|              | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |ownershipqospolicy|                          | |DataReaderQos::ownership-api|              | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| |propertypolicyqos|                           | |DataReaderQos::properties-api|             | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |RTPSendpointqos|                             | |DataReaderQos::endpoint-api|               | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |readerresourcelimitsqos|                     | |DataReaderQos::reader_resource_limits-api| | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |timebasedfilterqospolicy|                    | |DataReaderQos::time_based_filter-api|      | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |readerdatalifecycleqospolicy|                | |DataReaderQos::reader_data_lifecycle-api|  | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |RTPSreliablereaderqos|                       | |DataReaderQos::reliable_reader_qos-api|    | Yes (*)  |
+-----------------------------------------------+---------------------------------------------+----------+
| |typeconsistencyqos|                          | |DataReaderQos::type_consistency-api|       | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+
| |datasharingqospolicy|                        | |DataReaderQos::data_sharing-api|           | No       |
+-----------------------------------------------+---------------------------------------------+----------+
| ``boolean``                                   | |DataReaderQos::expects_inline_qos-api|     | Yes      |
+-----------------------------------------------+---------------------------------------------+----------+

The following non-consolidated property-assigned QoS apply to DataReaders:

+----------------------------------+-------------------------------------------------+
| Property name                    | Non-consolidated QoS                            |
+==================================+=================================================+
| partitions                       | :ref:`property_policies_partitions`             |
+----------------------------------+-------------------------------------------------+

Refer to the detailed description of each |QosPolicy-api| class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the |ReliabilityQosPolicy::max_blocking_time-api| data member of |ReliabilityQosPolicy-api| can be modified
   any time.
.. note::

   Not all data members of RTPSReliableReaderQos are mutable, please refer to |RTPSReliableReaderQos|
   for more information.

The QoS value of a previously created DataReader can be modified using the
|DataReader::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDataReaderQos:

Default DataReaderQos
^^^^^^^^^^^^^^^^^^^^^

The default DataReaderQos refers to the value returned by the
|Subscriber::get_default_datareader_qos-api| member function on the
:ref:`dds_layer_subscriber_subscriber` instance.
The special value :code:`DATAREADER_QOS_DEFAULT` can be used as QoS argument on
|Subscriber::create_datareader-api| or
|DataReader::set_qos-api| member functions to indicate that the current default
DataReaderQos should be used.

When the system starts, the default DataReaderQos is equivalent to
the default constructed value |DataReaderQos::DataReaderQos-api|.
The default DataReaderQos can be modified at any time using the
|Subscriber::set_default_datareader_qos-api| member function on the
Subscriber instance.
Modifying the default DataReaderQos will not affect already existing
:ref:`dds_layer_subscriber_dataReader` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAREADERQOS
   :end-before: //!
   :dedent: 8

|Subscriber::set_default_datareader_qos-api| member function also accepts
the special value :code:`DATAREADER_QOS_DEFAULT` as input argument.
This will reset the current default DataReaderQos to default constructed
value |DataReaderQos::DataReaderQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value :code:`DATAREADER_QOS_DEFAULT` has different meaning depending on where it is used:

   * On |Subscriber::create_datareader-api|
     and |DataReader::set_qos-api| it refers to the default
     DataReaderQos as returned by
     |Subscriber::get_default_datareader_qos-api|.
   * On |Subscriber::set_default_datareader_qos-api| it refers
     to the default constructed |DataReaderQos::DataReaderQos-api|.

