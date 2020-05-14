.. _dds_layer_subscriber_dataReader:

DataReader
==========


A :class:`DataReader` is attached to exactly one :ref:`dds_layer_subscriber_subscriber` that acts as a factory for it.
Additionally, each :class:`DataReader` is bound to a single :ref:`dds_layer_topic_topic` since its creation.
This :ref:`dds_layer_topic_topic` must exist prior to the creation of the :class:`DataReader`,
and must be bound to the data type that the :class:`DataReader` wants to publish.

The effect of creating a new :class:`DataReader` in a :ref:`dds_layer_subscriber_subscriber` for a specific
:ref:`dds_layer_topic_topic` is to initiate a new subscription with the name and data type described
by the :ref:`dds_layer_topic_topic`.

Once the :class:`DataReader` is created, the application will be informed when changes in the data value are received
from remote publications, and can then retrieve this changes using the :func:`read` or :func:`take` member functions
of the :class:`DataReader`.

.. _dds_layer_subscriber_dataReaderQos:

DataReaderQos
-------------

:class:`DataReaderQos` controls the behavior of the :ref:`dds_layer_subscriber_dataReader`.
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
| HistoryQosPolicy               | :func:`history`                    | No       |
+--------------------------------+------------------------------------+----------+
| ResourceLimitsQosPolicy        | :func:`resource_limits`            | No       |
+--------------------------------+------------------------------------+----------+
| LifespanQosPolicy              | :func:`lifespan`                   | Yes      |
+--------------------------------+------------------------------------+----------+
| UserDataQosPolicy              | :func:`user_data`                  | Yes      |
+--------------------------------+------------------------------------+----------+
| OwnershipQosPolicy             | :func:`ownership`                  | No       |
+--------------------------------+------------------------------------+----------+
| PropertyPolicyQos              | :func:`properties`                 | Yes      |
+--------------------------------+------------------------------------+----------+
| RTPSEndpointQos                | :func:`endpoint`                   | Yes      |
+--------------------------------+------------------------------------+----------+
| ReaderResourceLimitsQos        | :func:`reader_resource_limits`     | Yes      |
+--------------------------------+------------------------------------+----------+
| TimeBasedFilterQosPolicy       | :func:`time_based_filter`          | Yes      |
+--------------------------------+------------------------------------+----------+
| ReaderDataLifecycleQosPolicy   | :func:`reader_data_lifecycle`      | Yes      |
+--------------------------------+------------------------------------+----------+
| RTPSReliableReaderQos          | :func:`reliable_reader_qos`        | Yes      |
+--------------------------------+------------------------------------+----------+
| TypeConsistencyQos             | :func:`type_consistency`           | Yes      |
+--------------------------------+------------------------------------+----------+
| boolean                        | :func:`expects_inline_qos`         | Yes      |
+--------------------------------+------------------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the ``max_blocking_time`` data member of ReliabilityQosPolicy can be modified any time.

The QoS value of a previously created :ref:`dds_layer_subscriber_dataReader` can be modified using the
:func:`set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDataReaderQos:

Default DataReaderQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_subscriber_dataReaderQos` refers to the value returned by the
:func:`get_default_dataReader_qos` member function on the :ref:`dds_layer_subscriber_subscriber` instance.
The special value ``DATAREADER_QOS_DEFAULT`` can be used as QoS argument on :func:`create_datareader`
or :func:`set_qos` member functions to indicate that the current default :ref:`dds_layer_subscriber_dataReaderQos`
should be used.

When the system starts, the default :ref:`dds_layer_subscriber_dataReaderQos` is equivalent to the default constructed
value :func:`DataReaderQos`.
The default :ref:`dds_layer_subscriber_dataReaderQos` can be modified at any time using the
:func:`set_default_dataReader_qos` member function on the :ref:`dds_layer_subscriber_subscriber` instance.
Modifying the default :ref:`dds_layer_subscriber_dataReaderQos` will not affect already existing
:ref:`dds_layer_subscriber_dataReader` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAREADERQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_dataReader_qos` member function also accepts the special value ``DATAREADER_QOS_DEFAULT``
as input argument.
This will reset the current default :ref:`dds_layer_subscriber_dataReaderQos` to default constructed
value :func:`DataReaderQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``DATAREADER_QOS_DEFAULT`` has different meaning depending on where it is used:
   * On :func:`create_datareader` and :func:`set_qos` it refers to the default
   :ref:`dds_layer_subscriber_dataReaderQos` as returned by :func:`get_default_dataReader_qos`.
   * On :func:`set_default_dataReader_qos` it refers to the default constructed :func:`DataReaderQos`.

