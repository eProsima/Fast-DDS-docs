.. _dds_layer_subscriber_dataReader:

DataReader
==========


A :cpp:class:`eprosima::fastdds::dds::DataReader` is attached to exactly one
:ref:`dds_layer_subscriber_subscriber` that acts as a factory for it.
Additionally, each :cpp:class:`eprosima::fastdds::dds::DataReader` is bound to a single
:ref:`dds_layer_topic_topic` since its creation.
This :ref:`dds_layer_topic_topic` must exist prior to the creation of the
:cpp:class:`eprosima::fastdds::dds::DataReader`,
and must be bound to the data type that the :cpp:class:`eprosima::fastdds::dds::DataReader` wants to publish.

The effect of creating a new :cpp:class:`eprosima::fastdds::dds::DataReader` in a
:ref:`dds_layer_subscriber_subscriber` for a specific
:ref:`dds_layer_topic_topic` is to initiate a new subscription with the name and data type described
by the :ref:`dds_layer_topic_topic`.

Once the :cpp:class:`eprosima::fastdds::dds::DataReader` is created, the application will be informed
when changes in the data value are received from remote publications, and can then retrieve this changes
using the :cpp:func:`eprosima::fastdds::dds::DataReader::read` or :cpp:func:`eprosima::fastdds::dds::DataReader::take`
member functions of the :cpp:class:`eprosima::fastdds::dds::DataReader`.

.. _dds_layer_subscriber_dataReaderQos:

DataReaderQos
-------------

:cpp:class:`eprosima::fastdds::dds::DataReaderQos` controls the behavior of the :ref:`dds_layer_subscriber_dataReader`.
Internally it contains the following :class:`QosPolicy` objects:

.. |durability| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::durability`
.. |durability_service| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::durability_service`
.. |deadline| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::deadline`
.. |latency_budget| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::latency_budget`
.. |liveliness| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::liveliness`
.. |reliability| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::reliability`
.. |destination_order| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::destination_order`
.. |history| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::history`
.. |resource_limits| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::resource_limits`
.. |lifespan| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::lifespan`
.. |user_data| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::user_data`
.. |ownership| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::ownership`
.. |properties| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::properties`
.. |endpoint| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::endpoint`
.. |reader_resource_limits| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::reader_resource_limits`
.. |time_based_filter| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::time_based_filter`
.. |reader_data_lifecycle| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::reader_data_lifecycle`
.. |reliable_reader_qos| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::reliable_reader_qos`
.. |user_data_qos| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::user_data`
.. |type_consistency| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::type_consistency`
.. |expects_inline_qos| replace:: :cpp:func:`eprosima::fastdds::dds::SubscriberQos::expects_inline_qos`

+-----------------------------------------------+------------------------------+----------+
| QosPolicy class                               | Accessor/Mutator             | Mutable  |
+===============================================+==============================+==========+
| :ref:`api_pim_durabilityqospolicy`            | |durability|                 | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_durabilityserviceqospolicy`     | |durability_service|         | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_deadlineqospolicy`              | |deadline|                   | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_latencybudgetqospolicy`         | |latency_budget|             | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_livelinessqospolicy`            | |liveliness|                 | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_reliabilityqospolicy`           | |reliability|                | No (*)   |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_destinationorderqospolicy`      | |destination_order|          | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_historyqospolicy`               | |history|                    | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_resourcelimitsqospolicy`        | |resource_limits|            | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_lifespanqospolicy`              | |lifespan|                   | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_userdataqospolicy`              | |user_data_qos|              | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_ownershipqospolicy`             | |ownership|                  | No       |
+-----------------------------------------------+------------------------------+----------+
| ``PropertyPolicyQos``                         | |properties|                 | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_RTPSendpointqos`                | |endpoint|                   | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_readerresourcelimitsqos`        | |reader_resource_limits|     | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_timebasedfilterqospolicy`       | |time_based_filter|          | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_readerdatalifecycleqospolicy`   | |reader_data_lifecycle|      | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_RTPSreliablereaderqos`          | |reliable_reader_qos|        | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_typeconsistencyqos`             | |type_consistency|           | Yes      |
+-----------------------------------------------+------------------------------+----------+
| ``boolean``                                   | |expects_inline_qos|         | Yes      |
+-----------------------------------------------+------------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the ``max_blocking_time`` data member of :ref:`api_pim_reliabilityqospolicy` can be modified any time.

The QoS value of a previously created :ref:`dds_layer_subscriber_dataReader` can be modified using the
:cpp:func:`eprosima::fastdds::dds::DataReader::set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDataReaderQos:

Default DataReaderQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_subscriber_dataReaderQos` refers to the value returned by the
:cpp:func:`eprosima::fastdds::dds::Subscriber::get_default_dataReader_qos` member function on the
:ref:`dds_layer_subscriber_subscriber` instance.
The special value ``DATAREADER_QOS_DEFAULT`` can be used as QoS argument on
:cpp:func:`eprosima::fastdds::dds::Subscriber::create_datareader` or
:cpp:func:`eprosima::fastdds::dds::DataReader::set_qos` member functions to indicate that the current default
:ref:`dds_layer_subscriber_dataReaderQos` should be used.

When the system starts, the default :ref:`dds_layer_subscriber_dataReaderQos` is equivalent to
the default constructed value :func:`DataReaderQos`.
The default :ref:`dds_layer_subscriber_dataReaderQos` can be modified at any time using the
:cpp:func:`eprosima::fastdds::dds::Subscriber::set_default_dataReader_qos` member function on the
ref:`dds_layer_subscriber_subscriber` instance.
Modifying the default :ref:`dds_layer_subscriber_dataReaderQos` will not affect already existing
:ref:`dds_layer_subscriber_dataReader` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAREADERQOS
   :end-before: //!
   :dedent: 8

:cpp:func:`eprosima::fastdds::dds::Subscriber::set_default_dataReader_qos` member function also accepts
the special value ``DATAREADER_QOS_DEFAULT`` as input argument.
This will reset the current default :ref:`dds_layer_subscriber_dataReaderQos` to default constructed
value :func:`DataReaderQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DATAREADERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``DATAREADER_QOS_DEFAULT`` has different meaning depending on where it is used:
   * On :cpp:func:`eprosima::fastdds::dds::Subscriber::create_datareader`
   and :cpp:func:`eprosima::fastdds::dds::DataReader::set_qos` it refers to the default
   :ref:`dds_layer_subscriber_dataReaderQos` as returned by
   :cpp:func:`eprosima::fastdds::dds::Subscriber::get_default_dataReader_qos`.
   * On :cpp:func:`eprosima::fastdds::dds::Subscriber::set_default_dataReader_qos` it refers
   to the default constructed :func:`DataReaderQos`.

