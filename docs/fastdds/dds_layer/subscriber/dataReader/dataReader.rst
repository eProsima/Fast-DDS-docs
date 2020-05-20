.. _dds_layer_subscriber_dataReader:

DataReader
==========


A :ref:`api_pim_datareader` is attached to exactly one
:ref:`dds_layer_subscriber_subscriber` that acts as a factory for it.
Additionally, each :ref:`api_pim_datareader` is bound to a single
:ref:`dds_layer_topic_topic` since its creation.
This :ref:`dds_layer_topic_topic` must exist prior to the creation of the
:ref:`api_pim_datareader`,
and must be bound to the data type that the :ref:`api_pim_datareader` wants to publish.

The effect of creating a new :ref:`api_pim_datareader` in a
:ref:`dds_layer_subscriber_subscriber` for a specific
:ref:`dds_layer_topic_topic` is to initiate a new subscription with the name and data type described
by the :ref:`dds_layer_topic_topic`.

Once the :ref:`api_pim_datareader` is created, the application will be informed
when changes in the data value are received from remote publications.
These changes can then be retrieved using the :func:`read` or :func:`take`
member functions of the :ref:`api_pim_datareader`.

.. _dds_layer_subscriber_dataReaderQos:

DataReaderQos
-------------

:ref:`api_pim_datareaderqos` controls the behavior of the :ref:`dds_layer_subscriber_dataReader`.
Internally it contains the following :class:`QosPolicy` objects:

.. |durability| replace:: :cpp:func:`durability()<eprosima::fastdds::dds::SubscriberQos::durability>`
.. |dur_service| replace:: :cpp:func:`durability_service()<eprosima::fastdds::dds::SubscriberQos::durability_service>`
.. |deadline| replace:: :cpp:func:`deadline()<eprosima::fastdds::dds::SubscriberQos::deadline>`
.. |latency_budget| replace:: :cpp:func:`latency_budget()<eprosima::fastdds::dds::SubscriberQos::latency_budget>`
.. |liveliness| replace:: :cpp:func:`liveliness()<eprosima::fastdds::dds::SubscriberQos::liveliness>`
.. |reliability| replace:: :cpp:func:`reliability()<eprosima::fastdds::dds::SubscriberQos::reliability>`
.. |dest_order| replace:: :cpp:func:`destination_order()<eprosima::fastdds::dds::SubscriberQos::destination_order>`
.. |history| replace:: :cpp:func:`history()<eprosima::fastdds::dds::SubscriberQos::history>`
.. |resource_limits| replace:: :cpp:func:`resource_limits()<eprosima::fastdds::dds::SubscriberQos::resource_limits>`
.. |lifespan| replace:: :cpp:func:`lifespan()<eprosima::fastdds::dds::SubscriberQos::lifespan>`
.. |user_data| replace:: :cpp:func:`user_data()<eprosima::fastdds::dds::SubscriberQos::user_data>`
.. |ownership| replace:: :cpp:func:`ownership()<eprosima::fastdds::dds::SubscriberQos::ownership>`
.. |properties| replace:: :cpp:func:`properties()<eprosima::fastdds::dds::SubscriberQos::properties>`
.. |endpoint| replace:: :cpp:func:`endpoint()<eprosima::fastdds::dds::SubscriberQos::endpoint>`
.. |rs_lim| replace:: :cpp:func:`reader_resource_limits()<eprosima::fastdds::dds::SubscriberQos::reader_resource_limits>`
.. |time_based_filt| replace:: :cpp:func:`time_based_filter()<eprosima::fastdds::dds::SubscriberQos::time_based_filter>`
.. |data_lc| replace:: :cpp:func:`reader_data_lifecycle()<eprosima::fastdds::dds::SubscriberQos::reader_data_lifecycle>`
.. |rel_rd_qos| replace:: :cpp:func:`reliable_reader_qos()<eprosima::fastdds::dds::SubscriberQos::reliable_reader_qos>`
.. |user_data_qos| replace:: :cpp:func:`user_data()<eprosima::fastdds::dds::SubscriberQos::user_data>`
.. |type_consistency| replace:: :cpp:func:`type_consistency()<eprosima::fastdds::dds::SubscriberQos::type_consistency>`
.. |exp_in_qos| replace:: :cpp:func:`expects_inline_qos()<eprosima::fastdds::dds::SubscriberQos::expects_inline_qos>`

+-----------------------------------------------+------------------------------+----------+
| QosPolicy class                               | Accessor/Mutator             | Mutable  |
+===============================================+==============================+==========+
| :ref:`api_pim_durabilityqospolicy`            | |durability|                 | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_durabilityserviceqospolicy`     | |dur_service|                | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_deadlineqospolicy`              | |deadline|                   | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_latencybudgetqospolicy`         | |latency_budget|             | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_livelinessqospolicy`            | |liveliness|                 | No       |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_reliabilityqospolicy`           | |reliability|                | No (*)   |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_destinationorderqospolicy`      | |dest_order|                 | No       |
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
| :ref:`api_pim_propertypolicyqos`              | |properties|                 | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_RTPSendpointqos`                | |endpoint|                   | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_readerresourcelimitsqos`        | |rs_lim|                     | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_timebasedfilterqospolicy`       | |time_based_filt|            | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_readerdatalifecycleqospolicy`   | |data_lc|                    | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_RTPSreliablereaderqos`          | |rel_rd_qos|                 | Yes      |
+-----------------------------------------------+------------------------------+----------+
| :ref:`api_pim_typeconsistencyqos`             | |type_consistency|           | Yes      |
+-----------------------------------------------+------------------------------+----------+
| ``boolean``                                   | |exp_in_qos|                 | Yes      |
+-----------------------------------------------+------------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

.. note::

   Reliability kind (whether the publication is reliable or best effort) is not mutable.
   However, the ``max_blocking_time`` data member of :ref:`api_pim_reliabilityqospolicy` can be modified any time.

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
:func:`get_default_datareader_qos` member function on the
:ref:`dds_layer_subscriber_subscriber` instance.
The special value ``DATAREADER_QOS_DEFAULT`` can be used as QoS argument on
:func:`create_datareader` or
:func:`set_qos` member functions to indicate that the current default
:ref:`dds_layer_subscriber_dataReaderQos` should be used.

When the system starts, the default :ref:`dds_layer_subscriber_dataReaderQos` is equivalent to
the default constructed value :func:`DataReaderQos`.
The default :ref:`dds_layer_subscriber_dataReaderQos` can be modified at any time using the
:func:`set_default_datareader_qos` member function on the
ref:`dds_layer_subscriber_subscriber` instance.
Modifying the default :ref:`dds_layer_subscriber_dataReaderQos` will not affect already existing
:ref:`dds_layer_subscriber_dataReader` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DATAREADERQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_datareader_qos` member function also accepts
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

   * On :func:`create_datareader`
     and :func:`set_qos` it refers to the default
     :ref:`dds_layer_subscriber_dataReaderQos` as returned by
     :func:`get_default_datareader_qos`.
   * On :func:`set_default_dataReader_qos` it refers
     to the default constructed :func:`DataReaderQos`.

