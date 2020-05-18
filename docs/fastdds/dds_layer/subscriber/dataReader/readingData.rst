.. _dds_layer_subscriber_accessreceived:

Accessing received data
=======================

The :ref:`dds_layer_subscriber_dataReader` informs through the
:ref:`dds_layer_subscriber_dataReaderListener` when it receives new data values from any matching
:ref:`dds_layer_publisher_dataWriter`, so that the application knows there is new data to process.
The application can then access and consume the received data values.

This access can be done by *reading* or *taking*.

 * **Reading** is done with the :cpp:func:`eprosima::fastdds::dds::DataReader::read_next_data` member function.
   It reads and returns the next, non-previously accessed data value available on the
   :ref:`dds_layer_subscriber_dataReader`.
 * **Taking** is done with the :cpp:func:`eprosima::fastdds::dds::DataReader::take_next_data` member function.
   It reads and returns the next, non-previously accessed data value available on the
   :ref:`dds_layer_subscriber_dataReader`.
   Additionally, it also removes the value from the :ref:`dds_layer_subscriber_dataReader`,
   so it is no longer accessible.

If there is no unread data in the :ref:`dds_layer_subscriber_dataReader`, both operations will return
``NO_DATA`` and nothing is returned.

In addition to the data value, the data access operations also return a :ref:`dds_layer_subscriber_sampleInfo`
instance with additional information that help interpreting the returned data value, like the originating
:ref:`dds_layer_publisher_dataWriter` or the publication time stamp.
Please, refer to the :ref:`dds_layer_subscriber_sampleInfo` section for an extensive description of its contents.




