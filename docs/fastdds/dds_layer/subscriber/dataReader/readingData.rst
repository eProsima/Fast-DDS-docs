.. _dds_layer_subscriber_accessreceived:

Accessing received data
=======================

The application can access and consume the data values received on the :ref:`dds_layer_subscriber_dataReader`
by *reading* or *taking*.

 * **Reading** is done with the :func:`read_next_data` member function.
   It reads the next, non-previously accessed data value available on the
   :ref:`dds_layer_subscriber_dataReader`, and stores it in the provided data buffer.
 * **Taking** is done with the :func:`take_next_data` member function.
   It reads the next, non-previously accessed data value available on the
   :ref:`dds_layer_subscriber_dataReader`, and stores it in the provided data buffer.
   Additionally, it also removes the value from the :ref:`dds_layer_subscriber_dataReader`,
   so it is no longer accessible.

If there is no unread data in the :ref:`dds_layer_subscriber_dataReader`, both operations will return
``NO_DATA`` and nothing is returned.

In addition to the data value, the data access operations also provide a :ref:`dds_layer_subscriber_sampleInfo`
instance with additional information that help interpreting the returned data value, like the originating
:ref:`dds_layer_publisher_dataWriter` or the publication time stamp.
Please, refer to the :ref:`dds_layer_subscriber_sampleInfo` section for an extensive description of its contents.


.. _dds_layer_subscriber_accessreceived_listener:

Accessing data on callbacks
---------------------------

When the :ref:`dds_layer_subscriber_dataReader` new data values from any matching
:ref:`dds_layer_publisher_dataWriter`, it informs the application through
two Listener callbacks:

* :cpp:func:`on_data_available<eprosima::fastdds::dds::DataReaderListener::on_data_available>`.

* :cpp:func:`on_data_on_readers<eprosima::fastdds::dds::SubscriberListener::on_data_on_readers>`

These callbacks can be used to retrieve the newly arrived data, as in the following example.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_READ_LISTENER
   :end-before: //!


.. _dds_layer_subscriber_accessreceived_wait:

Accessing data with a waiting thread
------------------------------------

Instead of relying on the Listener to try and get new data values,
the application can also dedicate a thread to wait until any new data is available on the
:ref:`dds_layer_subscriber_dataReader`.
This can be done with the :func:`wait_for_unread_message` member function,
that blocks until a new data sample is available or the given timeout expires.
If no new data was available after the timeout expired, it will return with value ``false``.
This function returning with value ``true`` means there is new data available on the
:ref:`dds_layer_subscriber_dataReaderListener` ready for the application to retrieve.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_READ_WAIT
   :end-before: //!
   :dedent: 8

