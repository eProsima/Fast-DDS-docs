.. _non-blocking-calls:

Non-blocking calls
==================

.. note::
   As OSX does not support necessary POSIX Real-time features, this feature is not fully supported on OSX.
   The feature is limited by the implementation of `std::timed_mutex` and `std::condition_variable_any`.

Several functions on the :ref:`Fast DDS API<api_reference>` can be blocked
for an undefined period of time if when several operations compete for the control of a resource.
The blocked function cannot continue until the operation that gained the control finishes.

Real-time applications need a predictable behavior, including a predictable maximum time since a function
is called until it returns control.
In order to comply with this restriction, Fast DDS can be configured to limit the maximum blocking time
of these functions.
If the blocking time limit is exceeded, they return control even if they could not finish their job.

This configuration needs two steps:

* Set the CMake option ``-DSTRICT_REALTIME=ON`` during the compilation of the application.
* Configure the maximum blocking times for the functions.

.. |write| replace:: :cpp:func:`DataWriter::write()<eprosima::fastdds::dds::DataWriter::write>`
.. |take| replace:: :cpp:func:`DataReader::take_next_data()<eprosima::fastdds::dds::DataReader::take_next_data>`
.. |read| replace:: :cpp:func:`DataReader::read_next_data()<eprosima::fastdds::dds::DataReader::read_next_data>`
.. |wait| replace:: :cpp:func:`DataReader::wait_for_unread_samples()<eprosima::fastdds::dds::DataReader::wait_for_unread_samples>`

.. list-table:: **Fast RTPS non-blocking API**
   :header-rows: 1
   :align: left

   * - Method
     - Configuration attribute
     - Default value
   * - |write|
     - ``reliability().max_blocking_time`` on :ref:`dds_layer_publisher_dataWriterQos`.
     - 100 milliseconds.
   * - |take|
     - ``reliability().max_blocking_time`` on :ref:`dds_layer_subscriber_dataReaderQos`.
     - 100 milliseconds.
   * - |read|
     - ``reliability().max_blocking_time`` on :ref:`dds_layer_subscriber_dataReaderQos`.
     - 100 milliseconds.
   * - |wait|
     - The method accepts an argument with the maximum blocking time.
     -


