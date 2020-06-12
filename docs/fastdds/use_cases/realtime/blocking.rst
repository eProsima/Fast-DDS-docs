.. _non-blocking-calls:

Non-blocking calls
==================

.. note::
   As OSX does not support necessary POSIX Real-time features, this feature is not fully supported on OSX.
   In that case, the feature is limited by the implementation of `std::timed_mutex` and `std::condition_variable_any`.

Several functions on the :ref:`Fast DDS API<api_reference>` can be blocked
for an undefined period of time when operations compete for the control of a resource.
The blocked function cannot continue until the operation that gained the control finishes, thus blocking
the calling thread.

Real-time applications need a predictable behavior, including a predictable maximum time since a function
is called until it returns control.
In order to comply with this restriction, *Fast DDS* can be configured to limit the maximum blocking time
of these functions.
If the blocking time limit is exceeded, the requested operation is aborted and function terminated,
returning the control to the caller.

This configuration needs two steps:

* Set the CMake option ``-DSTRICT_REALTIME=ON`` during the compilation of the application.
* Configure the maximum blocking times for the functions.

.. |write| replace:: :cpp:func:`DataWriter::write()<eprosima::fastdds::dds::DataWriter::write>`
.. |take| replace:: :cpp:func:`DataReader::take_next_sample()<eprosima::fastdds::dds::DataReader::take_next_sample>`
.. |read| replace:: :cpp:func:`DataReader::read_next_sample()<eprosima::fastdds::dds::DataReader::read_next_sample>`
.. |wait| replace:: :cpp:func:`DataReader::wait_for_unread_message()<eprosima::fastdds::dds::DataReader::wait_for_unread_mesage>`
.. |writer-reliability| replace:: :cpp:func:`reliability().max_blocking_time<eprosima::fastdds::dds::DataWriterQos::reliability>`
.. |reader-reliability| replace:: :cpp:func:`reliability().max_blocking_time<eprosima::fastdds::dds::DataReaderQos::reliability>`

.. list-table:: **Fast RTPS non-blocking API**
   :header-rows: 1
   :align: left

   * - Method
     - Configuration attribute
     - Default value
   * - |write|
     - |writer-reliability| on :ref:`dds_layer_publisher_dataWriterQos`.
     - 100 milliseconds.
   * - |take|
     - |reader-reliability| on :ref:`dds_layer_subscriber_dataReaderQos`.
     - 100 milliseconds.
   * - |read|
     - |reader-reliability| on :ref:`dds_layer_subscriber_dataReaderQos`.
     - 100 milliseconds.
   * - |wait|
     - The method accepts an argument with the maximum blocking time.
     -


