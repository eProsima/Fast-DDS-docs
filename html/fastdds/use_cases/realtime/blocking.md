<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="non-blocking-calls"></a>

<a id="id1"></a>

# Non-blocking calls

#### NOTE
As OSX does not support necessary POSIX Real-time features, this feature is not fully supported on OSX.
In that case, the feature is limited by the implementation of std::timed_mutex and std::condition_variable_any.

Several functions on the [Fast DDS API](../../api_reference/api_reference.md#api-reference) can be blocked
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

* Set the CMake option `-DSTRICT_REALTIME=ON` during the compilation of the application.
* Configure the maximum blocking times for the functions.

#### **Fast RTPS non-blocking API**

| Method                                                                                                                                                              | Configuration attribute                                                                                                                                                                                                                                             | Default value     |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------|
| [`DataWriter::write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv)                                    | [`reliability().max_blocking_time`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos11reliabilityEv) on [DataWriterQos](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).    | 100 milliseconds. |
| [`DataReader::take_next_sample()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16take_next_sampleEPvP10SampleInfo) | [`reliability().max_blocking_time`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos11reliabilityEv) on [DataReaderQos](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos). | 100 milliseconds. |
| [`DataReader::read_next_sample()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16read_next_sampleEPvP10SampleInfo) | [`reliability().max_blocking_time`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos11reliabilityEv) on [DataReaderQos](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos). | 100 milliseconds. |
| `DataReader::wait_for_unread_message()`                                                                                                                             | The method accepts an argument with the maximum blocking time.                                                                                                                                                                                                      |                   |
