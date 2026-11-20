<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-classes"></a>

<a id="module-structure"></a>

# Module Structure

The logging module provides the following classes:

* [`Log`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE) is the core class of the logging module.
  This singleton is not only in charge of the logging operations (see [Logging Messages](logging_messages.md#dds-layer-log-logging)), but it also
  provides configuration APIs to set different logging configuration aspects (see [Module Configuration](logging_configuration.md#dds-layer-log-config)), as well
  as logging filtering at various levels (see [Filters](logging_filtering.md#dds-layer-log-filter)).
  It contains zero or more [`LogConsumer`](../api_reference/log/log_consumer.md#_CPPv4N8eprosima7fastdds3dds11LogConsumerE) objects.
  The singleton’s consuming thread feeds the log entries added to the logging queue using the macros defined in
  [Logging Messages](logging_messages.md#dds-layer-log-logging) to the log consumers sequentially (see [Logging Thread](logging_thread.md#dds-layer-log-thread)).

  #### WARNING
  [`Log`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE) API exposes member function [`Log::QueueLog()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log8QueueLogERKNSt6stringERKN3Log7ContextEN3Log4KindE).
  However, this function is not intended to be used directly.
  To add messages to the log queue, use the methods described in [Logging Messages](logging_messages.md#dds-layer-log-logging).
* [`LogConsumer`](../api_reference/log/log_consumer.md#_CPPv4N8eprosima7fastdds3dds11LogConsumerE) is the base class for all the log consumers (see [Consumers](consumer.md#dds-layer-log-consumer)).
  It includes the member functions that derived classes should overload to consume log entries.
  - [`OStreamConsumer`](../api_reference/log/ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE) derives from [`LogConsumer`](../api_reference/log/log_consumer.md#_CPPv4N8eprosima7fastdds3dds11LogConsumerE).
    It defines how to consume log entries for outputting to an `std::ostream` object.
    It includes a member function that derived classes must overload to define the desired `std::ostream` object.

    1. [`StdoutConsumer`](../api_reference/log/stdout_consumer.md#_CPPv4N8eprosima7fastdds3dds14StdoutConsumerE) derives from [`OStreamConsumer`](../api_reference/log/ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE).
    It defines STDOUT as the output `std::ostream` object (see [StdoutConsumer](consumer.md#dds-layer-ostream-consumer-stdout)).

    2. [`StdoutErrConsumer`](../api_reference/log/stdouterr_consumer.md#_CPPv4N8eprosima7fastdds3dds17StdoutErrConsumerE) derives from [`OStreamConsumer`](../api_reference/log/ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE).
    It defines a [`Log::Kind`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) threshold so that if the [`Log::Kind`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) is equal to or more severe than the selected
    threshold, the output defined will be STDERR.
    Otherwise, it defines STDOUT as the output (see [StdoutErrConsumer](consumer.md#dds-layer-ostream-consumer-stdouterr)).

    3. [`FileConsumer`](../api_reference/log/file_consumer.md#_CPPv4N8eprosima7fastdds3dds12FileConsumerE) derives from [`OStreamConsumer`](../api_reference/log/ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE).
    It defines an user specified file as the output `std::ostream` object
    (see [FileConsumer](consumer.md#dds-layer-ostream-consumer-file)).

![image](01-figures/fast_dds/log/class_diagram.svg)

*Logging module class diagram*

The module can be further extended by creating new consumer classes deriving from [`LogConsumer`](../api_reference/log/log_consumer.md#_CPPv4N8eprosima7fastdds3dds11LogConsumerE) and/or
[`OStreamConsumer`](../api_reference/log/ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE).
To enable a custom consumer just follow the instructions on [Register Consumers](logging_configuration.md#dds-layer-log-register-consumers).
