<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-consumer"></a>

<a id="consumers"></a>

# Consumers

Consumers are classes that take a [`Log::Entry`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5EntryE) and produce a log output accordingly.
*eProsima Fast DDS* provides three different log consumers that output log entries to different streams:

* [StdoutConsumer](#dds-layer-ostream-consumer-stdout): Outputs log entries to STDOUT
* [StdoutErrConsumer](#dds-layer-ostream-consumer-stdouterr): Outputs log entries to STDOUT or STDERR depending on the given threshold.
* [FileConsumer](#dds-layer-ostream-consumer-file): Outputs log entries to a user specified file.

<a id="dds-layer-ostream-consumer-stdout"></a>

<a id="stdoutconsumer"></a>

## StdoutConsumer

[`StdoutConsumer`](../api_reference/log/stdout_consumer.md#_CPPv4N8eprosima7fastdds3dds14StdoutConsumerE) outputs log entries to STDOUT stream following the convection specified in
[Log Entry Specification](logging_spec.md#dds-layer-log-logging-spec).
It is the default logging module if the CMake option `LOG_CONSUMER_DEFAULT` is set to `STDOUT`.
It can be registered and unregistered using the methods explained in
[Register Consumers](logging_configuration.md#dds-layer-log-register-consumers) and [Reset Configuration](logging_configuration.md#dds-layer-log-reset).

```c++
// Create a StdoutConsumer consumer that logs entries to stdout stream.
std::unique_ptr<StdoutConsumer> stdout_consumer(new StdoutConsumer());

// Register the consumer.
Log::RegisterConsumer(std::move(stdout_consumer));
```

<a id="dds-layer-ostream-consumer-stdouterr"></a>

<a id="stdouterrconsumer"></a>

## StdoutErrConsumer

[`StdoutErrConsumer`](../api_reference/log/stdouterr_consumer.md#_CPPv4N8eprosima7fastdds3dds17StdoutErrConsumerE) uses a [`Log::Kind`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) threshold to filter the output of the log entries.
Those log entries whose [`Log::Kind`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) is equal to or more severe than the given threshold output to STDERR.
Other log entries output to STDOUT.
It is the default and only log consumer of the logging module if the CMake option `LOG_CONSUMER_DEFAULT` is set to
`AUTO`, `STDOUTERR`, or not set at all.
By default, the threshold is set to [`Log::Kind::Warning`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind7WarningE).
[`StdoutErrConsumer::stderr_threshold()`](../api_reference/log/stdouterr_consumer.md#_CPPv4N8eprosima7fastdds3dds17StdoutErrConsumer16stderr_thresholdERKN3Log4KindE) allows the user to modify the default threshold.

Additionally, if CMake option `LOG_CONSUMER_DEFAULT` is set to `STDOUTERR`, the logging module will use this consumer
as the default log consumer.

```c++
// Create a StdoutErrConsumer consumer that logs entries to stderr only when the Log::Kind is equal to ERROR
std::unique_ptr<StdoutErrConsumer> stdouterr_consumer(new StdoutErrConsumer());
stdouterr_consumer->stderr_threshold(Log::Kind::Error);

// Register the consumer
Log::RegisterConsumer(std::move(stdouterr_consumer));
```

<a id="dds-layer-ostream-consumer-file"></a>

<a id="fileconsumer"></a>

## FileConsumer

[`FileConsumer`](../api_reference/log/file_consumer.md#_CPPv4N8eprosima7fastdds3dds12FileConsumerE) provides the logging module with log-to-file logging capabilities.
Applications willing to hold a persistent execution log record can specify a logging file using this consumer.
Furthermore, the application can choose whether the file stream should be in “write” or “append” mode, according to the
behaviour defined by `std::fstream::open()`.

```c++
// Create a FileConsumer consumer that logs entries in "archive_1.log", opening the file in "write" mode.
std::unique_ptr<FileConsumer> write_file_consumer(new FileConsumer("archive_1.log", false));

// Create a FileConsumer consumer that logs entries in "archive_2.log", opening the file in "append" mode.
std::unique_ptr<FileConsumer> append_file_consumer(new FileConsumer("archive_2.log", true));

// Register the consumers.
Log::RegisterConsumer(std::move(write_file_consumer));
Log::RegisterConsumer(std::move(append_file_consumer));
```
