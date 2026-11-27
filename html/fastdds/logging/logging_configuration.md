<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-config"></a>

<a id="module-configuration"></a>

# Module Configuration

The logging module offers a variety of configuration options.
The different components of a log entry (see [Log Entry Specification](logging_spec.md#dds-layer-log-logging-spec)) can be configured as explained in
[Log Entry](#dds-layer-log-config-entry).
Furthermore, the logging module allows for registering several log consumer, allowing applications to direct the
logging output to different destinations (see [Register Consumers](#dds-layer-log-register-consumers)).
In addition, some of the logging features can be configured using *eProsima Fast DDS* XML configuration files (see
[XML Configuration](#dds-layer-log-xml)).

<a id="dds-layer-log-config-entry"></a>

<a id="log-entry"></a>

## Log Entry

All the different components of a log entry are summarized in the following table (please refer to each component’s
section for further explanation):

| Component                                         | Optional   | Default   |
|---------------------------------------------------|------------|-----------|
| [Timestamp](#dds-layer-log-timestamp)             | NO         | ENABLED   |
| [Category](#dds-layer-log-category)               | NO         | ENABLED   |
| [Verbosity Level](#dds-layer-log-verbosity-level) | NO         | ENABLED   |
| [Message](#dds-layer-log-message)                 | NO         | ENABLED   |
| [File Context](#dds-layer-log-file-context)       | YES        | DISABLED  |
| [Function Name](#dds-layer-log-function-name)     | YES        | ENABLED   |

<a id="dds-layer-log-timestamp"></a>

<a id="timestamp"></a>

### Timestamp

The log timestamp follows the [ISO 8601 standard](https://www.iso.org/iso-8601-date-and-time-format.html) for local
timestamps, i.e. *YYYY-MM-DD hh:mm:ss.sss*.
This component cannot be further configured or disabled.

<a id="dds-layer-log-category"></a>

<a id="category"></a>

### Category

Log entries have a category assigned when producing the log via the macros presented in
[Logging Messages](logging_messages.md#dds-layer-log-logging).
The category component can be used to filter log entries so that only those categories specified in the filter are
consumed (see [Filters](logging_filtering.md#dds-layer-log-filter)).
This component cannot be further configured or disabled.

<a id="dds-layer-log-verbosity-level"></a>

<a id="verbosity-level"></a>

### Verbosity Level

*eProsima Fast DDS* logging module provides three verbosity levels defined by the [`Log::Kind`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) enumeration,
those are:

* [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE): Used to log error messages.
* [`Log::Kind::Warning`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind7WarningE): Used to log error and warning messages.
* [`Log::Kind::Info`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind4InfoE): Used to log error, warning, and info messages.

The logging module’s verbosity level defaults to [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE), which means that only messages logged with
[`EPROSIMA_LOG_ERROR`](../api_reference/log/logerror.md#c.EPROSIMA_LOG_ERROR) would be consumed.
The verbosity level can be set and retrieved using member functions [`Log::SetVerbosity()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log12SetVerbosityEN3Log4KindE) and [`Log::GetVerbosity()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log12GetVerbosityEv)
respectively.

```c++
// Set log verbosity level to Log::Kind::Info
Log::SetVerbosity(Log::Kind::Info);

// Get log verbosity level
Log::Kind verbosity_level = Log::GetVerbosity();
```

#### WARNING
Setting any of the CMake options `LOG_NO_INFO`, `LOG_NO_WARNING` or `LOG_NO_ERROR` to `ON`
will completely disable the corresponding verbosity level.
`LOG_NO_INFO` is set to `ON` for Single-Config generators as default value if not in `Debug` mode.

<a id="dds-layer-log-message"></a>

<a id="message"></a>

### Message

This component constitutes the body of the log entry.
It is specified when producing the log via the macros presented in [Logging Messages](logging_messages.md#dds-layer-log-logging).
The message component can be used to filter log entries so that only those entries whose message pattern-matches the
filter are consumed (see [Filters](logging_filtering.md#dds-layer-log-filter)).
This component cannot be further configured or disabled.

<a id="dds-layer-log-file-context"></a>

<a id="file-context"></a>

### File Context

This component specifies the origin of the log entry in terms of file name and line number (see
[Logging Messages](logging_messages.md#dds-layer-log-logging) for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The file context component can be enabled/disabled using the member function [`Log::ReportFilenames()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log15ReportFilenamesEb).

```c++
// Enable file name and line number reporting
Log::ReportFilenames(true);

// Disable file name and line number reporting
Log::ReportFilenames(false);
```

<a id="dds-layer-log-function-name"></a>

<a id="function-name"></a>

### Function Name

This component specifies the origin of the log entry in terms of the function name (see
[Logging Messages](logging_messages.md#dds-layer-log-logging) for a log entry example featuring this component).
This is useful when tracing code flow for debugging purposes.
The function name component can be enabled/disabled using the member function [`Log::ReportFunctions()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log15ReportFunctionsEb).

```c++
// Enable function name reporting
Log::ReportFunctions(true);

// Disable function name reporting
Log::ReportFunctions(false);
```

<a id="dds-layer-log-register-consumers"></a>

<a id="register-consumers"></a>

## Register Consumers

*eProsima Fast DDS* logging module supports zero or more [consumers](consumer.md#dds-layer-log-consumer) logging the entries
registered in the logging queue with the methods described in [Logging Messages](logging_messages.md#dds-layer-log-logging).
To register a consumer, the [`Log`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE) class exposes member function [`Log::RegisterConsumer()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log16RegisterConsumerERRNSt10unique_ptrI11LogConsumerEE)

```c++
// Create a FileConsumer consumer that logs entries in "archive.log"
std::unique_ptr<FileConsumer> file_consumer(new FileConsumer("archive.log"));
// Register the consumer. Log entries will be logged to STDOUT and "archive.log"
Log::RegisterConsumer(std::move(file_consumer));
```

The consumers list can be emptied with member function [`Log::ClearConsumers()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log14ClearConsumersEv).

```c++
// Clear all the consumers. Log entries are discarded upon consumption.
Log::ClearConsumers();
```

#### NOTE
Registering and configuring consumers can also be done using *Fast DDS* XML configuration files.
Please refer to [XML Configuration](#dds-layer-log-xml) for details.

#### WARNING
[`Log::ClearConsumers()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log14ClearConsumersEv) empties the consumers lists.
All log entries are discarded until a new consumer is register via [`Log::RegisterConsumer()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log16RegisterConsumerERRNSt10unique_ptrI11LogConsumerEE), or until
[`Log::Reset()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5ResetEv) is called.

<a id="dds-layer-log-reset"></a>

<a id="reset-configuration"></a>

## Reset Configuration

The logging module’s configuration can be reset to default settings with member function [`Log::Reset()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5ResetEv).

#### WARNING
Resetting the module’s configuration entails:

* Setting [Verbosity Level](#dds-layer-log-verbosity-level) to [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE).
* Disabling [File Context](#dds-layer-log-file-context) component.
* Enabling [Function Name](#dds-layer-log-function-name) component.
* Clear all [Filters](logging_filtering.md#dds-layer-log-filter).
* Clear all consumers and reset the default consumer according to CMake option `LOG_CONSUMER_DEFAULT`.

<a id="dds-layer-log-xml"></a>

<a id="xml-configuration"></a>

## XML Configuration

*eProsima Fast DDS* allows for registering and configuring log consumers using XML configuration files.
Please refer to [Log profiles](../xml_configuration/log.md#logprofiles) for details.
