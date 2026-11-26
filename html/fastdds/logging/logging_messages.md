<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-logging"></a>

<a id="logging-messages"></a>

# Logging Messages

The logging of messages is handled by three dedicated macros, one for each available verbosity level (see
[Verbosity Level](logging_configuration.md#dds-layer-log-verbosity-level)):

* [`EPROSIMA_LOG_INFO`](../api_reference/log/loginfo.md#c.EPROSIMA_LOG_INFO): Logs messages with [`Log::Kind::Info`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind4InfoE) verbosity.
* [`EPROSIMA_LOG_WARNING`](../api_reference/log/logwarning.md#c.EPROSIMA_LOG_WARNING): Logs messages with [`Log::Kind::Warning`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind7WarningE) verbosity.
* [`EPROSIMA_LOG_ERROR`](../api_reference/log/logerror.md#c.EPROSIMA_LOG_ERROR): Logs messages with [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE) verbosity.

Said macros take exactly two arguments, a category and a message, and produce a log entry showing the message itself
plus some meta information depending on the module’s configuration (see [Log Entry Specification](logging_spec.md#dds-layer-log-logging-spec) and
[Log Entry](logging_configuration.md#dds-layer-log-config-entry)).

```c++
EPROSIMA_LOG_INFO(DOCUMENTATION_CATEGORY, "This is an info message");
EPROSIMA_LOG_WARNING(DOCUMENTATION_CATEGORY, "This is an warning message");
EPROSIMA_LOG_ERROR(DOCUMENTATION_CATEGORY, "This is an error message");
```

There exist some old log macros used in previous versions: `logInfo`, `logWarning` and `logError`.
These macros are still available as long as user does not manually disable them by `ENABLE_OLD_LOG_MACROS`
CMake option or in-site macro `ENABLE_OLD_LOG_MACROS_` before including *Log* module.
See section [Old Log macros disable](disable_module.md#old-log-macros-disable) for more information.

#### WARNING
Note that each message level is deactivated when CMake options `LOG_NO_INFO`, `LOG_NO_WARNING` or
`LOG_NO_ERROR` are set to `ON` respectively.
For more information about how to enable and disable each individual logging macro, please refer to
[Disable Logging Module](disable_module.md#dds-layer-log-disable).
