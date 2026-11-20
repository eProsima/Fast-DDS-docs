<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-intro"></a>

<a id="logging"></a>

# Logging

*eProsima Fast DDS* provides an extensible built-in logging module that exposes the following main functionalities:

* Three different logging levels: [`Log::Kind::Info`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind4InfoE), [`Log::Kind::Warning`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind7WarningE), and [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE) (see
  [Logging Messages](logging_messages.md#dds-layer-log-logging)).
* Message filtering according to different criteria: category, content, or source file (see
  [Filters](logging_filtering.md#dds-layer-log-filter)).
* Output to STDOUT, STDERR and/or log files (see [Consumers](consumer.md#dds-layer-log-consumer)).

This section is devoted to explain the use, configuration, and extensibility of Fast DDS’ logging module.

* [Module Structure](logging_module.md)
* [Log Entry Specification](logging_spec.md)
* [Logging Thread](logging_thread.md)
* [Logging Messages](logging_messages.md)
* [Module Configuration](logging_configuration.md)
  * [Log Entry](logging_configuration.md#log-entry)
    * [Timestamp](logging_configuration.md#timestamp)
    * [Category](logging_configuration.md#category)
    * [Verbosity Level](logging_configuration.md#verbosity-level)
    * [Message](logging_configuration.md#message)
    * [File Context](logging_configuration.md#file-context)
    * [Function Name](logging_configuration.md#function-name)
  * [Register Consumers](logging_configuration.md#register-consumers)
  * [Reset Configuration](logging_configuration.md#reset-configuration)
  * [XML Configuration](logging_configuration.md#xml-configuration)
* [Filters](logging_filtering.md)
  * [Category Filtering](logging_filtering.md#category-filtering)
  * [File Name Filtering](logging_filtering.md#file-name-filtering)
  * [Content Filtering](logging_filtering.md#content-filtering)
  * [Reset Logging Filters](logging_filtering.md#reset-logging-filters)
* [Consumers](consumer.md)
  * [StdoutConsumer](consumer.md#stdoutconsumer)
  * [StdoutErrConsumer](consumer.md#stdouterrconsumer)
  * [FileConsumer](consumer.md#fileconsumer)
* [Disable Logging Module](disable_module.md)
* [Old Log macros disable](disable_module.md#old-log-macros-disable)
