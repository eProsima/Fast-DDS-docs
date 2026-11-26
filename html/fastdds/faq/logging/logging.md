<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-logging-questions"></a>

<a id="logging-frequently-asked-questions"></a>

# Logging Frequently Asked Questions

**What are the functionalities of the logging module?**

<br/> There are three functionalities in the logging module: three logging levels, for info, warnings and errors; message filtering; and output to `STDOUT`, `STDERR` and log files.
For further information, refer to [Logging](../../logging/intro.md#dds-layer-log-intro).

---

**What are the main classes in the logging module?**

<br/> The `Log` class is in charge of the logging operations and provides configuration APIs to set different logging configuration aspects and logging filtering at various levels.
The `LogConsumer` class includes the member functions that derived classes should overload to consume log entries.
For further information, refer to [Module Structure](../../logging/logging_module.md#dds-layer-log-classes).

---

**How does the logging module prevent blocking of the application thread when a logging operation is performed?**

<br/> When it is created, the logging module creates a thread that awakens every time an entry is added to the queue and falls back into idle state once the work is done.
For further information, refer to [Logging Thread](../../logging/logging_thread.md#dds-layer-log-thread).

---

**How is the logging of messages handled?**

<br/> It is handled by three macros, for log messages with `Log::Kind::Info`, `Log::Kind::Warning`, and `Log::Kind::Info` verbosities.
These macros produce a log entry showing a message and some meta information.
For further information, refer to [Logging Messages](../../logging/logging_messages.md#dds-layer-log-logging).

---

**What is the purpose of assigning a category to log entries in the logging module?**

<br/> The category component can be used to filter log entries so that only those categories specified in the filter are consumed.
For further information, refer to [Category](../../logging/logging_configuration.md#dds-layer-log-category).

---

**What are the three different filtering possibilities provided by the \*eProsima Fast DDS\* logging module for log entry filtering?**

<br/> Fast DDS provides three different filtering possibilities: Category Filtering, File Name Filtering, Content Filtering.
For further information, refer to [Filters](../../logging/logging_filtering.md#dds-layer-log-filter).

---

**What is the order in which log entry filters are applied when consuming logs in \*eProsima Fast DDS\*?**

<br/> Filters are applied in the specific order presented in the previous question, meaning that file name filtering is only applied to the entries that pattern-match the category filter, and content filtering is only applied to the entries that pattern-match both category and file name filters.
For further information, refer to [Filters](../../logging/logging_filtering.md#dds-layer-log-filter).

---

**What is the primary method for filtering log entries by their Category component in the \*eProsima Fast DDS\* logging module?**

<br/> Log entries can be filtered upon consumption according to their Category component using regular expressions.
Each time an entry is ready to be consumed, the category filter is applied using `std::regex_search()`.
To set a category filter, the member function [`Log::SetCategoryFilter()`](../../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log17SetCategoryFilterERKNSt5regexE) is used.
For further information, refer to [Category Filtering](../../logging/logging_filtering.md#dds-layer-log-filter-category).

---

**What is the purpose of setting a file name filter in the context of log entry consumption?**

<br/> Log entries can be filtered upon consumption according to their File Context component using regular expressions.
Each time an entry is ready to be consumed, the file name filter is applied using `std::regex_search()`.
To set a file name filter, the member function [`Log::SetFilenameFilter()`](../../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log17SetFilenameFilterERKNSt5regexE) is used.
For further information, refer to [File Name Filtering](../../logging/logging_filtering.md#dds-layer-log-filter-filename).

---

**What is the method for filtering log entries in terms of their message component?**

<br/> Log entries can be filtered upon consumption according to their Message component using regular expressions.
Each time an entry is ready to be consumed, the content filter is applied using `std::regex_search()`.
To set a content filter, the member function [`Log::SetErrorStringFilter()`](../../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log20SetErrorStringFilterERKNSt5regexE) is used.
For further information, refer to [Content Filtering](../../logging/logging_filtering.md#dds-layer-log-filter-content).

---

**What are consumers?**

<br/> Consumers are classes that take a [`Log::Entry`](../../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5EntryE) and produce a log output accordingly.
For further information, refer to [Consumers](../../logging/consumer.md#dds-layer-log-consumer).

---

**What is the function of a "StdoutErrConsumer" in the context of log output?**

<br/> `StdoutErrConsumer`: Outputs log entries to `STDOUT` or `STDERR` depending on the given threshold.
For further information, refer to [Consumers](../../logging/consumer.md#dds-layer-log-consumer).

---

**What is the primary function of the "FileConsumer" class in the context of logging?**

<br/> `FileConsumer`: Outputs log entries to a user specified file.
For further information, refer to [FileConsumer](../../logging/consumer.md#dds-layer-ostream-consumer-file).

---

**What is the primary function of the "StdoutConsumer" class, as described in the provided text?**

<br/> `StdoutConsumer` outputs log entries to the `STDOUT` stream following the convention specified in the [Log Entry Specification](../../logging/logging_spec.md#dds-layer-log-logging-spec) .
It is the default logging module if the CMake option `LOG_CONSUMER_DEFAULT` is set to `STDOUT`.

<br/>
