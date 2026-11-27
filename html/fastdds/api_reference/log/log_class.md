<a id="api-log-log"></a>

<a id="log"></a>

# Log

### class Log

Logging utilities. Logging is accessed through the three macros above, and configuration on the log output can be achieved through static methods on the class. Logging at various levels can be disabled dynamically (through the Verbosity level) or statically (through the LOG_NO_[VERB] macros) for maximum performance. 

### Public Types

### enum Kind

Types of log entry.
* Error: Maximum priority. Can only be disabled statically through LOG_NO_ERROR.
* Warning: Medium priority. Can be disabled statically and dynamically.
* Info: Low priority. Useful for debugging. Disabled by default on release branches. 

*Values:*

### enumerator Error

### enumerator Warning

### enumerator Info

### Public Static Functions

### static void RegisterConsumer(std::unique_ptr<[LogConsumer](log_consumer.md#_CPPv4N8eprosima7fastdds3dds11LogConsumerE)> &&consumer)

Registers an user defined consumer to route log output. There is a default stdout consumer active as default. 

* **Parameters:**
  **consumer** – r-value to a consumer unique_ptr. It will be invalidated after the call. 

### static void ClearConsumers()

Removes all registered consumers, including the default stdout. 

### static void ReportFilenames(bool)

Enables the reporting of filenames in log entries. Disabled by default. 

### static void ReportFunctions(bool)

Enables the reporting of function names in log entries. Enabled by default when supported. 

### static void SetVerbosity([Log](#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](#_CPPv4N8eprosima7fastdds3dds3Log4KindE))

Sets the verbosity level, allowing for messages equal or under that priority to be logged. 

### static [Log](#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](#_CPPv4N8eprosima7fastdds3dds3Log4KindE) GetVerbosity()

Returns the current verbosity level. 

### static void SetCategoryFilter(const std::regex&)

Sets a filter that will pattern-match against log categories, dropping any unmatched categories. 

### static void UnsetCategoryFilter()

Unset the category filter. 

### static bool HasCategoryFilter()

Returns whether a category filter was set or not. 

### static std::regex GetCategoryFilter()

Returns a copy of the current category filter or an empty object otherwise. 

### static void SetFilenameFilter(const std::regex&)

Sets a filter that will pattern-match against filenames, dropping any unmatched categories. 

### static std::regex GetFilenameFilter()

Returns a copy of the current filename filter or an empty object otherwise. 

### static void SetErrorStringFilter(const std::regex&)

Sets a filter that will pattern-match against the provided error string, dropping any unmatched categories. 

### static void SetThreadConfig(const rtps::[ThreadSettings](../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE)&)

Sets thread configuration for the logging thread. 

### static std::regex GetErrorStringFilter()

Returns a copy of the current error string filter or an empty object otherwise. 

### static void Reset()

Returns the logging engine to configuration defaults. 

### static void Flush()

Waits until all info logged up to the call time is consumed. 

### static void KillThread()

Stops the logging thread. It will re-launch on the next call to a successful log macro. 

### static void QueueLog(const std::string &message, const [Log](#_CPPv4N8eprosima7fastdds3dds3LogE)::[Context](#_CPPv4N8eprosima7fastdds3dds3Log7ContextE)&, [Log](#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](#_CPPv4N8eprosima7fastdds3dds3Log4KindE))

Not recommended to call this method directly! Use the following macros:
* [EPROSIMA_LOG_INFO(cat, msg)](loginfo.md#Log_8hpp_1afc47370188af10adcb3cd947706cb7e7);
* [EPROSIMA_LOG_WARNING(cat, msg)](logwarning.md#Log_8hpp_1ae8c4659af1126e27c8d0ed893b2810ab);
* [EPROSIMA_LOG_ERROR(cat, msg)](logerror.md#Log_8hpp_1a1e19903dcfd0d0fea83b75746ceadd4b);

This is a very high sensible point of the code and it should be refactored to be as efficient as possible. 

### struct Context

### struct Entry
