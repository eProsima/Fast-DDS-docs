<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-filter"></a>

<a id="filters"></a>

# Filters

*eProsima Fast DDS* logging module allows for log entry filtering when consuming the logs, so that an application
execution output can be limited to specific areas of interest.
Beside the [Verbosity Level](logging_configuration.md#dds-layer-log-verbosity-level), *Fast DDS* provides three different filtering possibilities.

It is worth mentioning that filters are applied in the specific order presented above, meaning that file name filtering
is only applied to the entries that pattern-match the category filter, and content filtering is only applied to the
entries that pattern-match both category and file name filters.

<a id="dds-layer-log-filter-category"></a>

<a id="category-filtering"></a>

## Category Filtering

Log entries can be filtered upon consumption according to their [Category](logging_configuration.md#dds-layer-log-category) component using regular
expressions.
Each time an entry is ready to be consumed, the category filter is applied using `std::regex_search()`.
To set a category filter, member function [`Log::SetCategoryFilter()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log17SetCategoryFilterERKNSt5regexE) is used:

```c++
// Set filter using regular expression
Log::SetCategoryFilter(std::regex("(CATEGORY_1)|(CATEGORY_2)"));

// Would be consumed
EPROSIMA_LOG_ERROR(CATEGORY_1, "First log entry");
// Would be consumed
EPROSIMA_LOG_ERROR(CATEGORY_2, "Second log entry");
// Would NOT be consumed
EPROSIMA_LOG_ERROR(CATEGORY_3, "Third log entry");
```

The previous example would produce the following output:

```default
2020-05-27 15:07:05.771 [CATEGORY_FILTER_1 Error] First log entry -> Function main
2020-05-27 15:07:05.771 [CATEGORY_FILTER_2 Error] Second log entry -> Function main
```

<a id="dds-layer-log-filter-filename"></a>

<a id="file-name-filtering"></a>

## File Name Filtering

Log entries can be filtered upon consumption according to their [File Context](logging_configuration.md#dds-layer-log-file-context) component using
regular expressions.
Each time an entry is ready to be consumed, the file name filter is applied using `std::regex_search()`.
To set a file name filter, member function [`Log::SetFilenameFilter()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log17SetFilenameFilterERKNSt5regexE) is used:

```c++
// Filename: example.cpp

// Enable file name and line number reporting
Log::ReportFilenames(true);

// Set filter using regular expression so filename must match "example"
Log::SetFilenameFilter(std::regex("example"));
// Would be consumed
EPROSIMA_LOG_ERROR(CATEGORY, "First log entry");

// Set filter using regular expression so filename must match "other"
Log::SetFilenameFilter(std::regex("other"));
// Would NOT be consumed
EPROSIMA_LOG_ERROR(CATEGORY, "Second log entry");
```

The previous example would produce the following output:

```default
2020-05-27 15:07:05.771 [CATEGORY Error] First log entry (example.cpp:50) -> Function main
```

#### NOTE
File name filters are applied even when the [File Context](logging_configuration.md#dds-layer-log-file-context) entry component is disabled.

<a id="dds-layer-log-filter-content"></a>

<a id="content-filtering"></a>

## Content Filtering

Log entries can be filtered upon consumption according to their [Message](logging_configuration.md#dds-layer-log-message) component using regular
expressions.
Each time an entry is ready to be consumed, the content filter is applied using `std::regex_search()`.
To set a content filter, member function [`Log::SetErrorStringFilter()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log20SetErrorStringFilterERKNSt5regexE) is used:

```c++
// Set filter using regular expression so message component must match "First"
Log::SetErrorStringFilter(std::regex("First"));
// Would be consumed
EPROSIMA_LOG_ERROR(CATEGORY, "First log entry");
// Would NOT be consumed
EPROSIMA_LOG_ERROR(CATEGORY, "Second log entry");
```

The previous example would produce the following output:

```default
2020-05-27 15:07:05.771 [CATEGORY Error] First log entry -> Function main
```

<a id="reset-logging-filters"></a>

## Reset Logging Filters

The logging module’s filters can be reset with member function [`Log::Reset()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5ResetEv).

#### WARNING
Resetting the module’s filters entails:

* Setting [Verbosity Level](logging_configuration.md#dds-layer-log-verbosity-level) to [`Log::Kind::Error`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind5ErrorE).
* Disabling [File Context](logging_configuration.md#dds-layer-log-file-context) component.
* Enabling [Function Name](logging_configuration.md#dds-layer-log-function-name) component.
* Clear all [Filters](#dds-layer-log-filter).
* Clear all consumers and reset the default consumer according to CMake option `LOG_CONSUMER_DEFAULT`.
