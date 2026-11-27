<a id="api-log-log-info"></a>

<a id="eprosima-log-info"></a>

# EPROSIMA_LOG_INFO

### EPROSIMA_LOG_INFO(cat, msg) EPROSIMA_LOG_INFO_IMPL_(cat, msg)

Logs an info message. Disable it through Log::SetVerbosity, define LOG_NO_INFO, or being in a release branch. 

eProsima log layer. Logging categories and verbosity can be specified dynamically at runtime. However, even on a category not covered by the current verbosity level, there is some overhead on calling a log macro. For maximum performance, you can opt out of logging any particular level by defining the following symbols:

* define LOG_NO_ERROR
* define LOG_NO_WARNING
* define LOG_NO_INFO

Additionally. the lowest level (Info) is disabled by default on release branches.
