<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-thread"></a>

<a id="logging-thread"></a>

# Logging Thread

Calls to the macros presented in [Logging Messages](logging_messages.md#dds-layer-log-logging) merely add the log entry to a ready-to-consume queue.
Upon creation, the logging module spawns a thread that awakes every time an entry is added to the queue.
When awaken, this thread feeds all the entries in the queue to all the registered [Consumers](consumer.md#dds-layer-log-consumer).
Once the work is done, the thread falls back into idle state.
This strategy prevents the module from blocking the application thread when a logging operation is performed.
However, sometimes applications may want to wait until the logging routine is done to continue their operation.
The logging module provides this capability via the member function [`Log::Flush()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log5FlushEv).
Furthermore, it is possible to completely eliminate the thread and its resources using member function
[`Log::KillThread()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log10KillThreadEv).
In addition, it is possible to configure certain settings of this logging thread via the member function
[`Log::SetThreadConfig()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log15SetThreadConfigERKN4rtps14ThreadSettingsE).

```c++
// Block current thread until the log queue is empty.
Log::Flush();

// Stop the loggin thread and free its resources.
Log::KillThread();

// Configure ThreadSettings for the logging thread
Log::SetThreadConfig(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
```

#### WARNING
A call to any of the macros present in [Logging Messages](logging_messages.md#dds-layer-log-logging) will spawn the logging thread even if it has
been previously killed with [`Log::KillThread()`](../api_reference/log/log_class.md#_CPPv4N8eprosima7fastdds3dds3Log10KillThreadEv).
