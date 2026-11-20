<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-log-disable"></a>

<a id="disable-logging-module"></a>

# Disable Logging Module

Setting the [Verbosity Level](logging_configuration.md#dds-layer-log-verbosity-level), translates into entries not being added to the log queue if the
entry’s level has lower importance than the set one.
This check is performed when calling the macros defined in [Logging Messages](logging_messages.md#dds-layer-log-logging).
However, it is possible to fully disable each macro (and therefore each verbosity level individually) at build time.

* [`EPROSIMA_LOG_INFO`](../api_reference/log/loginfo.md#c.EPROSIMA_LOG_INFO) is fully disabled by either:
  * Setting CMake option `LOG_NO_INFO` to `ON` (default for Single-Config generators if `CMAKE_BUILD_TYPE`
    is other than `Debug`).
  * Defining macro `HAVE_LOG_NO_INFO` to `1`.
* [`EPROSIMA_LOG_WARNING`](../api_reference/log/logwarning.md#c.EPROSIMA_LOG_WARNING) is fully disabled by either:
  * Setting CMake option `LOG_NO_WARNING` to `ON`.
  * Defining macro `HAVE_LOG_NO_WARNING` to `1`.
* [`EPROSIMA_LOG_ERROR`](../api_reference/log/logerror.md#c.EPROSIMA_LOG_ERROR) is fully disabled by either:
  * Setting CMake option `LOG_NO_ERROR` to `ON`.
  * Defining macro `HAVE_LOG_NO_ERROR` to `1`.

Applying either of the previously described methods will set the macro to be empty at configuration time, thus allowing
the compiler to optimize the call out.
This is done so that all the debugging messages present on the library are optimized out at build time if not building
for debugging purposes, thus preventing them to impact performance.

`INTERNAL_DEBUG` CMake option activates log macros compilation, so the arguments of the macros are compiled.
However:

* It does not activate the log Warning and Error messages, i.e. the messages are not written in the log queue.
* [`EPROSIMA_LOG_INFO`](../api_reference/log/loginfo.md#c.EPROSIMA_LOG_INFO) has a special behaviour to simplify working with Multi-Config capability IDEs.
  If CMake option `LOG_NO_INFO` is `OFF`, or the C++ definition `HAVE_LOG_NO_INFO` is `0`, then logging is enabled
  only for `Debug` configuration.
  In this scenario, setting `FASTDDS_ENFORCE_LOG_INFO` to `ON` will enable [`EPROSIMA_LOG_INFO`](../api_reference/log/loginfo.md#c.EPROSIMA_LOG_INFO) even on non `Debug`
  configurations.
  This is specially useful when using the *Fast DDS*’ logging module in an external application which links with
  *Fast DDS* compiled in `Release`.
  In that case, applications wanting to use all three levels of logging can simply add the following code prior to
  including any Fast DDS header:
  ```c
  #define HAVE_LOG_NO_INFO 0
  #define FASTDDS_ENFORCE_LOG_INFO 1
  ```

#### WARNING
`INTERNAL_DEBUG` can be automatically set to `ON` if CMake option `EPROSIMA_BUILD` is set to `ON`.

<a id="old-log-macros-disable"></a>

<a id="id1"></a>

# Old Log macros disable

Before version 2.8.2, Fast DDS project used log macros: `logInfo`, `logWarning` and
`logError`, which may collide with other libraries.
These log macros have been replaced by new ones with a more specific format: (e.g. `EPROSIMA_LOG_INFO`).
In order to disable old macros compilation, use CMake option `ENABLE_OLD_LOG_MACROS = ON`
or define `ENABLE_OLD_LOG_MACROS_ 0` before including the log module
`#include <fastdds/dds/log/Log.hpp>`.

#### WARNING
These macros will be deprecated in future versions of Fast DDS.
The use of the new format ones is encouraged.
