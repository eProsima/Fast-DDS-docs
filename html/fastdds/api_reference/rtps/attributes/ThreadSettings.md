<a id="threadsettings"></a>

# ThreadSettings

### struct ThreadSettings

Struct [ThreadSettings](#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) to specify various thread settings. This class is used to define attributes across a wide set of Qos and APIs. 

### Public Functions

### bool operator==(const [ThreadSettings](#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &rhs) const

Compare the left hand side (LHS) ThreadSetting with another one for equality.

* **Parameters:**
  **rhs** – The [ThreadSettings](#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) instance to compare with the LHS one. 

### bool operator!=(const [ThreadSettings](#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &rhs) const

Compare the left hand side (LHS) ThreadSetting with another one for inequality.

* **Parameters:**
  **rhs** – The [ThreadSettings](#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) instance to compare with the LHS one. 

### Public Members

### std::int32_t scheduling_policy = -1

The scheduling policy used for this thread. 

Configures the scheduling policy used for the thread. A value of -1 indicates system default.

This value is platform specific and it is used as-is to configure the specific platform thread. It is ignored on Windows platforms. Setting this value to something other than the default one may require different privileges on different platforms. 

### std::int32_t priority = std::numeric_limits<std::int32_t>::min()

The thread’s priority. 

Configures the thread’s priority. A value of -2^31 indicates system default.

This value is platform specific and it is used as-is to configure the specific platform thread. Setting this value to something other than the default one may require different privileges on different platforms. 

### std::uint64_t affinity = 0

The thread’s affinity. 

On some systems (Windows, Linux), this is a bit mask for setting the threads affinity to each core individually. On MacOS, this sets the affinity tag for the thread, and the OS tries to share the L2 cache between threads with the same affinity. A value of 0 indicates no particular affinity.

This value is platform specific and it is used as-is to configure the specific platform thread. Setting this value to something other than the default one may require different privileges on different platforms. 

### std::int32_t stack_size = -1

The thread’s stack size in bytes. 

Configures the thread’s stack size. A value of -1 indicates system default.

This value is platform specific and it is used as-is to configure the specific platform thread. Setting this value to something other than the default one may require different privileges on different platforms.
