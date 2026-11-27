<a id="flowcontrollerdescriptor"></a>

# FlowControllerDescriptor

### struct FlowControllerDescriptor

Configuration values for creating flow controllers. 

This descriptor is used to define the configuration applied in the creation of a flow controller. 

**Since**
: 2.4.0 

### Public Members

### std::string name = [FASTDDS_FLOW_CONTROLLER_DEFAULT](#_CPPv4N8eprosima7fastdds4rtps31FASTDDS_FLOW_CONTROLLER_DEFAULTE)

Name of the flow controller. 

### [FlowControllerSchedulerPolicy](FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicyE) scheduler = [FlowControllerSchedulerPolicy](FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicyE)::[FIFO](FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy4FIFOE)

Scheduler policy used by the flow controller.

Default value: FlowControllerScheduler::FIFO_SCHEDULER 

### std::int32_t max_bytes_per_period = 0

Maximum number of bytes to be sent to network per period.

Range of bytes: [1, 2147483647]; 0 value means no limit. Default value: 0 

### std::uint64_t period_ms = 100

Period time in milliseconds.

Period of time on which the flow controller is allowed to send max_bytes_per_period. Default value: 100ms. 

### [ThreadSettings](../attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) sender_thread

Thread settings for the sender thread. 

### const char \*const eprosima::fastdds::rtps::FASTDDS_FLOW_CONTROLLER_DEFAULT

Name of the default flow controller. 

### const char \*const eprosima::fastdds::rtps::FASTDDS_STATISTICS_FLOW_CONTROLLER_DEFAULT

Name of the default flow controller for statistics writers.
