<a id="api-pim-deadlinemissedstatus"></a>

<a id="deadlinemissedstatus"></a>

# DeadlineMissedStatus

### struct DeadlineMissedStatus

A struct storing the deadline status. 

### Public Functions

### inline DeadlineMissedStatus()

Constructor. 

### inline ~DeadlineMissedStatus()

Destructor. 

### Public Members

### std::uint32_t total_count

Total cumulative number of offered deadline periods elapsed during which a writer failed to provide data. 

Missed deadlines accumulate, that is, each deadline period the total_count will be incremented by 1 

### std::uint32_t total_count_change

The change in total_count since the last time the listener was called or the status was read. 

### InstanceHandle_t last_instance_handle

Handle to the last instance missing the deadline.
