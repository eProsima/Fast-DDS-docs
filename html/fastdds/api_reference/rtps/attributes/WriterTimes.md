<a id="writertimes"></a>

# WriterTimes

### struct WriterTimes

Struct [WriterTimes](#structeprosima_1_1fastdds_1_1rtps_1_1WriterTimes), defining the times associated with the Reliable Writers events. 

### Public Members

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) initial_heartbeat_delay = {0, 12 \* 1000 \* 1000}

Initial heartbeat delay. Default value 12ms. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) heartbeat_period = {3, 0}

Periodic HB period, default value 3s. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) nack_response_delay = {0, 5 \* 1000 \* 1000}

Delay to apply to the response of a ACKNACK message, default value 5ms. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) nack_supression_duration = {0, 0}

This time allows the [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter) to ignore nack messages too soon after the data as sent, default value 0s.
