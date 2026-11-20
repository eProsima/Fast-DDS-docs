<a id="flowcontrollerschedulerpolicy"></a>

# FlowControllerSchedulerPolicy

### enum class eprosima::fastdds::rtps::FlowControllerSchedulerPolicy : std::int32_t

Supported scheduler policy by a flow controller. 

A flow controller’s scheduler policy takes the decision of which samples are the next ones to be sent to the network. Fast DDS flow controller supports several scheduler policies listed in this enumeration. 

*Values:*

### enumerator FIFO

FIFO scheduler policy: first written sample by user, first sample scheduled to be sent to network. 

### enumerator ROUND_ROBIN

Round Robin scheduler policy: schedules one sample of each DataWriter in circular order. 

### enumerator HIGH_PRIORITY

High priority scheduler policy: samples with highest priority are scheduled first to be sent to network. 

### enumerator PRIORITY_WITH_RESERVATION

Priority with reservation scheduler policy: guarantee each DataWriter’s minimum reservation of throughput. Samples not fitting the reservation are scheduled by priority. 

### enumerator RETRANSMISSION_RESERVATION

 **[Pro]**  Retransmission reservation scheduler policy: guarantees a minimum bandwidth share to retransmissions
