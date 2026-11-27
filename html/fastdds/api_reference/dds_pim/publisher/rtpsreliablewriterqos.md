<a id="api-pim-rtpsreliablewriterqos"></a>

<a id="rtpsreliablewriterqos"></a>

# RTPSReliableWriterQos

### class RTPSReliableWriterQos

Qos Policy to configure the DisablePositiveACKsQos and the writer timing attributes. 

### Public Functions

### inline RTPSReliableWriterQos()

Constructor. 

### virtual ~RTPSReliableWriterQos() = default

Destructor. 

### Public Members

### fastdds::rtps::[WriterTimes](../../rtps/attributes/WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimesE) times

Writer Timing Attributes. 

### [DisablePositiveACKsQosPolicy](../core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicyE) disable_positive_acks

Disable positive acks QoS, implemented in the library. 

### bool disable_heartbeat_piggyback = false

Disable heartbeat piggyback mechanism.
