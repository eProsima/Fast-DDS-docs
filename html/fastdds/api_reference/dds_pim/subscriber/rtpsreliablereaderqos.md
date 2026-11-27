<a id="api-pim-rtpsreliablereaderqos"></a>

<a id="rtpsreliablereaderqos"></a>

# RTPSReliableReaderQos

### class RTPSReliableReaderQos

Qos Policy to configure the DisablePositiveACKsQos and the reader attributes. 

### Public Functions

### inline RTPSReliableReaderQos()

Constructor. 

### virtual ~RTPSReliableReaderQos() = default

Destructor. 

### Public Members

### fastdds::rtps::[ReaderTimes](../../rtps/attributes/ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimesE) times

Times associated with the Reliable Readers events. 

### [DisablePositiveACKsQosPolicy](../core/policy/disablepositiveacksqospolicy.md#_CPPv4N8eprosima7fastdds3dds28DisablePositiveACKsQosPolicyE) disable_positive_acks

Control the sending of positive ACKs.
