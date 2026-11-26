<a id="writerattributes"></a>

# WriterAttributes

### class WriterAttributes

Class [WriterAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1WriterAttributes), defining the attributes of a [RTPSWriter](../writer/RTPSWriter.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSWriter). 

### Public Members

### [EndpointAttributes](EndpointAttributes.md#_CPPv4N8eprosima7fastdds4rtps18EndpointAttributesE) endpoint

Attributes of the associated endpoint. 

### [WriterTimes](WriterTimes.md#_CPPv4N8eprosima7fastdds4rtps11WriterTimesE) times

Writer Times (only used for RELIABLE). 

### fastdds::dds::[LivelinessQosPolicyKind](../../dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKindE) liveliness_kind

Liveliness kind. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) liveliness_lease_duration

Liveliness lease duration. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) liveliness_announcement_period

Liveliness announcement period. 

### [RTPSWriterPublishMode](RTPSWriterPublishMode.md#_CPPv4N8eprosima7fastdds4rtps21RTPSWriterPublishModeE) mode

Indicates if the Writer is synchronous or asynchronous. 

### bool disable_heartbeat_piggyback

Disable the sending of heartbeat piggybacks. 

### ResourceLimitedContainerConfig matched_readers_allocation

Define the allocation behaviour for matched-reader-dependent collections. 

### bool disable_positive_acks

Disable the sending of positive ACKs. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) keep_duration

Keep duration to keep a sample before considering it has been acked. 

### std::string flow_controller_name = fastdds::rtps::[FASTDDS_FLOW_CONTROLLER_DEFAULT](../flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps31FASTDDS_FLOW_CONTROLLER_DEFAULTE)

Flow controller name. Default: [fastdds::rtps::FASTDDS_FLOW_CONTROLLER_DEFAULT](../flowcontrol/FlowControllerDescriptor.md#namespaceeprosima_1_1fastdds_1_1rtps_1af19b59d6b6e6b830a5c9753e81ff44f7). 

### bool separate_sending = false

Whether to send data to each matched reader separately. 

### std::int32_t transport_priority = 0

Transport priority for this Writer.
