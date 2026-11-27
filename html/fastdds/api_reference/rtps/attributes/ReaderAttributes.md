<a id="readerattributes"></a>

# ReaderAttributes

### class ReaderAttributes

Class [ReaderAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1ReaderAttributes), to define the attributes of a [RTPSReader](../reader/RTPSReader.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSReader). 

### Public Members

### [EndpointAttributes](EndpointAttributes.md#_CPPv4N8eprosima7fastdds4rtps18EndpointAttributesE) endpoint = {}

Attributes of the associated endpoint. 

### [ReaderTimes](ReaderTimes.md#_CPPv4N8eprosima7fastdds4rtps11ReaderTimesE) times = {}

Times associated with this reader (only for stateful readers) 

### fastdds::dds::[LivelinessQosPolicyKind](../../dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKindE) liveliness_kind = fastdds::dds::[LivelinessQosPolicyKind](../../dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKindE)::[AUTOMATIC_LIVELINESS_QOS](../../dds_pim/core/policy/livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKind24AUTOMATIC_LIVELINESS_QOSE)

Liveliness kind. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) liveliness_lease_duration = {TIME_T_INFINITE_SECONDS, TIME_T_INFINITE_NANOSECONDS}

Liveliness lease duration. 

### bool expects_inline_qos = false

Indicates if the reader expects Inline qos, default value false. 

### bool disable_positive_acks = false

Disable positive ACKs. 

### bool accept_messages_from_unkown_writers = false

Enable or disable the reception of messages from unknown writers. 

### ResourceLimitedContainerConfig matched_writers_allocation = {}

Define the allocation behaviour for matched-writer-dependent collections. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) data_sharing_listener_thread = {}

Thread settings for the data-sharing listener thread.
