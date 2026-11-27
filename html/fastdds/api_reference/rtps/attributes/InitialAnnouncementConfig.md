<a id="initialannouncementconfig"></a>

# InitialAnnouncementConfig

### struct InitialAnnouncementConfig

Struct [InitialAnnouncementConfig](#structeprosima_1_1fastdds_1_1rtps_1_1InitialAnnouncementConfig) defines the behavior of the [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) initial announcements. 

### Public Members

### std::uint32_t count = 5u

Number of initial announcements with specific period (default 5) 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) period = {0, 100000000u}

Specific period for initial announcements (default 100ms)
