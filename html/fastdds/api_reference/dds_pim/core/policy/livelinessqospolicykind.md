<a id="api-pim-livelinessqospolicykind"></a>

<a id="livelinessqospolicykind"></a>

# LivelinessQosPolicyKind

### enum eprosima::fastdds::dds::LivelinessQosPolicyKind

Enum LivelinessQosPolicyKind, different kinds of liveliness for [LivelinessQosPolicy](livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)

*Values:*

### enumerator AUTOMATIC_LIVELINESS_QOS

The infrastructure will automatically signal liveliness for the DataWriters at least as often as required by the lease_duration. 

### enumerator MANUAL_BY_PARTICIPANT_LIVELINESS_QOS

The Service will assume that as long as at least one [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) within the [DomainParticipant](../../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) has asserted its liveliness the other Entities in that same [DomainParticipant](../../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) are also alive. 

### enumerator MANUAL_BY_TOPIC_LIVELINESS_QOS

The Service will only assume liveliness of the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) if the application has asserted liveliness of that [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) itself.
