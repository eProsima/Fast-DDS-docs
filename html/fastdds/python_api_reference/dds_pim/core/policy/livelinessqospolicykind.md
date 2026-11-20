<a id="python-api-pim-livelinessqospolicykind"></a>

<a id="livelinessqospolicykind"></a>

# LivelinessQosPolicyKind

### fastdds.AUTOMATIC_LIVELINESS_QOS

The infrastructure will automatically signal liveliness for the DataWriters at least as often as required by the lease_duration.

### fastdds.MANUAL_BY_PARTICIPANT_LIVELINESS_QOS

The Service will assume that as long as at least one Entity within the DomainParticipant has asserted its liveliness the other
Entities in that same DomainParticipant are also alive.

### fastdds.MANUAL_BY_TOPIC_LIVELINESS_QOS

The Service will only assume liveliness of the DataWriter if the application has asserted liveliness of that DataWriter itself.
