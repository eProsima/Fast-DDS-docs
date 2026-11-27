<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domainparticipantlistener"></a>

<a id="domainparticipantlistener"></a>

# DomainParticipantListener

[`DomainParticipantListener`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) is an abstract class defining the callbacks that will be triggered
in response to state changes on the [DomainParticipant](../domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

DomainParticipantListener inherits from [TopicListener](../../topic/topicListener/topicListener.md#dds-layer-topic-topiclistener),
[PublisherListener](../../publisher/publisherListener/publisherListener.md#dds-layer-publisher-publisherlistener), and [SubscriberListener](../../subscriber/subscriberListener/subscriberListener.md#dds-layer-subscriber-subscriberlistener).
Therefore, it has the ability to react to every kind of event that is
reported to any of its attached Entities.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that DomainParticipantListener inherits from other Listeners will only be called
if no other Entity was able to handle the event,
either because it has no Listener attached,
or because the callback is disabled by the [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) on the Entity.

Additionally, DomainParticipantListener adds the following non-standard callbacks:

* [`on_participant_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_participant_discoveryEP17DomainParticipantN7fastdds4rtps26ParticipantDiscoveryStatusERK27ParticipantBuiltinTopicDataRb): A new DomainParticipant is discovered in the same domain,
  a previously known DomainParticipant has been removed, or some DomainParticipant
  has changed its QoS.
  This method provides an overload with an additional boolean output parameter so a discovery callback can tell the
  middleware if a newly discovered participant has to be ignored via the use of the
  [`ignore_participant()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant18ignore_participantERK16InstanceHandle_t).
  This overload should be used when there is a need to ignore participants inside the discovery callback, since calling
  [`ignore_participant()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant18ignore_participantERK16InstanceHandle_t) inside the listener might deadlock.
  If both callbacks are implemented, the discovery callback with the `should_be_ignored` boolean flag takes
  precedence.
  The second discovery callback is only executed if the discovered DomainParticipant is not ignored in the first
  callback (`should_be_ignored` parameter returns `false`).
* [`on_data_reader_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_reader_discoveryEP17DomainParticipantN4rtps21ReaderDiscoveryStatusERK28SubscriptionBuiltinTopicDataRb): A new [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) is discovered
  in the same domain, a previously known DataReader has been removed, or some DataReader has changed its QoS.
* [`on_data_writer_discovery()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_writer_discoveryEP17DomainParticipantN4rtps21WriterDiscoveryStatusERK27PublicationBuiltinTopicDataRb): A new [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) is discovered
  in the same domain, a previously known DataWriter has been removed, or some DataWriter has changed its QoS.
* [`onParticipantAuthentication()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener27onParticipantAuthenticationEP17DomainParticipantRRN4rtps29ParticipantAuthenticationInfoE): Informs about the result of the authentication process
  of a remote DomainParticipant (either on failure or success).
* [`should_endpoints_match()`](../../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener22should_endpoints_matchEPK17DomainParticipantRK28SubscriptionBuiltinTopicDataRK27PublicationBuiltinTopicData): Called after standard QoS matching rules have been
  evaluated for a pair of endpoints (a [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and a
  [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) on the same topic with compatible QoS).
  It allows the application to apply custom matching logic as an additional filter on top of the standard rules.
  The callback receives the [`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) calling the listener, a
  `SubscriptionBuiltinTopicData` with the reader information, and a `PublicationBuiltinTopicData` with the
  writer information.
  Returning `true` allows the match to proceed; returning `false` prevents it.
  The default implementation always returns `true`.

#### IMPORTANT
For more information about callbacks and its hierarchy, please refer to
[Listener](../../core/entity/entity.md#dds-layer-core-entity-commonchars-listener).

```c++
class CustomDomainParticipantListener : public DomainParticipantListener
{

public:

    CustomDomainParticipantListener()
        : DomainParticipantListener()
    {
    }

    virtual ~CustomDomainParticipantListener()
    {
    }

    void on_participant_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ParticipantDiscoveryStatus status,
            const ParticipantBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        if (status == eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DISCOVERED_PARTICIPANT)
        {
            std::cout << "New participant discovered" << std::endl;
            // The following line can be modified to evaluate whether the discovered participant should be ignored
            // (usually based on fields present in the discovery information)
            bool ignoring_condition = false;
            if (ignoring_condition)
            {
                should_be_ignored = true; // Request the ignoring of the discovered participant
            }
        }
        else if (status == eprosima::fastdds::rtps::ParticipantDiscoveryStatus::REMOVED_PARTICIPANT ||
                status == eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DROPPED_PARTICIPANT)
        {
            std::cout << "Participant lost" << std::endl;
        }
    }

#if HAVE_SECURITY
    void onParticipantAuthentication(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ParticipantAuthenticationInfo&& info) override
    {
        if (info.status == eprosima::fastdds::rtps::ParticipantAuthenticationInfo::AUTHORIZED_PARTICIPANT)
        {
            std::cout << "A participant was authorized" << std::endl;
        }
        else if (info.status == eprosima::fastdds::rtps::ParticipantAuthenticationInfo::UNAUTHORIZED_PARTICIPANT)
        {
            std::cout << "A participant failed authorization" << std::endl;
        }
    }

#endif // if HAVE_SECURITY

    void on_data_reader_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::rtps::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER)
        {
            std::cout << "New datareader discovered" << std::endl;
            // The following line can be modified to evaluate whether the discovered datareader should be ignored
            // (usually based on fields present in the discovery information)
            bool ignoring_condition = false;
            if (ignoring_condition)
            {
                should_be_ignored = true; // Request the ignoring of the discovered datareader
            }
        }
        else if (reason == eprosima::fastdds::rtps::ReaderDiscoveryStatus::REMOVED_READER)
        {
            std::cout << "Datareader lost" << std::endl;
        }
    }

    void on_data_writer_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::WriterDiscoveryStatus reason,
            const eprosima::fastdds::dds::PublicationBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        static_cast<void>(participant);
        static_cast<void>(info);

        should_be_ignored = false;
        if (reason == eprosima::fastdds::rtps::WriterDiscoveryStatus::DISCOVERED_WRITER)
        {
            std::cout << "New datawriter discovered" << std::endl;
            // The following line can be modified to evaluate whether the discovered datawriter should be ignored
            // (usually based on fields present in the discovery information)
            bool ignoring_condition = false;
            if (ignoring_condition)
            {
                should_be_ignored = true; // Request the ignoring of the discovered datawriter
            }
        }
        else if (reason == eprosima::fastdds::rtps::WriterDiscoveryStatus::REMOVED_WRITER)
        {
            std::cout << "Datawriter lost" << std::endl;
        }
    }

    bool should_endpoints_match(
            const DomainParticipant* participant,
            const eprosima::fastdds::rtps::SubscriptionBuiltinTopicData& reader_info,
            const eprosima::fastdds::dds::PublicationBuiltinTopicData& writer_info) override
    {
        static_cast<void>(participant);
        // For example, only allow matching between endpoints belonging to different participants.
        // This prevents intra-participant communication while still allowing
        // cross-participant communication for endpoints that pass standard QoS checks.
        return reader_info.participant_guid.guidPrefix != writer_info.participant_guid.guidPrefix;
    }

};
```
