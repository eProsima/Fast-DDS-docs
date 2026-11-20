<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="domainparticipant-discovery-callbacks"></a>

<a id="domainparticipantlistener-discovery-callbacks"></a>

# DomainParticipantListener Discovery Callbacks

As stated in [DomainParticipantListener](../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener), the [`DomainParticipantListener`](../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) is an abstract class
defining the callbacks that will be triggered in response to state changes on the DomainParticipant.
Fast DDS defines three callbacks attached to events that may occur during discovery:
[`on_participant_discovery()`](../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_participant_discoveryEP17DomainParticipantN7fastdds4rtps26ParticipantDiscoveryStatusERK27ParticipantBuiltinTopicDataRb),
[`on_data_reader_discovery()`](../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_reader_discoveryEP17DomainParticipantN4rtps21ReaderDiscoveryStatusERK28SubscriptionBuiltinTopicDataRb),
[`on_data_writer_discovery()`](../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_data_writer_discoveryEP17DomainParticipantN4rtps21WriterDiscoveryStatusERK27PublicationBuiltinTopicDataRb).
Further information about the DomainParticipantListener is provided in the [DomainParticipantListener](../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener)
section.
The following is an example of the implementation of DomainParticipantListener discovery callbacks.

```c++
class DiscoveryDomainParticipantListener : public DomainParticipantListener
{
    /* Custom Callback on_participant_discovery */
    void on_participant_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ParticipantDiscoveryStatus status,
            const ParticipantBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        static_cast<void>(participant);
        switch (status){
            case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DISCOVERED_PARTICIPANT:
            {
                /* Process the case when a new DomainParticipant was found in the domain */
                std::cout << "New DomainParticipant '" << info.participant_name
                          << "' with ID '" << info.guid.entityId << "' and GuidPrefix '"
                          << info.guid.guidPrefix << "' discovered." << std::endl;
                /* The following line can be substituted to evaluate whether the discovered participant should be ignored */
                bool ignoring_condition = false;
                if (ignoring_condition)
                {
                    should_be_ignored = true;     // Request the ignoring of the discovered participant
                }
            }
            break;
            case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::CHANGED_QOS_PARTICIPANT:
                /* Process the case when a DomainParticipant changed its QOS */
                break;
            case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::REMOVED_PARTICIPANT:
                /* Process the case when a DomainParticipant was removed from the domain */
                std::cout << "DomainParticipant '" << info.participant_name
                          << "' with ID '" << info.guid.entityId << "' and GuidPrefix '"
                          << info.guid.guidPrefix << "' left the domain." << std::endl;
                break;
        }
    }

    /* Custom Callback on_data_reader_discovery */
    void on_data_reader_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::rtps::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        static_cast<void>(participant);
        switch (reason){
            case eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER:
            {
                /* Process the case when a new datareader was found in the domain */
                std::cout << "New DataReader subscribed to topic '" << info.topic_name
                          << "' of type '" << info.type_name << "' discovered";
                /* The following line can be substituted to evaluate whether the discovered datareader should be ignored */
                bool ignoring_condition = false;
                if (ignoring_condition)
                {
                    should_be_ignored = true;     // Request the ignoring of the discovered datareader
                }
            }
            break;
            case eprosima::fastdds::rtps::ReaderDiscoveryStatus::CHANGED_QOS_READER:
                /* Process the case when a datareader changed its QOS */
                break;
            case eprosima::fastdds::rtps::ReaderDiscoveryStatus::REMOVED_READER:
                /* Process the case when a datareader was removed from the domain */
                std::cout << "DataReader subscribed to topic '" << info.topic_name
                          << "' of type '" << info.type_name << "' left the domain.";
                break;
        }
    }

    /* Custom Callback on_data_writer_discovery */
    void on_data_writer_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::WriterDiscoveryStatus reason,
            const eprosima::fastdds::dds::PublicationBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        static_cast<void>(participant);
        switch (reason){
            case eprosima::fastdds::rtps::WriterDiscoveryStatus::DISCOVERED_WRITER:
            {
                /* Process the case when a new datawriter was found in the domain */
                std::cout << "New DataWriter publishing under topic '" << info.topic_name
                          << "' of type '" << info.type_name << "' discovered";
                /* The following line can be substituted to evaluate whether the discovered datawriter should be ignored */
                bool ignoring_condition = false;
                if (ignoring_condition)
                {
                    should_be_ignored = true;     // Request the ignoring of the discovered datawriter
                }
            }
            break;
            case eprosima::fastdds::rtps::WriterDiscoveryStatus::CHANGED_QOS_WRITER:
                /* Process the case when a datawriter changed its QOS */
                break;
            case eprosima::fastdds::rtps::WriterDiscoveryStatus::REMOVED_WRITER:
                /* Process the case when a datawriter was removed from the domain */
                std::cout << "DataWriter publishing under topic '" << info.topic_name
                          << "' of type '" << info.type_name << "' left the domain.";
                break;
        }
    }

};
```

To use the previously implemented discovery callbacks in `DiscoveryDomainParticipantListener` class, which
inherits from the DomainParticipantListener, an object of this class is created and registered as a listener
of the DomainParticipant.

```c++
// Create the participant QoS and configure values
DomainParticipantQos pqos;

// Create a custom user DomainParticipantListener
DiscoveryDomainParticipantListener* plistener = new DiscoveryDomainParticipantListener();
// Pass the listener on DomainParticipant creation.
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(
    0, pqos, plistener);
```

#### IMPORTANT
Read more about callbacks and its hierarchy [here](../dds_layer/core/entity/entity.md#dds-layer-core-entity-commonchars-listener)
