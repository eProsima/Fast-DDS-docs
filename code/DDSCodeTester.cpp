#include <iostream>
#include <memory>
#include <sstream>
#include <thread>

#include <fastcdr/Cdr.h>

#include <fastdds/dds/builtin/topic/ParticipantBuiltinTopicData.hpp>
#include <fastdds/dds/core/condition/GuardCondition.hpp>
#include <fastdds/dds/core/condition/WaitSet.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/log/FileConsumer.hpp>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/log/OStreamConsumer.hpp>
#include <fastdds/dds/log/StdoutConsumer.hpp>
#include <fastdds/dds/log/StdoutErrConsumer.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/PublisherListener.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/SubscriberListener.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicListener.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicData.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicDataFactory.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicPubSubType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilder.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilderFactory.hpp>
#include <fastdds/dds/xtypes/type_representation/TypeObject.hpp>
#include <fastdds/dds/xtypes/utils.hpp>
#include <fastdds/rtps/attributes/ThreadSettings.hpp>
#include <fastdds/rtps/common/WriteParams.hpp>
#include <fastdds/rtps/history/IPayloadPool.hpp>
#include <fastdds/rtps/reader/ReaderDiscoveryStatus.hpp>
#include <fastdds/rtps/transport/ChainingTransport.hpp>
#include <fastdds/rtps/transport/ChainingTransportDescriptor.hpp>
#include <fastdds/rtps/transport/network/AllowedNetworkInterface.hpp>
#include <fastdds/rtps/transport/network/BlockedNetworkInterface.hpp>
#include <fastdds/rtps/transport/network/NetmaskFilterKind.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>
#include <fastdds/rtps/transport/TCPTransportDescriptor.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/TCPv6TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.hpp>
#include <fastdds/rtps/transport/NetworkBuffer.hpp>
#include <fastdds/statistics/dds/domain/DomainParticipant.hpp>
#include <fastdds/statistics/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/statistics/topic_names.hpp>
#include <fastdds/utils/IPLocator.hpp>

using namespace eprosima::fastdds::dds;

class CustomChainingTransportDescriptor : public eprosima::fastdds::rtps::ChainingTransportDescriptor
{
public:

    CustomChainingTransportDescriptor(
            std::shared_ptr<eprosima::fastdds::rtps::TransportDescriptorInterface> low_level)
        : ChainingTransportDescriptor(low_level)
    {
    }

    eprosima::fastdds::rtps::TransportInterface* create_transport() const override;
};

class HelloWorld
{
public:

    void msg(
            const std::string&)
    {
    }

    std::string msg()
    {
        return "";
    }

};

//CHAINING_TRANSPORT_OVERRIDE
class CustomChainingTransport : public eprosima::fastdds::rtps::ChainingTransport
{

public:

    CustomChainingTransport(
            const CustomChainingTransportDescriptor& descriptor)
        : ChainingTransport(descriptor)
        , descriptor_(descriptor)
    {
    }

    eprosima::fastdds::rtps::TransportDescriptorInterface* get_configuration()
    {
        return &descriptor_;
    }

    bool send(
            eprosima::fastdds::rtps::SenderResource* low_sender_resource,
            const std::vector<eprosima::fastdds::rtps::NetworkBuffer>& buffers,
            uint32_t total_bytes,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_begin,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_end,
            const std::chrono::steady_clock::time_point& timeout) override
    {
        //
        // Preprocess outcoming buffer.
        //

        // Call low level transport
        return low_sender_resource->send(buffers, total_bytes, destination_locators_begin,
                       destination_locators_end, timeout);
    }

    void receive(
            eprosima::fastdds::rtps::TransportReceiverInterface* next_receiver,
            const eprosima::fastdds::rtps::octet* receive_buffer,
            uint32_t receive_buffer_size,
            const eprosima::fastdds::rtps::Locator_t& local_locator,
            const eprosima::fastdds::rtps::Locator_t& remote_locator) override
    {
        //
        // Preprocess incoming buffer.
        //

        // Call upper level
        next_receiver->OnDataReceived(receive_buffer, receive_buffer_size, local_locator, remote_locator);
    }

private:

    CustomChainingTransportDescriptor descriptor_;
};
//!--

eprosima::fastdds::rtps::TransportInterface* CustomChainingTransportDescriptor::create_transport() const
{
    return new CustomChainingTransport(*this);
}

//DDS_DOMAINPARTICIPANT_LISTENER_SPECIALIZATION
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

};
//!--

// Custom Payload pool example for documentation
class CustomPayloadPool : public eprosima::fastdds::rtps::IPayloadPool
{
public:

    CustomPayloadPool() = default;
    ~CustomPayloadPool() = default;
    bool get_payload(
            unsigned int size,
            eprosima::fastdds::rtps::SerializedPayload_t& payload)
    {
        return true;
    }

    bool get_payload(
            const eprosima::fastdds::rtps::SerializedPayload_t& data,
            eprosima::fastdds::rtps::SerializedPayload_t& payload)
    {
        return true;
    }

    bool release_payload(
            eprosima::fastdds::rtps::SerializedPayload_t& payload)
    {
        return true;
    }

};
//!--

void dds_domain_examples()
{
    {
        //DDS_LOAD_XML_PROFILE
        // Load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Profiles can now be used to create Entities
        DomainParticipant* participant_with_profile =
                DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile");
        if (nullptr == participant_with_profile)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_DOMAINPARTICIPANT
        // Create a DomainParticipant with default DomainParticipantQos and no Listener
        // The value PARTICIPANT_QOS_DEFAULT is used to denote the default QoS.
        DomainParticipant* participant_with_default_attributes =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant_with_default_attributes)
        {
            // Error
            return;
        }

        // A custom DomainParticipantQos can be provided to the creation method
        DomainParticipantQos custom_qos;

        // Modify QoS attributes
        // (...)

        DomainParticipant* participant_with_custom_qos =
                DomainParticipantFactory::get_instance()->create_participant(0, custom_qos);
        if (nullptr == participant_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with default QoS and a custom Listener.
        // CustomDomainParticipantListener inherits from DomainParticipantListener.
        // The value PARTICIPANT_QOS_DEFAULT is used to denote the default QoS.
        CustomDomainParticipantListener custom_listener;
        DomainParticipant* participant_with_default_qos_and_custom_listener =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT,
                        &custom_listener);
        if (nullptr == participant_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_DOMAINPARTICIPANT
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DomainParticipant using a profile and no Listener
        DomainParticipant* participant_with_profile =
                DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile");
        if (nullptr == participant_with_profile)
        {
            // Error
            return;
        }

        // Create a DomainParticipant using a profile and a custom Listener.
        // CustomDomainParticipantListener inherits from DomainParticipantListener.
        CustomDomainParticipantListener custom_listener;
        DomainParticipant* participant_with_profile_and_custom_listener =
                DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile",
                        &custom_listener);
        if (nullptr == participant_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_DOMAINPARTICIPANT_DEFAULT_PROFILE
        // Create a DomainParticipant using the environment profile and no Listener
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant_with_default_profile();
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a DomainParticipant using the environment profile and a custom Listener.
        // CustomDomainParticipantListener inherits from DomainParticipantListener.
        CustomDomainParticipantListener custom_listener;
        DomainParticipant* participant_with_custom_listener =
                DomainParticipantFactory::get_instance()->create_participant_with_default_profile(
            &custom_listener, StatusMask::none());
        if (nullptr == participant_with_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTQOS
        // Create a DomainParticipant with default DomainParticipantQos
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DomainParticipantQos qos = participant->get_qos();

        // Modify QoS attributes
        qos.entity_factory().autoenable_created_entities = false;

        // Assign the new Qos to the object
        participant->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTEXTENDEDQOS
        // Create a DomainParticipant with DomainParticipantExtendedQos from profile
        DomainParticipantExtendedQos profile_extended_qos;
        DomainParticipantFactory::get_instance()->get_participant_extended_qos_from_profile("participant_profile",
                profile_extended_qos);

        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(profile_extended_qos);
        if (nullptr == participant)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
        // Create a custom DomainParticipantQos
        DomainParticipantQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a DomainParticipant with a custom DomainParticipantQos

        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, custom_qos);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Set the QoS on the participant to the default
        if (participant->set_qos(PARTICIPANT_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (participant->set_qos(DomainParticipantFactory::get_instance()->get_default_participant_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_DOMAINPARTICIPANT
        // Create a DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Use the DomainParticipant to communicate
        // (...)

        // Delete entities created by the DomainParticipant
        if (participant->delete_contained_entities() != RETCODE_OK)
        {
            // DomainParticipant failed to delete the entities it created.
            return;
        }

        // Delete the DomainParticipant
        if (DomainParticipantFactory::get_instance()->delete_participant(participant) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
        // Get the current QoS or create a new one from scratch
        DomainParticipantQos qos_type1 = DomainParticipantFactory::get_instance()->get_default_participant_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if (DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type1) !=
                RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new default DomainParticipantQos.
        DomainParticipant* participant_with_qos_type1 =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DomainParticipantQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if (DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type2) !=
                RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        DomainParticipant* participant_with_qos_type2 =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default DomainParticipantQos to the original default constructed values
        if (DomainParticipantFactory::get_instance()->set_default_participant_qos(PARTICIPANT_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (DomainParticipantFactory::get_instance()->set_default_participant_qos(DomainParticipantQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTFACTORYQOS
        DomainParticipantFactoryQos qos;

        // Setting autoenable_created_entities to true makes the created DomainParticipants
        // to be enabled upon creation
        qos.entity_factory().autoenable_created_entities = true;
        if (DomainParticipantFactory::get_instance()->set_qos(qos) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new DomainParticipantFactoryQos.
        // The returned DomainParticipant is already enabled
        DomainParticipant* enabled_participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == enabled_participant)
        {
            // Error
            return;
        }

        // Setting autoenable_created_entities to false makes the created DomainParticipants
        // to be disabled upon creation
        qos.entity_factory().autoenable_created_entities = false;
        if (DomainParticipantFactory::get_instance()->set_qos(qos) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new DomainParticipantFactoryQos.
        // The returned DomainParticipant is disabled and will need to be enabled explicitly
        DomainParticipant* disabled_participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == disabled_participant)
        {
            // Error
            return;
        }
        //!--
    }
    {
        // DDS_SECURITY_AUTH_PLUGIN
        DomainParticipantQos pqos;

        // Activate DDS:Auth:PKI-DH plugin
        pqos.properties().properties().emplace_back("dds.sec.auth.plugin",
                "builtin.PKI-DH");

        // Configure DDS:Auth:PKI-DH plugin
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.identity_ca",
            "file://maincacert.pem");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.identity_certificate",
            "file://partcert.pem");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.identity_crl",
            "file://crl.pem");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.private_key",
            "file://partkey.pem");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.password",
            "domainParticipantPassword");
        //!--
    }
    {
        // DDS_SECURITY_AUTH_HANDSHAKE_PROPS
        DomainParticipantQos pqos;

        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.max_handshake_requests",
            "5");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.initial_handshake_resend_period",
            "250");
        pqos.properties().properties().emplace_back(
            "dds.sec.auth.builtin.PKI-DH.handshake_resend_period_gain",
            "1.5");
        //!--
    }
    {
        // DDS_SECURITY_ACCESS_CONTROL_PLUGIN
        DomainParticipantQos pqos;

        // Activate DDS:Access:Permissions plugin
        pqos.properties().properties().emplace_back("dds.sec.access.plugin",
                "builtin.Access-Permissions");

        // Configure DDS:Access:Permissions plugin
        pqos.properties().properties().emplace_back(
            "dds.sec.access.builtin.Access-Permissions.permissions_ca",
            "file://certs/maincacert.pem");
        pqos.properties().properties().emplace_back(
            "dds.sec.access.builtin.Access-Permissions.governance",
            "file://certs/governance.smime");
        pqos.properties().properties().emplace_back(
            "dds.sec.access.builtin.Access-Permissions.permissions",
            "file://certs/permissions.smime");
        //!--
    }
    {
        // DDS_SECURITY_CRYPTO_PLUGIN_DOMAINPARTICIPANT
        DomainParticipantQos pqos;

        // Activate DDS:Crypto:AES-GCM-GMAC plugin
        pqos.properties().properties().emplace_back("dds.sec.crypto.plugin",
                "builtin.AES-GCM-GMAC");
        //!--
    }
    {
        // DDS_SECURITY_LOGGING_PLUGIN
        DomainParticipantQos pqos;

        // Activate DDS:Logging:DDS_LogTopic plugin
        pqos.properties().properties().emplace_back("dds.sec.log.plugin",
                "builtin.DDS_LogTopic");

        // Configure DDS:Logging:DDS_LogTopic plugin
        pqos.properties().properties().emplace_back(
            "dds.sec.log.builtin.DDS_LogTopic.logging_level",
            "EMERGENCY_LEVEL");
        pqos.properties().properties().emplace_back(
            "dds.sec.log.builtin.DDS_LogTopic.log_file",
            "myLogFile.log");
        //!--
    }
    {
        // FASTDDS_STATISTICS_MODULE
        DomainParticipantQos pqos;

        // Activate Fast DDS Statistics module
        pqos.properties().properties().emplace_back("fastdds.statistics",
                "HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC");
        //!--
    }
    {
        // FASTDDS_MONITOR_SERVICE_PROPERTY
        DomainParticipantQos pqos;

        // Enable Fast DDS Monitor Service through properties
        pqos.properties().properties().emplace_back("fastdds.enable_monitor_service",
                "true");

        // Enable Fast DDS Monitor Service through statistics properties (other way)
        pqos.properties().properties().emplace_back("fastdds.statistics",
                "MONITOR_SERVICE_TOPIC");

        DomainParticipant* participant_with_mon_srv = DomainParticipantFactory::get_instance()->create_participant(0,
                        pqos);

        //!--
    }
    {
        // FASTDDS_MONITOR_SERVICE_API

        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0,
                        PARTICIPANT_QOS_DEFAULT);

        // Obtain pointer to child class
        eprosima::fastdds::statistics::dds::DomainParticipant* statistics_participant =
                eprosima::fastdds::statistics::dds::DomainParticipant::narrow(participant);


        // Enable Fast DDS Monitor Service through API
        statistics_participant->enable_monitor_service();

        // Disable Fast DDS Monitor Service through API
        statistics_participant->disable_monitor_service();


        //!--
    }
    {
        // FASTDDS_PHYSICAL_PROPERTIES
        /* Create participant which announces default physical properties */
        DomainParticipantQos pqos_default_physical;
        // NOTE: If FASTDDS_STATISTICS is defined, then setting the properties to "" is not necessary
        pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.host", "");
        pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.user", "");
        pqos_default_physical.properties().properties().emplace_back("fastdds.physical_data.process", "");
        DomainParticipant* participant_with_physical = DomainParticipantFactory::get_instance()->create_participant(0,
                        pqos_default_physical);

        /* Create participant which announces custom physical properties */
        DomainParticipantQos pqos_custom_physical;
        // NOTE: If FASTDDS_STATISTICS is defined, then clear the properties before setting them
        // pqos_custom_physical.properties().properties().clear()
        pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.host", "custom_hostname");
        pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.user", "custom_username");
        pqos_custom_physical.properties().properties().emplace_back("fastdds.physical_data.process", "custom_process");
        DomainParticipant* participant_custom_physical = DomainParticipantFactory::get_instance()->create_participant(0,
                        pqos_custom_physical);

        /* Create participant which does not announce physical properties */
        DomainParticipantQos pqos_no_physical;
        pqos_no_physical.properties().properties().clear();
        DomainParticipant* participant_without_physical = DomainParticipantFactory::get_instance()->create_participant(
            0, pqos_no_physical);

        /* Load physical properties from default XML file */
        DomainParticipantFactory::get_instance()->load_profiles();
        DomainParticipantQos pqos_default_xml_physical =
                DomainParticipantFactory::get_instance()->get_default_participant_qos();
        DomainParticipant* participant_default_xml_physical =
                DomainParticipantFactory::get_instance()->create_participant(0, pqos_default_xml_physical);

        /* Load physical properties from specific XML file */
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("somefile.xml");
        DomainParticipantFactory::get_instance()->load_profiles();
        DomainParticipantQos pqos_custom_xml_physical =
                DomainParticipantFactory::get_instance()->get_default_participant_qos();
        DomainParticipant* participant_custom_xml_physical =
                DomainParticipantFactory::get_instance()->create_participant(0, pqos_custom_xml_physical);
        //!--

        DomainParticipantFactory::get_instance()->delete_participant(participant_with_physical);
        DomainParticipantFactory::get_instance()->delete_participant(participant_custom_physical);
        DomainParticipantFactory::get_instance()->delete_participant(participant_without_physical);
        DomainParticipantFactory::get_instance()->delete_participant(participant_default_xml_physical);
        DomainParticipantFactory::get_instance()->delete_participant(participant_custom_xml_physical);
    }
    {
        // ENABLE_DISABLE_STATISTICS_DATAWRITER
        // Create a DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Obtain pointer to child class
        eprosima::fastdds::statistics::dds::DomainParticipant* statistics_participant =
                eprosima::fastdds::statistics::dds::DomainParticipant::narrow(participant);

        // Enable statistics DataWriter
        if (statistics_participant->enable_statistics_datawriter(eprosima::fastdds::statistics::GAP_COUNT_TOPIC,
                eprosima::fastdds::statistics::dds::STATISTICS_DATAWRITER_QOS) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Use the DomainParticipant to communicate
        // (...)

        // Disable statistics DataWriter
        if (statistics_participant->disable_statistics_datawriter(eprosima::fastdds::statistics::GAP_COUNT_TOPIC) !=
                RETCODE_OK)
        {
            // Error
            return;
        }

        // Delete DomainParticipant
        if (DomainParticipantFactory::get_instance()->delete_participant(participant) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        // PULL_MODE_DATAWRITER
        DataWriterQos wqos;

        // Enable pull mode
        wqos.properties().properties().emplace_back(
            "fastdds.push_mode",
            "false");
        //!--
    }

    {
        //CONF-QOS-PARTITIONS
        PublisherQos pub_11_qos;
        pub_11_qos.partition().push_back("Partition_1");
        pub_11_qos.partition().push_back("Partition_2");

        PublisherQos pub_12_qos;
        pub_12_qos.partition().push_back("*");

        PublisherQos pub_21_qos;
        //No partitions defined for pub_21

        PublisherQos pub_22_qos;
        pub_22_qos.partition().push_back("Partition*");

        SubscriberQos subs_31_qos;
        subs_31_qos.partition().push_back("Partition_1");

        SubscriberQos subs_32_qos;
        subs_32_qos.partition().push_back("Partition_2");

        SubscriberQos subs_33_qos;
        subs_33_qos.partition().push_back("Partition_3");

        SubscriberQos subs_34_qos;
        //No partitions defined for subs_34
        //!--
    }

    {
        // PARTITION-ON-ENDPOINT
        DataWriterQos wqos;

        // Add partitions
        wqos.properties().properties().emplace_back(
            "partitions",
            "part1;part2");

        DataReaderQos rqos;

        // Add partitions
        rqos.properties().properties().emplace_back(
            "partitions",
            "part1;part2");
        //!--
    }

    {
        //DDS-STATIC-DISCOVERY-FORMAT
        DomainParticipantQos participant_qos;
        participant_qos.properties().properties().emplace_back(
            "dds.discovery.static_edp.exchange_format",
            "v1_Reduced"
            );
        //!--
    }

    {
        // IGNORE_LOCAL_ENDPOINTS_DOMAINPARTICIPANT
        DomainParticipantQos participant_qos;

        // Avoid local matching of this participant's endpoints
        participant_qos.properties().properties().emplace_back(
            "fastdds.ignore_local_endpoints",
            "true");
        //!--
    }

    {
        //DDS-SHM-ENFORCE-META-TRAFFIC
        DomainParticipantQos participant_qos;

        // SHM transport will listen for unicast meta-traffic
        participant_qos.properties().properties().emplace_back(
            "fastdds.shm.enforce_metatraffic",
            "unicast");
        //!--
    }

    {
        // MAX_MESSAGE_SIZE_PROPERTY_PARTICIPANT
        DomainParticipantQos pqos;

        // Set maximum number of bytes of the datagram to be sent
        pqos.properties().properties().emplace_back(
            "fastdds.max_message_size",
            "1200");
        //!--
    }

    {
        // MAX_MESSAGE_SIZE_PROPERTY_WRITER
        DataWriterQos wqos;

        // Set maximum number of bytes of the datagram to be sent
        wqos.properties().properties().emplace_back(
            "fastdds.max_message_size",
            "1200");
        //!--
    }

    {
        // TYPE_PROPAGATION_PROPERTY
        DomainParticipantQos pqos;

        pqos.properties().properties().emplace_back(
            "fastdds.type_propagation",
            "enabled");
        //!--
    }
}

//DOMAINPARTICIPANTLISTENER-DISCOVERY-CALLBACKS
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
                std::cout << "New DomainParticipant '" << info.participant_name <<
                    "' with ID '" << info.guid.entityId << "' and GuidPrefix '" <<
                    info.guid.guidPrefix << "' discovered." << std::endl;
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
                std::cout << "DomainParticipant '" << info.participant_name <<
                    "' with ID '" << info.guid.entityId << "' and GuidPrefix '" <<
                    info.guid.guidPrefix << "' left the domain." << std::endl;
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
                std::cout << "New DataReader subscribed to topic '" << info.topic_name <<
                    "' of type '" << info.type_name << "' discovered";
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
                std::cout << "DataReader subscribed to topic '" << info.topic_name <<
                    "' of type '" << info.type_name << "' left the domain.";
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
                std::cout << "New DataWriter publishing under topic '" << info.topic_name <<
                    "' of type '" << info.type_name << "' discovered";
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
                std::cout << "DataWriter publishing under topic '" << info.topic_name <<
                    "' of type '" << info.type_name << "' left the domain.";
                break;
        }
    }

};
//!--

//!--REMOTE_TYPE_MATCHING
class RemoteDiscoveryDomainParticipantListener : public DomainParticipantListener
{
    /* Custom Callback on_data_reader_discovery */
    void on_data_reader_discovery(
            DomainParticipant* participant,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::rtps::SubscriptionBuiltinTopicData& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        // Get remote type information
        xtypes::TypeObject remote_type_object;
        if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                    info.type_information.type_information.complete().typeid_with_size().type_id(),
                    remote_type_object))
        {
            // Error
            return;
        }
        // Register remotely discovered type
        DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
            remote_type_object)->build();
        TypeSupport dyn_type_support(new DynamicPubSubType(remote_type));
        dyn_type_support.register_type(participant);

        // Create a Topic with the remotely discovered type.
        Topic* topic =
                participant->create_topic(info.topic_name.to_string(), dyn_type_support.get_type_name(),
                        TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create endpoint
        Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        DataWriter* data_writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer)
        {
            // Error
            return;
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
        // Get remote type information
        xtypes::TypeObject remote_type_object;
        if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                    info.type_information.type_information.complete().typeid_with_size().type_id(),
                    remote_type_object))
        {
            // Error
            return;
        }
        // Register remotely discovered type
        DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
            remote_type_object)->build();
        TypeSupport dyn_type_support(new DynamicPubSubType(remote_type));
        dyn_type_support.register_type(participant);

        // Create a Topic with the remotely discovered type.
        Topic* topic =
                participant->create_topic(info.topic_name.to_string(), dyn_type_support.get_type_name(),
                        TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create endpoint
        Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // The QoS depends on the remote endpoint QoS. For simplicity, default QoS have been assumed.
        DataReader* data_reader = subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }
    }

};
//!--

//!--REMOTE_TYPE_INTROSPECTION
class TypeIntrospectionSubscriber : public DomainParticipantListener
{
    //!--DYNTYPE_IDL_SERIALIZATION
    /* Custom Callback on_data_reader_discovery */
    void on_data_reader_discovery(
            DomainParticipant* /* participant */,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus /* reason */,
            const eprosima::fastdds::dds::SubscriptionBuiltinTopicData& info,
            bool& /* should_be_ignored */) override
    {
        // Get remote type information
        xtypes::TypeObject remote_type_object;
        if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                    info.type_information.type_information.complete().typeid_with_size().type_id(),
                    remote_type_object))
        {
            // Error
            return;
        }

        // Build remotely discovered type
        DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
            remote_type_object)->build();

        // Serialize DynamicType into its IDL representation
        std::stringstream idl;
        idl_serialize(remote_type, idl);

        // Print IDL representation
        std::cout << "Type discovered:\n" << idl.str() << std::endl;
    }

    /* Custom Callback on_data_writer_discovery */
    void on_data_writer_discovery(
            DomainParticipant* /* participant */,
            eprosima::fastdds::rtps::WriterDiscoveryStatus /*reason*/,
            const eprosima::fastdds::dds::PublicationBuiltinTopicData& info,
            bool& /* should_be_ignored */) override
    {
        // Get remote type information
        xtypes::TypeObject remote_type_object;
        if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                    info.type_information.type_information.complete().typeid_with_size().type_id(),
                    remote_type_object))
        {
            // Error
            return;
        }

        // Build remotely discovered type
        DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
            remote_type_object)->build();

        // Serialize DynamicType into its IDL representation
        std::stringstream idl;
        idl_serialize(remote_type, idl);

        // Print IDL representation
        std::cout << "Type discovered:\n" << idl.str() << std::endl;
    }

    //!--

    //!--DYNDATA_JSON_SERIALIZATION
    void on_data_available(
            DataReader* reader)
    {
        // Dynamic DataType
        DynamicData::_ref_type new_data =
                DynamicDataFactory::get_instance()->create_data(dyn_type_);

        SampleInfo info;

        while ((RETCODE_OK == reader->take_next_sample(&new_data, &info)))
        {
            std::stringstream output;
            output << std::setw(4);

            // Serialize DynamicData into JSON string format
            json_serialize(new_data, DynamicDataJsonFormat::EPROSIMA, output);
            std::cout << "Message received:\n" << output.str() << std::endl;
        }
    }

    // DynamicType created in discovery callback
    DynamicType::_ref_type dyn_type_;
    //!--
};
//!--

void dds_discovery_examples()
{
    using Locator_t = eprosima::fastdds::rtps::Locator_t;
    using IPLocator = eprosima::fastdds::rtps::IPLocator;
    using DiscoveryProtocol = eprosima::fastdds::rtps::DiscoveryProtocol;
    using ParticipantFilteringFlags = eprosima::fastdds::rtps::ParticipantFilteringFlags;
    {
        //SET-DISCOVERY-CALLBACKS
        // Create the participant QoS and configure values
        DomainParticipantQos pqos;

        // Create a custom user DomainParticipantListener
        DiscoveryDomainParticipantListener* plistener = new DiscoveryDomainParticipantListener();
        // Pass the listener on DomainParticipant creation.
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(
            0, pqos, plistener);
        //!--
    }
    {
        //CONF-DISCOVERY-PROTOCOL
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::SIMPLE;
        //!--
    }
    {
        //CONF-DISCOVERY-IGNORE-FLAGS
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.ignoreParticipantFlags =
                static_cast<eprosima::fastdds::rtps::ParticipantFilteringFlags>(
            ParticipantFilteringFlags::FILTER_DIFFERENT_PROCESS |
            ParticipantFilteringFlags::FILTER_SAME_PROCESS);
        //!--
    }
    {
        //CONF-DISCOVERY-LEASE-DURATION
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.leaseDuration = Duration_t(10, 20);
        //!--
    }
    {
        //CONF-DISCOVERY-LEASE-ANNOUNCEMENT
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(1, 2);
        //!--
    }
    {
        //DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.initial_announcements.count = 5;
        pqos.wire_protocol().builtin.discovery_config.initial_announcements.period = Duration_t(0, 100000000u);
        //!--
    }
    {
        //CONF-QOS-DISCOVERY-EDP-ATTRIBUTES
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = false;
        //!--
    }
    {
        //CONF_STATIC_DISCOVERY_CODE
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
        pqos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
        //!--
    }
    {
        //CONF_QOS_STATIC_DISCOVERY_USERID
        // Configure the DataWriter
        DataWriterQos wqos;
        wqos.endpoint().user_defined_id = 1;

        // Configure the DataReader
        DataReaderQos rqos;
        rqos.endpoint().user_defined_id = 3;
        //!--
    }
    {
        //CONF_STATIC_DISCOVERY_XML_FILE
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config("file://RemotePublisher.xml");
        pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config("file://RemoteSubscriber.xml");
        //!--
    }
    {
        //CONF_STATIC_DISCOVERY_XML_DATA
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.static_edp_xml_config(
            "data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
            "<staticdiscovery><participant><name>RTPSParticipant</name></participant></staticdiscovery>");
        //!--
    }
    {
        //CONF_SERVER_DISCOVERY_PROTOCOL
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::CLIENT;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::SUPER_CLIENT;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::SERVER;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::BACKUP;
        //!--
    }
    {
        //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_1
        // Using the ``>>`` operator and the ``std::istringstream`` type.
        DomainParticipantQos serverQos;
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> serverQos.wire_protocol().prefix;
        //!--
    }
    {
        //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_2
        // Manual setting of the ``unsigned char`` in ASCII format.
        eprosima::fastdds::rtps::GuidPrefix_t serverGuidPrefix;
        serverGuidPrefix.value[0] = eprosima::fastdds::rtps::octet(0x44);
        serverGuidPrefix.value[1] = eprosima::fastdds::rtps::octet(0x53);
        serverGuidPrefix.value[2] = eprosima::fastdds::rtps::octet(0x00);
        serverGuidPrefix.value[3] = eprosima::fastdds::rtps::octet(0x5f);
        serverGuidPrefix.value[4] = eprosima::fastdds::rtps::octet(0x45);
        serverGuidPrefix.value[5] = eprosima::fastdds::rtps::octet(0x50);
        serverGuidPrefix.value[6] = eprosima::fastdds::rtps::octet(0x52);
        serverGuidPrefix.value[7] = eprosima::fastdds::rtps::octet(0x4f);
        serverGuidPrefix.value[8] = eprosima::fastdds::rtps::octet(0x53);
        serverGuidPrefix.value[9] = eprosima::fastdds::rtps::octet(0x49);
        serverGuidPrefix.value[10] = eprosima::fastdds::rtps::octet(0x4d);
        serverGuidPrefix.value[11] = eprosima::fastdds::rtps::octet(0x41);

        DomainParticipantQos serverQos;
        serverQos.wire_protocol().prefix = serverGuidPrefix;
        //!--
    }
    {
        //CONF_SERVER_CLIENT_LOCATORS
        Locator_t locator;
        // The default locator kind is UDPv4
        locator.kind = LOCATOR_KIND_UDPv4;
        IPLocator::setIPv4(locator, 192, 168, 1, 133);
        locator.port = 64863;

        DomainParticipantQos clientQos;
        clientQos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(locator);
        //!--
    }

    {
        //CONF_SERVER_SERVER_LOCATORS
        Locator_t locator;
        // The default locator kind is UDPv4
        locator.kind = LOCATOR_KIND_UDPv4;
        IPLocator::setIPv4(locator, 192, 168, 1, 133);
        locator.port = 64863;

        DomainParticipantQos serverQos;
        serverQos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF_SERVER_CLIENT_PING
        DomainParticipantQos participant_qos;
        participant_qos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =
                Duration_t(0, 250000000);
        //!--
    }

    {
        DomainParticipant* client_or_server;
        //CONF_SERVER_ADD_SERVERS
        // Get existing QoS for the server or client
        DomainParticipantQos client_or_server_qos;
        client_or_server->get_qos(client_or_server_qos);

        /* Create a new server entry to which the client or server should connect */
        // Set server's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11812;

        /* Update list of remote servers for this client or server */
        client_or_server_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(locator);
        if (RETCODE_OK != client_or_server->set_qos(client_or_server_qos))
        {
            // Error
            return;
        }
        //!--
    }

    {
        //CONF_SERVER_DNS_LOCATORS
        Locator_t locator;
        auto response = eprosima::fastdds::rtps::IPLocator::resolveNameDNS("localhost");
        // Get the first returned IPv4
        if (response.first.size() > 0)
        {
            IPLocator::setIPv4(locator, response.first.begin()->data());
            locator.port = 11811;
        }
        // Use the locator to create server or client
        //!--
    }

    {
        //CONF_SERVER_FULL_EXAMPLE
        // Get default participant QoS
        DomainParticipantQos server_qos = PARTICIPANT_QOS_DEFAULT;

        // Set participant as SERVER
        server_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::SERVER;

        // Set SERVER's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11811;
        server_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);

        /* Add a remote serve to which this server will connect */
        // Set remote SERVER's listening locator for PDP
        Locator_t remote_locator;
        IPLocator::setIPv4(remote_locator, 127, 0, 0, 1);
        remote_locator.port = 11812;

        // Add remote SERVER to SERVER's list of SERVERs
        server_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_locator);

        // Create SERVER
        DomainParticipant* server =
                DomainParticipantFactory::get_instance()->create_participant(0, server_qos);
        if (nullptr == server)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //CONF_CLIENT_FULL_EXAMPLE
        // Get default participant QoS
        DomainParticipantQos client_qos = PARTICIPANT_QOS_DEFAULT;

        // Set participant as CLIENT
        client_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol::CLIENT;

        // Set SERVER's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11811;

        // Add remote SERVER to CLIENT's list of SERVERs
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(locator);

        // Set ping period to 250 ms
        client_qos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =
                Duration_t(0, 250000000);

        // Create CLIENT
        DomainParticipant* client =
                DomainParticipantFactory::get_instance()->create_participant(0, client_qos);
        if (nullptr == client)
        {
            // Error
            return;
        }
        //!--

        // Check XML static discovery from file
        // The (file://) flag is optional.
        std::string file = "file://static_Discovery.xml";
        DomainParticipantFactory* factory = DomainParticipantFactory::get_instance();
        if (RETCODE_OK != factory->check_xml_static_discovery(file))
        {
            std::cout << "Error parsing xml file " << file << std::endl;
        }
        //!--

        // Check XML static discovery from data
        // The (data://) flag is required to load the configuration directly.
        std::string fileData = "data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
                "<staticdiscovery>" \
                "<participant>" \
                "<name>HelloWorldPublisher</name>" \
                "<writer>" \
                "<userId>1</userId>" \
                "<entityID>2</entityID>" \
                "<topicName>HelloWorldTopic</topicName>" \
                "<topicDataType>HelloWorld</topicDataType>" \
                "</writer>" \
                "</participant>" \
                "</staticdiscovery>";
        if (RETCODE_OK != factory->check_xml_static_discovery(fileData))
        {
            std::cout << "Error parsing xml file data:" << std::endl << fileData << std::endl;
        }
        //!--
    }
}

//DDS_TOPIC_LISTENER_SPECIALIZATION
class CustomTopicListener : public TopicListener
{

public:

    CustomTopicListener()
        : TopicListener()
    {
    }

    virtual ~CustomTopicListener()
    {
    }

    void on_inconsistent_topic(
            Topic* topic,
            InconsistentTopicStatus status) override
    {
        static_cast<void>(topic);
        static_cast<void>(status);
        std::cout << "Inconsistent topic received discovered" << std::endl;
    }

};
//!--

struct Foo
{
    int32_t a;
    uint64_t b;
};

FASTDDS_SEQUENCE(FooSeq, Foo);

class CustomDataType : public TopicDataType
{
public:

    CustomDataType()
        : TopicDataType()
    {
        set_name("Foo");
    }

    bool serialize(
            const void* const data,
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override
    {
        return true;
    }

    bool deserialize(
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            void* data) override
    {
        return true;
    }

    uint32_t calculate_serialized_size(
            const void* const data,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override
    {
        return 0;
    }

    void* create_data() override
    {
        return nullptr;
    }

    void delete_data(
            void* data) override
    {
    }

    bool compute_key(
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
            bool force_md5) override
    {
        return true;
    }

    bool compute_key(
            const void* const data,
            eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
            bool force_md5) override
    {
        return true;
    }

};

void dds_topic_examples()
{
    {
        //DDS_CREATE_TOPIC
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic with default TopicQos and no Listener
        // The symbol TOPIC_QOS_DEFAULT is used to denote the default QoS.
        Topic* topic_with_default_qos =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr == topic_with_default_qos)
        {
            // Error
            return;
        }

        // A custom TopicQos can be provided to the creation method
        TopicQos custom_qos;

        // Modify QoS attributes
        // (...)

        Topic* topic_with_custom_qos =
                participant->create_topic("TopicName", "DataTypeName", custom_qos);
        if (nullptr == topic_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a Topic with default QoS and a custom Listener.
        // CustomTopicListener inherits from TopicListener.
        // The symbol TOPIC_QOS_DEFAULT is used to denote the default QoS.
        CustomTopicListener custom_listener;
        Topic* topic_with_default_qos_and_custom_listener =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT, &custom_listener);
        if (nullptr == topic_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_TOPIC
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic using a profile and no Listener
        Topic* topic_with_profile =
                participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile");
        if (nullptr == topic_with_profile)
        {
            // Error
            return;
        }

        // Create a Topic using a profile and a custom Listener.
        // CustomTopicListener inherits from TopicListener.
        CustomTopicListener custom_listener;
        Topic* topic_with_profile_and_custom_listener =
                participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile", &custom_listener);
        if (nullptr == topic_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_TOPICQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic with default TopicQos
        Topic* topic =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        TopicQos qos = topic->get_qos();

        // Modify QoS attributes
        // (...)

        // Assign the new Qos to the object
        topic->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_TOPICQOS_TO_DEFAULT
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a custom TopicQos
        TopicQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a topic with a custom TopicQos
        Topic* topic = participant->create_topic("TopicName", "DataTypeName", custom_qos);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Set the QoS on the topic to the default
        if (topic->set_qos(TOPIC_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (topic->set_qos(participant->get_default_topic_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_TOPIC
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic
        Topic* topic =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Use the Topic to communicate
        // (...)

        // Delete the Topic
        if (participant->delete_topic(topic) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_TOPICQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        TopicQos qos_type1 = participant->get_default_topic_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if (participant->set_default_topic_qos(qos_type1) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        Topic* topic_with_qos_type1 =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr == topic_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        TopicQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if (participant->set_default_topic_qos(qos_type2) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        Topic* topic_with_qos_type2 =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr == topic_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default TopicQos to the original default constructed values
        if (participant->set_default_topic_qos(TOPIC_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_topic_qos(TopicQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_TYPE_REGISTER
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Register the data type in the DomainParticipant.
        // If nullptr is used as name argument, the one returned by the type itself is used
        TypeSupport custom_type_support(new CustomDataType());
        custom_type_support.register_type(participant, nullptr);

        // The previous instruction is equivalent to the following one
        // Even if we are registering the same data type with the same name twice, no error will be issued
        custom_type_support.register_type(participant, custom_type_support.get_type_name());

        // Create a Topic with the registered type.
        Topic* topic =
                participant->create_topic("topic_name", custom_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create an alias for the same data type using a different name.
        custom_type_support.register_type(participant, "data_type_name");

        // We can now use the aliased name to If no name is given, it uses the name returned by the type itself
        Topic* another_topic =
                participant->create_topic("other_topic_name", "data_type_name", TOPIC_QOS_DEFAULT);
        if (nullptr == another_topic)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DYNAMIC_TYPES
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Load the XML file with the type description
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("example_type.xml");

        // Retrieve the an instance of the desired type
        DynamicTypeBuilder::_ref_type dyn_type_builder;
        DomainParticipantFactory::get_instance()->get_dynamic_type_builder_from_xml_by_name("DynamicType",
                dyn_type_builder);

        // Register dynamic type
        TypeSupport dyn_type_support(new DynamicPubSubType(dyn_type_builder->build()));
        dyn_type_support.register_type(participant, nullptr);

        // Create a Topic with the registered type.
        Topic* topic =
                participant->create_topic("topic_name", dyn_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }
        //!--
    }
}

void dds_content_filtered_topic_examples()
{
    {
        //DDS_CREATE_CONTENT_FILTERED_TOPIC
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create the Topic.
        /* IDL
         *
         * struct HelloWorld
         * {
         *     long index;
         *     string message;
         * }
         *
         */
        Topic* topic =
                participant->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create a ContentFilteredTopic using an expression with no parameters
        std::string expression = "message like 'Hello*'";
        std::vector<std::string> parameters;
        ContentFilteredTopic* filter_topic =
                participant->create_contentfilteredtopic("HelloWorldFilteredTopic1", topic, expression, parameters);
        if (nullptr == filter_topic)
        {
            // Error
            return;
        }

        // Create a ContentFilteredTopic using an expression with parameters
        expression = "message like %0 or index > %1";
        parameters.push_back("'*world*'");
        parameters.push_back("20");
        ContentFilteredTopic* filter_topic_with_parameters =
                participant->create_contentfilteredtopic("HelloWorldFilteredTopic2", topic, expression, parameters);
        if (nullptr == filter_topic_with_parameters)
        {
            // Error
            return;
        }

        // The ContentFilteredTopic instances can then be used to create DataReader objects.
        Subscriber* subscriber =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        DataReader* reader_on_filter = subscriber->create_datareader(filter_topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == reader_on_filter)
        {
            // Error
            return;
        }

        DataReader* reader_on_filter_with_parameters =
                subscriber->create_datareader(filter_topic_with_parameters, DATAREADER_QOS_DEFAULT);
        if (nullptr == reader_on_filter_with_parameters)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_UPDATE_CONTENT_FILTERED_TOPIC

        // This lambda prints all the information of a ContentFilteredTopic
        auto print_filter_info = [](
            const ContentFilteredTopic* filter_topic)
                {
                    std::cout << "ContentFilteredTopic info for '" << filter_topic->get_name() << "':" << std::endl;
                    std::cout << "  - Related Topic: " << filter_topic->get_related_topic()->get_name() << std::endl;
                    std::cout << "  - Expression:    " << filter_topic->get_filter_expression() << std::endl;
                    std::cout << "  - Parameters:" << std::endl;

                    std::vector<std::string> parameters;
                    filter_topic->get_expression_parameters(parameters);
                    size_t i = 0;
                    for (const std::string& parameter : parameters)
                    {
                        std::cout << "    " << i++ << ": " << parameter << std::endl;
                    }
                };

        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic
        /* IDL
         *
         * struct HelloWorld
         * {
         *     long index;
         *     string message;
         * }
         *
         */
        Topic* topic =
                participant->create_topic("HelloWorldTopic", "HelloWorldTopic", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create a ContentFilteredTopic
        ContentFilteredTopic* filter_topic =
                participant->create_contentfilteredtopic("HelloWorldFilteredTopic", topic, "index > 10", {});
        if (nullptr == filter_topic)
        {
            // Error
            return;
        }

        // Print the information
        print_filter_info(filter_topic);

        // Use the ContentFilteredTopic on DataReader objects.
        // (...)

        // Update the expression
        if (RETCODE_OK !=
                filter_topic->set_filter_expression("message like %0 or index > %1", {"'Hello*'", "15"}))
        {
            // Error
            return;
        }

        // Print the updated information
        print_filter_info(filter_topic);

        // Update the parameters
        if (RETCODE_OK !=
                filter_topic->set_expression_parameters({"'*world*'", "222"}))
        {
            // Error
            return;
        }

        // Print the updated information
        print_filter_info(filter_topic);

        //!--


        //DDS_CONTENT_FILTERED_TOPIC_SQL_EXAMPLE
        ContentFilteredTopic* sql_filter_topic =
                participant->create_contentfilteredtopic("Shape", topic,
                        "x < 23 AND y > 50 AND width BETWEEN %0 AND %1",
                        {"10", "20"});
        //!--
    }


    {
        //DDS_DELETE_CONTENT_FILTERED_TOPIC
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Topic
        /* IDL
         *
         * struct HelloWorld
         * {
         *     long index;
         *     string message;
         * }
         *
         */
        Topic* topic =
                participant->create_topic("HelloWorldTopic", "HelloWorldTopic", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create a ContentFilteredTopic
        ContentFilteredTopic* filter_topic =
                participant->create_contentfilteredtopic("HelloWorldFilteredTopic", topic, "index > 10", {});
        if (nullptr == filter_topic)
        {
            // Error
            return;
        }

        // Use the ContentFilteredTopic on DataReader objects.
        // (...)

        // Delete the ContentFilteredTopic
        if (RETCODE_OK != participant->delete_contentfilteredtopic(filter_topic))
        {
            // Error
            return;
        }
        //!--
    }
}

void dds_custom_filters_examples()
{
    //DDS_CUSTOM_FILTER_CLASS
    class MyCustomFilter : public IContentFilter
    {
    public:

        MyCustomFilter(
                int low_mark,
                int high_mark)
            : low_mark_(low_mark)
            , high_mark_(high_mark)
        {
        }

        bool evaluate(
                const SerializedPayload& payload,
                const FilterSampleInfo& sample_info,
                const GUID_t& reader_guid) const override
        {
            // Deserialize the `index` field from the serialized sample.
            /* IDL
             *
             * struct HelloWorld
             * {
             *     long index;
             *     string message;
             * }
             */
            eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload.data), payload.length);
            eprosima::fastcdr::Cdr deser(fastbuffer);
            // Deserialize encapsulation.
            deser.read_encapsulation();
            int index = 0;

            // Deserialize `index` field.
            try
            {
                deser >> index;
            }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException& exception)
            {
                return false;
            }

            // Custom filter: reject samples where index > low_mark_ and index < high_mark_.
            if (index > low_mark_ && index < high_mark_)
            {
                return false;
            }

            return true;
        }

    private:

        int low_mark_ = 0;
        int high_mark_ = 0;

    };
    //!--


    //DDS_CUSTOM_FILTER_FACTORY_CLASS
    class MyCustomFilterFactory : public IContentFilterFactory
    {
    public:

        ReturnCode_t create_content_filter(
                const char* filter_class_name, // My custom filter class name is 'MY_CUSTOM_FILTER'.
                const char* type_name, // This custom filter only supports one type: 'HelloWorld'.
                const TopicDataType* /*data_type*/, // Not used in this implementation.
                const char* filter_expression, // This Custom Filter doesn't implement a filter expression.
                const ParameterSeq& filter_parameters, // Always need two parameters to be set: low_mark and high_mark.
                IContentFilter*& filter_instance) override
        {
            // Check the ContentFilteredTopic should be created by my factory.
            if (0 != strcmp(filter_class_name, "MY_CUSTOM_FILTER"))
            {
                return RETCODE_BAD_PARAMETER;
            }

            // Check the ContentFilteredTopic is created for the unique type this Custom Filter supports.
            if (0 != strcmp(type_name, "HelloWorld"))
            {
                return RETCODE_BAD_PARAMETER;
            }

            // Check that the two mandatory filter parameters are set.
            if (2 != filter_parameters.length())
            {
                return RETCODE_BAD_PARAMETER;
            }

            // If there is an update, delete previous instance.
            if (nullptr != filter_instance)
            {
                delete(dynamic_cast<MyCustomFilter*>(filter_instance));
            }

            // Instantiation of the Custom Filter.
            filter_instance = new MyCustomFilter(std::stoi(filter_parameters[0]), std::stoi(filter_parameters[1]));

            return RETCODE_OK;
        }

        ReturnCode_t delete_content_filter(
                const char* filter_class_name,
                IContentFilter* filter_instance) override
        {
            // Check the ContentFilteredTopic should be created by my factory.
            if (0 != strcmp(filter_class_name, "MY_CUSTOM_FILTER"))
            {
                return RETCODE_BAD_PARAMETER;
            }

            // Deletion of the Custom Filter.
            delete(dynamic_cast<MyCustomFilter*>(filter_instance));

            return RETCODE_OK;
        }

    };
    //!--

    {
        //DDS_CUSTOM_FILTER_REGISTER_FACTORY
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create Custom Filter Factory
        MyCustomFilterFactory* factory = new MyCustomFilterFactory();


        // Registration of the factory
        if (RETCODE_OK !=
                participant->register_content_filter_factory("MY_CUSTOM_FILTER", factory))
        {
            // Error
            return;
        }
        //!--
    }


    {
        //DDS_CUSTOM_FILTER_CREATE_TOPIC
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create the Topic.
        Topic* topic =
                participant->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create a ContentFilteredTopic selecting the Custom Filter and using no expression with two parameters
        // Filter expression cannot be an empty one even when it is not used by the custom filter, as that effectively
        // disables any filtering
        std::string expression = " ";
        std::vector<std::string> parameters;
        parameters.push_back("10"); // Parameter for low_mark
        parameters.push_back("20"); // Parameter for low_mark
        ContentFilteredTopic* filter_topic =
                participant->create_contentfilteredtopic("HelloWorldFilteredTopic1", topic, expression, parameters,
                        "MY_CUSTOM_FILTER");
        if (nullptr == filter_topic)
        {
            // Error
            return;
        }

        // The ContentFilteredTopic instances can then be used to create DataReader objects.
        Subscriber* subscriber =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        DataReader* reader_on_filter = subscriber->create_datareader(filter_topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == reader_on_filter)
        {
            // Error
            return;
        }
        //!--
    }
}

class CustomPublisherListener : public PublisherListener
{
};

void dds_publisher_examples()
{
    {
        //DDS_CREATE_PUBLISHER
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Publisher with default PublisherQos and no Listener
        // The value PUBLISHER_QOS_DEFAULT is used to denote the default QoS.
        Publisher* publisher_with_default_qos =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher_with_default_qos)
        {
            // Error
            return;
        }

        // A custom PublisherQos can be provided to the creation method
        PublisherQos custom_qos;

        // Modify QoS attributes
        // (...)

        Publisher* publisher_with_custom_qos =
                participant->create_publisher(custom_qos);
        if (nullptr == publisher_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a Publisher with default QoS and a custom Listener.
        // CustomPublisherListener inherits from PublisherListener.
        // The value PUBLISHER_QOS_DEFAULT is used to denote the default QoS.
        CustomPublisherListener custom_listener;
        Publisher* publisher_with_default_qos_and_custom_listener =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT, &custom_listener);
        if (nullptr == publisher_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_PUBLISHER
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Publisher using a profile and no Listener
        Publisher* publisher_with_profile =
                participant->create_publisher_with_profile("publisher_profile");
        if (nullptr == publisher_with_profile)
        {
            // Error
            return;
        }

        // Create a Publisher using a profile and a custom Listener.
        // CustomPublisherListener inherits from PublisherListener.
        CustomPublisherListener custom_listener;
        Publisher* publisher_with_profile_and_custom_listener =
                participant->create_publisher_with_profile("publisher_profile", &custom_listener);
        if (nullptr == publisher_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_PUBLISHERQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Publisher with default PublisherQos
        Publisher* publisher =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        PublisherQos qos = publisher->get_qos();

        // Modify QoS attributes
        // (...)

        // Assign the new Qos to the object
        publisher->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_PUBLISHERQOS_TO_DEFAULT
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a custom PublisherQos
        PublisherQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a publisher with a custom PublisherQos
        Publisher* publisher = participant->create_publisher(custom_qos);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        // Set the QoS on the publisher to the default
        if (publisher->set_qos(PUBLISHER_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (publisher->set_qos(participant->get_default_publisher_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_PUBLISHER
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Publisher
        Publisher* publisher =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        // Use the Publisher to communicate
        // (...)

        // Delete the entities the Publisher created.
        if (publisher->delete_contained_entities() != RETCODE_OK)
        {
            // Publisher failed to delete the entities it created.
            return;
        }

        // Delete the Publisher
        if (participant->delete_publisher(publisher) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_PUBLISHERQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        PublisherQos qos_type1 = participant->get_default_publisher_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default PublisherQos
        if (participant->set_default_publisher_qos(qos_type1) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Publisher with the new default PublisherQos.
        Publisher* publisher_with_qos_type1 =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        PublisherQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default PublisherQos
        if (participant->set_default_publisher_qos(qos_type2) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Publisher with the new default PublisherQos.
        Publisher* publisher_with_qos_type2 =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default PublisherQos to the original default constructed values
        if (participant->set_default_publisher_qos(PUBLISHER_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_publisher_qos(PublisherQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }
}

//DDS_DATAWRITER_LISTENER_SPECIALIZATION
class CustomDataWriterListener : public DataWriterListener
{

public:

    CustomDataWriterListener()
        : DataWriterListener()
    {
    }

    virtual ~CustomDataWriterListener()
    {
    }

    void on_publication_matched(
            DataWriter* writer,
            const PublicationMatchedStatus& info) override
    {
        static_cast<void>(writer);
        if (info.current_count_change == 1)
        {
            std::cout << "Matched a remote Subscriber for one of our Topics" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Unmatched a remote Subscriber" << std::endl;
        }
    }

    void on_offered_deadline_missed(
            DataWriter* writer,
            const OfferedDeadlineMissedStatus& status) override
    {
        static_cast<void>(writer);
        static_cast<void>(status);
        std::cout << "Some data could not be delivered on time" << std::endl;
    }

    void on_offered_incompatible_qos(
            DataWriter* writer,
            const OfferedIncompatibleQosStatus& status) override
    {
        std::cout << "Found a remote Topic with incompatible QoS (QoS ID: " << status.last_policy_id <<
            ")" << std::endl;
    }

    void on_liveliness_lost(
            DataWriter* writer,
            const LivelinessLostStatus& status) override
    {
        static_cast<void>(writer);
        static_cast<void>(status);
        std::cout << "Liveliness lost. Matched Subscribers will consider us offline" << std::endl;
    }

    void on_unacknowledged_sample_removed(
            DataWriter* writer,
            const InstanceHandle_t& instance) override
    {
        static_cast<void>(writer);
        static_cast<void>(instance);
        std::cout << "Sample removed unacknowledged" << std::endl;
    }

};
//!--

void dds_dataWriter_examples()
{
    // Taken out of the examples to avoid bloating them
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    Publisher* publisher =
            participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    Topic* topic =
            participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

    {
        //DDS_CREATE_DATAWRITER
        // Create a DataWriter with default DataWriterQos and no Listener
        // The value DATAWRITER_QOS_DEFAULT is used to denote the default QoS.
        DataWriter* data_writer_with_default_qos =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer_with_default_qos)
        {
            // Error
            return;
        }

        // A custom DataWriterQos can be provided to the creation method
        DataWriterQos custom_qos;

        // Modify QoS attributes
        // (...)

        DataWriter* data_writer_with_custom_qos =
                publisher->create_datawriter(topic, custom_qos);
        if (nullptr == data_writer_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a DataWriter with default QoS and a custom Listener.
        // CustomDataWriterListener inherits from DataWriterListener.
        // The value DATAWRITER_QOS_DEFAULT is used to denote the default QoS.
        CustomDataWriterListener custom_listener;
        DataWriter* data_writer_with_default_qos_and_custom_listener =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, &custom_listener);
        if (nullptr == data_writer_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }

        // Create a DataWriter with default QoS and a custom TopicQos.
        // The value DATAWRITER_QOS_USE_TOPIC_QOS is used to denote the default QoS
        // and to override the TopicQos.
        Topic* topic;
        DataWriter* data_writer_with_default_qos_and_custom_topic_qos =
                publisher->create_datawriter(topic, DATAWRITER_QOS_USE_TOPIC_QOS);
        if (nullptr == data_writer_with_default_qos_and_custom_topic_qos)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_DATAWRITER
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DataWriter using a profile and no Listener
        DataWriter* data_writer_with_profile =
                publisher->create_datawriter_with_profile(topic, "data_writer_profile");
        if (nullptr == data_writer_with_profile)
        {
            // Error
            return;
        }

        // Create a DataWriter using a profile and a custom Listener.
        // CustomDataWriterListener inherits from DataWriterListener.
        CustomDataWriterListener custom_listener;
        DataWriter* data_writer_with_profile_and_custom_listener =
                publisher->create_datawriter_with_profile(topic, "data_writer_profile", &custom_listener);
        if (nullptr == data_writer_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PAYLOAD_POOL_DATAWRITER
        // A DataWriterQos must be provided to the creation method
        DataWriterQos qos;

        // Create PayloadPool
        std::shared_ptr<eprosima::fastdds::rtps::IPayloadPool> payload_pool =
                std::dynamic_pointer_cast<eprosima::fastdds::rtps::IPayloadPool>(std::make_shared<CustomPayloadPool>());

        DataWriter* data_writer = publisher->create_datawriter(topic, qos, nullptr, StatusMask::all(), payload_pool);
        if (nullptr == data_writer)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DATAWRITERQOS
        // Create a DataWriter with default DataWriterQos
        DataWriter* data_writer =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DataWriterQos qos = data_writer->get_qos();

        // Modify QoS attributes
        // (...)

        // Assign the new Qos to the object
        data_writer->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_DATAWRITERQOS_TO_DEFAULT
        // Create a custom DataWriterQos
        DataWriterQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a DataWriter with a custom DataWriterQos
        DataWriter* data_writer = publisher->create_datawriter(topic, custom_qos);
        if (nullptr == data_writer)
        {
            // Error
            return;
        }

        // Set the QoS on the DataWriter to the default
        if (data_writer->set_qos(DATAWRITER_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (data_writer->set_qos(publisher->get_default_datawriter_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_DATAWRITER
        // Create a DataWriter
        DataWriter* data_writer =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer)
        {
            // Error
            return;
        }

        // Use the DataWriter to communicate
        // (...)

        // Delete the DataWriter
        if (publisher->delete_datawriter(data_writer) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_DATAWRITERQOS
        // Get the current QoS or create a new one from scratch
        DataWriterQos qos_type1 = publisher->get_default_datawriter_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default DataWriterQos
        if (publisher->set_default_datawriter_qos(qos_type1) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataWriter with the new default DataWriterQos.
        DataWriter* data_writer_with_qos_type1 =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DataWriterQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default DataWriterQos
        if (publisher->set_default_datawriter_qos(qos_type2) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataWriter with the new default DataWriterQos.
        DataWriter* data_writer_with_qos_type2 =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default DataWriterQos to the original default constructed values
        if (publisher->set_default_datawriter_qos(DATAWRITER_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (publisher->set_default_datawriter_qos(DataWriterQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DATAWRITER_WRITE
        // Register the data type in the DomainParticipant.
        TypeSupport custom_type_support(new CustomDataType());
        custom_type_support.register_type(participant, custom_type_support.get_type_name());

        // Create a Topic with the registered type.
        Topic* custom_topic =
                participant->create_topic("topic_name", custom_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
        if (nullptr == custom_topic)
        {
            // Error
            return;
        }

        // Create a DataWriter
        DataWriter* data_writer =
                publisher->create_datawriter(custom_topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == data_writer)
        {
            // Error
            return;
        }

        // Get a data instance
        void* data = custom_type_support->create_data();

        // Fill the data values
        // (...)

        // Publish the new value, deduce the instance handle
        if (data_writer->write(data, eprosima::fastdds::rtps::InstanceHandle_t()) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The data instance can be reused to publish new values,
        // but delete it at the end to avoid leaks
        custom_type_support->delete_data(data);
        //!--

        {
            //DDS_DATAWRITER_LOAN_SAMPLES
            // Borrow a data instance
            void* data = nullptr;
            if (RETCODE_OK == data_writer->loan_sample(data))
            {
                bool error = false;

                // Fill the data values
                // (...)

                if (error)
                {
                    // Return the loan without publishing
                    data_writer->discard_loan(data);
                    return;
                }

                // Publish the new value
                if (data_writer->write(data, eprosima::fastdds::rtps::InstanceHandle_t()) != RETCODE_OK)
                {
                    // Error
                    return;
                }
            }

            // The data instance can be reused to publish new values,
            // but delete it at the end to avoid leaks
            custom_type_support->delete_data(data);
            //!--
        }

        {
            struct FlightPosition
            {
                std::string airline_name_;
                int flight_number_;
                double latitude_;
                double longitude_;
                double altitude_;

                void airline_name(
                        std::string)
                {
                }

                void flight_number(
                        int)
                {
                }

                void latitude(
                        double)
                {
                }

                void longitude(
                        double)
                {
                }

                void altitude(
                        double)
                {
                }

            };


            //REGISTER-INSTANCE
            // Create data sample
            FlightPosition first_flight_position;

            // Specify the flight instance
            first_flight_position.airline_name("IBERIA");
            first_flight_position.flight_number(1234);

            // Register instance
            eprosima::fastdds::rtps::InstanceHandle_t first_flight_handle =
                    data_writer->register_instance(&first_flight_position);
            //!

            //WRITE-REGISTERED-INSTANCE
            // Update position value received from the plane
            first_flight_position.latitude(39.08);
            first_flight_position.longitude(-84.21);
            first_flight_position.altitude(1500);

            // Write sample to the instance
            data_writer->write(&first_flight_position, first_flight_handle);
            //!

            //WRITE-NON-REGISTERED-INSTANCE
            // New data sample
            FlightPosition second_flight_position;

            // New instance
            second_flight_position.airline_name("RYANAIR");
            second_flight_position.flight_number(4321);

            // Update plane location
            second_flight_position.latitude(40.02);
            second_flight_position.longitude(-84.32);
            second_flight_position.altitude(5000);

            // Write sample directly without registering the instance
            data_writer->write(&second_flight_position);
            //!

            //WRONG-INSTANCE-UPDATE
            data_writer->write(&second_flight_position, first_flight_handle);
            //!

            //UNREGISTER-INSTANCE
            data_writer->unregister_instance(&first_flight_position, first_flight_handle);
            data_writer->unregister_instance(&second_flight_position, HANDLE_NIL);
            //!

            //DISPOSE-INSTANCE
            data_writer->dispose(&first_flight_position, first_flight_handle);
            data_writer->dispose(&second_flight_position, HANDLE_NIL);
            //!
        }
    }
}

//DDS_SUBSCRIBER_LISTENER_SPECIALIZATION
class CustomSubscriberListener : public SubscriberListener
{

public:

    CustomSubscriberListener()
        : SubscriberListener()
    {
    }

    virtual ~CustomSubscriberListener()
    {
    }

    virtual void on_data_on_readers(
            Subscriber* sub)
    {
        static_cast<void>(sub);
        std::cout << "New data available" << std::endl;
    }

};
//!--

void dds_subscriber_examples()
{
    {
        //DDS_CREATE_SUBSCRIBER
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Subscriber with default SubscriberQos and no Listener
        // The value SUBSCRIBER_QOS_DEFAULT is used to denote the default QoS.
        Subscriber* subscriber_with_default_qos =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber_with_default_qos)
        {
            // Error
            return;
        }

        // A custom SubscriberQos can be provided to the creation method
        SubscriberQos custom_qos;

        // Modify QoS attributes
        // (...)

        Subscriber* subscriber_with_custom_qos =
                participant->create_subscriber(custom_qos);
        if (nullptr == subscriber_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a Subscriber with default QoS and a custom Listener.
        // CustomSubscriberListener inherits from SubscriberListener.
        // The value SUBSCRIBER_QOS_DEFAULT is used to denote the default QoS.
        CustomSubscriberListener custom_listener;
        Subscriber* subscriber_with_default_qos_and_custom_listener =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, &custom_listener);
        if (nullptr == subscriber_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_SUBSCRIBER
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Subscriber using a profile and no Listener
        Subscriber* subscriber_with_profile =
                participant->create_subscriber_with_profile("subscriber_profile");
        if (nullptr == subscriber_with_profile)
        {
            // Error
            return;
        }

        // Create a Subscriber using a profile and a custom Listener.
        // CustomSubscriberListener inherits from SubscriberListener.
        CustomSubscriberListener custom_listener;
        Subscriber* subscriber_with_profile_and_custom_listener =
                participant->create_subscriber_with_profile("subscriber_profile", &custom_listener);
        if (nullptr == subscriber_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_SUBSCRIBERQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Subscriber with default SubscriberQos
        Subscriber* subscriber =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        SubscriberQos qos = subscriber->get_qos();

        // Modify QoS attributes
        qos.entity_factory().autoenable_created_entities = false;

        // Assign the new Qos to the object
        subscriber->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_SUBSCRIBERQOS_TO_DEFAULT
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a custom SubscriberQos
        SubscriberQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a subscriber with a custom SubscriberQos
        Subscriber* subscriber = participant->create_subscriber(custom_qos);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // Set the QoS on the subscriber to the default
        if (subscriber->set_qos(SUBSCRIBER_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (subscriber->set_qos(participant->get_default_subscriber_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_SUBSCRIBER
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create a Subscriber
        Subscriber* subscriber =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // Use the Subscriber to communicate
        // (...)

        // Delete the entities the subscriber created
        if (subscriber->delete_contained_entities() != RETCODE_OK)
        {
            // Subscriber failed to delete the entities it created
            return;
        }

        // Delete the Subscriber
        if (participant->delete_subscriber(subscriber) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_SUBSCRIBERQOS
        // Create a DomainParticipant in the desired domain
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        SubscriberQos qos_type1 = participant->get_default_subscriber_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default SubscriberQos
        if (participant->set_default_subscriber_qos(qos_type1) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Subscriber with the new default SubscriberQos.
        Subscriber* subscriber_with_qos_type1 =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        SubscriberQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default SubscriberQos
        if (participant->set_default_subscriber_qos(qos_type2) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Subscriber with the new default SubscriberQos.
        Subscriber* subscriber_with_qos_type2 =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default SubscriberQos to the original default constructed values
        if (participant->set_default_subscriber_qos(SUBSCRIBER_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_subscriber_qos(SubscriberQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }
}

//DDS_DATAREADER_LISTENER_SPECIALIZATION
class CustomDataReaderListener : public DataReaderListener
{

public:

    CustomDataReaderListener()
        : DataReaderListener()
    {
    }

    virtual ~CustomDataReaderListener()
    {
    }

    void on_data_available(
            DataReader* reader) override
    {
        static_cast<void>(reader);
        std::cout << "Received new data message" << std::endl;
    }

    void on_subscription_matched(
            DataReader* reader,
            const SubscriptionMatchedStatus& info) override
    {
        static_cast<void>(reader);
        if (info.current_count_change == 1)
        {
            std::cout << "Matched a remote DataWriter" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Unmatched a remote DataWriter" << std::endl;
        }
    }

    void on_requested_deadline_missed(
            DataReader* reader,
            const RequestedDeadlineMissedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "Some data was not received on time" << std::endl;
    }

    void on_liveliness_changed(
            DataReader* reader,
            const LivelinessChangedStatus& info) override
    {
        static_cast<void>(reader);
        if (info.alive_count_change == 1)
        {
            std::cout << "A matched DataWriter has become active" << std::endl;
        }
        else if (info.not_alive_count_change == 1)
        {
            std::cout << "A matched DataWriter has become inactive" << std::endl;
        }
    }

    void on_sample_rejected(
            DataReader* reader,
            const SampleRejectedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "A received data sample was rejected" << std::endl;
    }

    void on_requested_incompatible_qos(
            DataReader* reader,
            const RequestedIncompatibleQosStatus& info) override
    {
        std::cout << "Found a remote Topic with incompatible QoS (QoS ID: " << info.last_policy_id <<
            ")" << std::endl;
    }

    void on_sample_lost(
            DataReader* reader,
            const SampleLostStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "A data sample was lost and will not be received" << std::endl;
    }

};
//!--

void dds_dataReader_examples()
{
    // Taken out of the examples to avoid bloating them
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    Subscriber* subscriber =
            participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    Topic* topic =
            participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

    {
        //DDS_CREATE_DATAREADER
        // Create a DataReader with default DataReaderQos and no Listener
        // The value DATAREADER_QOS_DEFAULT is used to denote the default QoS.
        DataReader* data_reader_with_default_qos =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader_with_default_qos)
        {
            // Error
            return;
        }

        // A custom DataReaderQos can be provided to the creation method
        DataReaderQos custom_qos;

        // Modify QoS attributes
        // (...)

        DataReader* data_reader_with_custom_qos =
                subscriber->create_datareader(topic, custom_qos);
        if (nullptr == data_reader_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a DataReader with default QoS and a custom Listener.
        // CustomDataReaderListener inherits from DataReaderListener.
        // The value DATAREADER_QOS_DEFAULT is used to denote the default QoS.
        CustomDataReaderListener custom_listener;
        DataReader* data_reader_with_default_qos_and_custom_listener =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT, &custom_listener);
        if (nullptr == data_reader_with_default_qos_and_custom_listener)
        {
            // Error
            return;
        }

        // Create a DataReader with default QoS and a custom TopicQos.
        // The value DATAREADER_QOS_USE_TOPIC_QOS is used to denote the default QoS
        // and to override the TopicQos.
        Topic* topic;
        DataReader* data_reader_with_default_qos_and_custom_topic_qos =
                subscriber->create_datareader(topic, DATAREADER_QOS_USE_TOPIC_QOS);
        if (nullptr == data_reader_with_default_qos_and_custom_topic_qos)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PROFILE_DATAREADER
        // First load the XML with the profiles
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

        // Create a DataReader using a profile and no Listener
        DataReader* data_reader_with_profile =
                subscriber->create_datareader_with_profile(topic, "data_reader_profile");
        if (nullptr == data_reader_with_profile)
        {
            // Error
            return;
        }

        // Create a DataReader using a profile and a custom Listener.
        // CustomDataReaderListener inherits from DataReaderListener.
        CustomDataReaderListener custom_listener;
        DataReader* data_reader_with_profile_and_custom_listener =
                subscriber->create_datareader_with_profile(topic, "data_reader_profile", &custom_listener);
        if (nullptr == data_reader_with_profile_and_custom_listener)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CREATE_PAYLOAD_POOL_DATAREADER
        // A DataReaderQos must be provided to the creation method
        DataReaderQos qos;

        // Create PayloadPool
        std::shared_ptr<CustomPayloadPool> payload_pool = std::make_shared<CustomPayloadPool>();

        DataReader* data_reader = subscriber->create_datareader(topic, qos, nullptr, StatusMask::all(), payload_pool);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DATAREADERQOS
        // Create a DataReader with default DataReaderQos
        DataReader* data_reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DataReaderQos qos = data_reader->get_qos();

        // Modify QoS attributes
        // (...)

        // Assign the new Qos to the object
        data_reader->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_DATAREADERQOS_TO_DEFAULT
        // Create a custom DataReaderQos
        DataReaderQos custom_qos;

        // Modify QoS attributes
        // (...)

        // Create a DataWriter with a custom DataReaderQos
        DataReader* data_reader = subscriber->create_datareader(topic, custom_qos);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }

        // Set the QoS on the DataWriter to the default
        if (data_reader->set_qos(DATAREADER_QOS_DEFAULT) != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (data_reader->set_qos(subscriber->get_default_datareader_qos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_DATAREADER
        // Create a DataReader
        DataReader* data_reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }

        // Use the DataReader to communicate
        // (...)

        // Delete the entities the DataReader created
        if (data_reader->delete_contained_entities() != RETCODE_OK)
        {
            // DataReader failed to delete the entities it created.
            return;
        }

        // Delete the DataReader
        if (subscriber->delete_datareader(data_reader) != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_DATAREADERQOS
        // Get the current QoS or create a new one from scratch
        DataReaderQos qos_type1 = subscriber->get_default_datareader_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default DataReaderQos
        if (subscriber->set_default_datareader_qos(qos_type1) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataReader with the new default DataReaderQos.
        DataReader* data_reader_with_qos_type1 =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DataReaderQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default DataReaderQos
        if (subscriber->set_default_datareader_qos(qos_type2) != RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataReader with the new default DataReaderQos.
        DataReader* data_reader_with_qos_type2 =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default DataReaderQos to the original default constructed values
        if (subscriber->set_default_datareader_qos(DATAREADER_QOS_DEFAULT)
                != RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (subscriber->set_default_datareader_qos(DataReaderQos())
                != RETCODE_OK)
        {
            // Error
            return;
        }
        //!--
    }
    {
        //DDS_DATAREADER_READ_WAITSET
        // Create a DataReader
        DataReader* data_reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }

        // Prepare a wait-set to wait for data on the DataReader
        WaitSet wait_set;
        StatusCondition& condition = data_reader->get_statuscondition();
        condition.set_enabled_statuses(StatusMask::data_available());
        wait_set.attach_condition(condition);

        // Create a data and SampleInfo instance
        Foo data;
        SampleInfo info;

        //Define a timeout of 5 seconds
        eprosima::fastdds::dds::Duration_t timeout (5, 0);

        // Loop reading data as it arrives
        // This will make the current thread to be dedicated exclusively to
        // waiting and reading data until the remote DataWriter dies
        while (true)
        {
            ConditionSeq active_conditions;
            if (RETCODE_OK == wait_set.wait(active_conditions, timeout))
            {
                while (RETCODE_OK == data_reader->take_next_sample(&data, &info))
                {
                    if (info.valid_data)
                    {
                        // Do something with the data
                        std::cout << "Received new data value for topic "
                                  << topic->get_name()
                                  << std::endl;
                    }
                    else
                    {
                        // If the remote writer is not alive, we exit the reading loop
                        std::cout << "Remote writer for topic "
                                  << topic->get_name()
                                  << " is dead" << std::endl;
                        break;
                    }
                }
            }
            else
            {
                std::cout << "No data this time" << std::endl;
            }
        }
        //!--
    }
    {
        //DDS_DATAREADER_WAIT_FOR_UNREAD
        // Create a DataReader
        DataReader* data_reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == data_reader)
        {
            // Error
            return;
        }

        // Create a data and SampleInfo instance
        Foo data;
        SampleInfo info;

        //Define a timeout of 5 seconds
        eprosima::fastdds::dds::Duration_t timeout (5, 0);

        // Loop reading data as it arrives
        // This will make the current thread to be dedicated exclusively to
        // waiting and reading data until the remote DataWriter dies
        while (true)
        {
            if (data_reader->wait_for_unread_message(timeout))
            {
                if (RETCODE_OK == data_reader->take_next_sample(&data, &info))
                {
                    if (info.valid_data)
                    {
                        // Do something with the data
                        std::cout << "Received new data value for topic "
                                  << topic->get_name()
                                  << std::endl;
                    }
                    else
                    {
                        // If the remote writer is not alive, we exit the reading loop
                        std::cout << "Remote writer for topic "
                                  << topic->get_name()
                                  << " is dead" << std::endl;
                        break;
                    }
                }
            }
            else
            {
                std::cout << "No data this time" << std::endl;
            }
        }
        //!--

        {
            //READING-INSTANCE
            if (RETCODE_OK == data_reader->take_next_sample(&data, &info))
            {
                if (info.valid_data)
                {
                    // Data sample has been received
                }
                else if (info.instance_state == NOT_ALIVE_DISPOSED_INSTANCE_STATE)
                {
                    // A remote DataWriter has disposed the instance
                }
                else if (info.instance_state == NOT_ALIVE_NO_WRITERS_INSTANCE_STATE)
                {
                    // None of the matched DataWriters are writing in the instance.
                    // The instance can be safely disposed.
                }
            }
            //!
        }

        {
            //DDS_DATAREADER_LOAN_SEQUENCES
            // Sequences are automatically initialized to be empty (maximum == 0)
            FooSeq data_seq;
            SampleInfoSeq info_seq;

            // with empty sequences, a take() or read() will return loaned
            // sequence elements
            ReturnCode_t ret_code = data_reader->take(data_seq, info_seq,
                            LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                            ANY_VIEW_STATE, ANY_INSTANCE_STATE);

            // process the returned data

            // must return the loaned sequences when done processing
            data_reader->return_loan(data_seq, info_seq);
            //!--
        }

        {
            //DDS_DATAREADER_PROCESS_DATA
            // Sequences are automatically initialized to be empty (maximum == 0)
            FooSeq data_seq;
            SampleInfoSeq info_seq;

            // with empty sequences, a take() or read() will return loaned
            // sequence elements
            ReturnCode_t ret_code = data_reader->take(data_seq, info_seq,
                            LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                            ANY_VIEW_STATE, ANY_INSTANCE_STATE);

            // process the returned data
            if (ret_code == RETCODE_OK)
            {
                // Both info_seq.length() and data_seq.length() will have the number of samples returned
                for (FooSeq::size_type n = 0; n < info_seq.length(); ++n)
                {
                    // Only samples with valid data should be accessed
                    if (info_seq[n].valid_data && data_reader->is_sample_valid(&data_seq[n], &info_seq[n]))
                    {
                        // Do something with data_seq[n]
                    }
                }

                // must return the loaned sequences when done processing
                data_reader->return_loan(data_seq, info_seq);
            }
            //!--
        }
    }
}

//DDS_DATAREADER_READ_LISTENER
class CustomizedDataReaderListener : public DataReaderListener
{

public:

    CustomizedDataReaderListener()
        : DataReaderListener()
    {
    }

    virtual ~CustomizedDataReaderListener()
    {
    }

    void on_data_available(
            DataReader* reader) override
    {
        // Create a data and SampleInfo instance
        Foo data;
        SampleInfo info;

        // Keep taking data until there is nothing to take
        while (reader->take_next_sample(&data, &info) == RETCODE_OK)
        {
            if (info.valid_data)
            {
                // Do something with the data
                std::cout << "Received new data value for topic "
                          << reader->get_topicdescription()->get_name()
                          << std::endl;
            }
            else
            {
                std::cout << "Remote writer for topic "
                          << reader->get_topicdescription()->get_name()
                          << " is dead" << std::endl;
            }
        }
    }

};
//!--

void dds_qos_examples()
{
    // Taken out of the examples to avoid bloating them
    // Note that the following lines are included to make these examples compilable and more readable.
    // In a real application, calling creation functions multiple times on the same pointer should
    // be avoided, as this can lead to memory leaks. It is more robust to use unique_ptrs if necessary.
    DomainParticipantFactory* factory_ = DomainParticipantFactory::get_instance();
    DomainParticipant* participant_ = factory_->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    Subscriber* subscriber_ =
            participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    Publisher* publisher_ =
            participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
    Topic* topic_ =
            participant_->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
    DataReader* reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT);
    DataWriter* writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT);
    uint16_t domain = 7;
    {
        //DDS_CHANGE_DEADLINE_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The DeadlineQosPolicy is constructed with an infinite period by default
        // Change the period to 1 second
        writer_qos.deadline().period.seconds = 1;
        writer_qos.deadline().period.nanosec = 0;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_DURABILITY_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The DurabilityQosPolicy is constructed with kind = VOLATILE_DURABILITY_QOS by default
        // Change the kind to TRANSIENT_LOCAL_DURABILITY_QOS
        writer_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_ENTITY_FACTORY_QOS_POLICY
        // This example uses a Participant, but it can also be applied to
        // DomainParticipantFactory, Publisher and Subscriber entities
        DomainParticipantQos participant_qos;
        // The EntityFactoryQosPolicy is constructed with autoenable_created_entities = true by default
        // Change it to false
        participant_qos.entity_factory().autoenable_created_entities = false;
        // Use modified QoS in the creation of the corresponding entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }

    {
        //DDS_CHANGE_GROUP_DATA_QOS_POLICY
        // This example uses a Publisher, but it can also be applied to Subscriber entities
        PublisherQos publisher_qos;
        // The GroupDataQosPolicy is constructed with an empty collection by default
        // Collection is a private member so you need to use getters and setters to access

        // Add data to the collection in initialization
        std::vector<eprosima::fastdds::rtps::octet> vec;
        // Add two new octets to group data vector
        eprosima::fastdds::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        publisher_qos.group_data().data_vec(vec); // Setter function
        // Use modified QoS in the creation of the corresponding entity
        publisher_ = participant_->create_publisher(publisher_qos);

        // Add data to the collection at runtime
        vec = publisher_qos.group_data().data_vec(); // Getter to keep old values
        val = 31;
        vec.push_back(val);
        publisher_qos.group_data().data_vec(vec); // Setter function
        // Update the QoS in the corresponding entity
        publisher_->set_qos(publisher_qos);
        //!--
    }

    {
        //DDS_CHANGE_HISTORY_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The HistoryQosPolicy is constructed with kind = KEEP_LAST and depth = 1 by default
        // It is possible to adjust the depth and keep the kind as KEEP_LAST
        writer_qos.history().depth = 20;
        // Or you can also change the kind to KEEP_ALL (depth will not be used).
        writer_qos.history().kind = KEEP_ALL_HISTORY_QOS;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_LIFESPAN_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The LifespanQosPolicy is constructed with duration set to infinite by default
        // Change the duration to 5 s
        writer_qos.lifespan().duration = {5, 0};
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_LIVELINESS_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The LivelinessQosPolicy is constructed with kind = AUTOMATIC by default
        // Change the kind to MANUAL_BY_PARTICIPANT
        writer_qos.liveliness().kind = MANUAL_BY_PARTICIPANT_LIVELINESS_QOS;
        // The LivelinessQosPolicy is constructed with lease_duration set to infinite by default
        // Change the lease_duration to 1 second
        writer_qos.liveliness().lease_duration = {1, 0};
        // The LivelinessQosPolicy is constructed with announcement_period set to infinite by default
        // Change the announcement_period to 1 ms
        writer_qos.liveliness().announcement_period = {0, 1000000};
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_OWNERSHIP_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The OwnershipQosPolicy is constructed with kind = SHARED by default
        // Change the kind to EXCLUSIVE
        writer_qos.ownership().kind = EXCLUSIVE_OWNERSHIP_QOS;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_OWNERSHIP_STRENGTH_QOS_POLICY
        // This example only applies to DataWriter entities
        DataWriterQos writer_qos;
        // The OwnershipStrengthQosPolicy is constructed with value 0 by default
        // Change the strength to 10
        writer_qos.ownership_strength().value = 10;
        // Use modified QoS in the creation of the corresponding DataWriter
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_PARTITION_QOS_POLICY
        // This example uses a Publisher, but it can also be applied to Subscriber entities
        PublisherQos publisher_qos;
        // The PartitionsQosPolicy is constructed with max_size = 0 by default
        // Max_size is a private member so you need to use getters and setters to access
        // Change the max_size to 20
        publisher_qos.partition().set_max_size(20); // Setter function
        // The PartitionsQosPolicy is constructed with an empty list of partitions by default
        // Partitions is a private member so you need to use getters and setters to access

        // Add new partitions in initialization
        std::vector<std::string> part;
        part.push_back("part1");
        part.push_back("part2");
        publisher_qos.partition().names(part); // Setter function
        // Use modified QoS in the creation of the corresponding entity
        publisher_ = participant_->create_publisher(publisher_qos);

        // Add data to the collection at runtime
        part = publisher_qos.partition().names(); // Getter to keep old values
        part.push_back("part3");
        publisher_qos.partition().names(part); // Setter function
        // Update the QoS in the corresponding entity
        publisher_->set_qos(publisher_qos);
        //!--
    }

    {
        //DDS_CHANGE_RELIABILITY_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The ReliabilityQosPolicy is constructed with kind = BEST_EFFORT by default
        // Change the kind to RELIABLE
        writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        // The ReliabilityQosPolicy is constructed with max_blocking_time = 100ms by default
        // Change the max_blocking_time to 1s
        writer_qos.reliability().max_blocking_time = {1, 0};
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_RESOURCE_LIMITS_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader and Topic entities
        DataWriterQos writer_qos;
        // The ResourceLimitsQosPolicy is constructed with max_samples = 5000 by default
        // Change max_samples to 200
        writer_qos.resource_limits().max_samples = 200;
        // The ResourceLimitsQosPolicy is constructed with max_instances = 10 by default
        // Change max_instances to 20
        writer_qos.resource_limits().max_instances = 20;
        // The ResourceLimitsQosPolicy is constructed with max_samples_per_instance = 400 by default
        // Change max_samples_per_instance to 100 as it must be lower than max_samples
        writer_qos.resource_limits().max_samples_per_instance = 100;
        // The ResourceLimitsQosPolicy is constructed with allocated_samples = 100 by default
        // Change allocated_samples to 50
        writer_qos.resource_limits().allocated_samples = 50;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_TOPIC_DATA_QOS_POLICY
        // This example only applies to Topic entities
        TopicQos topic_qos;
        // The TopicDataQosPolicy is constructed with an empty vector by default
        std::vector<eprosima::fastdds::rtps::octet> vec;
        // Add two new octets to topic data vector
        eprosima::fastdds::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        topic_qos.topic_data().data_vec(vec); // Setter Function
        // Use modified QoS in the creation of the corresponding Topic
        topic_ = participant_->create_topic("<topic_name>", "<type_name>", topic_qos);
        //!--
    }

    {
        //DDS_CHANGE_USER_DATA_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DomainParticipant and DataReader entities
        DataWriterQos writer_qos;
        std::vector<eprosima::fastdds::rtps::octet> vec;
        // Add two new octets to user data vector
        eprosima::fastdds::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        writer_qos.user_data().data_vec(vec); // Setter Function
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_DATASHARING_QOS_POLICY
        // This example uses a DataWriter, but it can also be applied to DataReader entities
        DataWriterQos writer_qos;

        // DataSharing is set to AUTO by default, which means that DataSharing will be used if the topic
        // is compatible. If no Shared Memory directory is specified, the default one will be used.

        // Configure DataSharing to ON to enable DataSharing and specify the shared memory directory (mandatory)
        writer_qos.data_sharing().on("/path/to/shared_memory/directory");
        // Alternatively, configure DataSharing to OFF to disable it
        writer_qos.data_sharing().off();

        // Configure the DataSharing as AUTO with two user-defined IDs (can also be used with ON)
        std::vector<uint16_t> ids;
        ids.push_back(0x1234);
        ids.push_back(0xABCD);
        writer_qos.data_sharing().automatic(ids);

        // Alternatively, add them individually
        writer_qos.data_sharing().add_domain_id(uint16_t(0x1234));
        writer_qos.data_sharing().add_domain_id(uint16_t(0xABCD));
        // Or you can leave the IDs empty and the system will automatically create a
        // unique ID for the current machine

        // Set the maximum number of domains to 5. Setting 0 means 'unlimited'
        writer_qos.data_sharing().set_max_domains(5);

        // [OPTIONAL] ThreadSettings for listening thread
        writer_qos.data_sharing().data_sharing_listener_thread(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});

        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_DISABLE_POSITIVE_ACKS_QOS_POLICY
        // This Qos has different API for DataWriter and DataReader entities, because it is accesed
        // through the ReliableWriterQos and ReliableReaderQos respectively.
        // For further details see RTPSReliableWriterQos & RTPSReliableReaderQos sections.
        DataWriterQos writer_qos;
        DataReaderQos reader_qos;
        // The DisablePositiveACKsQosPolicy is constructed with enabled = false by default
        // Change enabled to true
        writer_qos.reliable_writer_qos().disable_positive_acks.enabled = true;
        reader_qos.reliable_reader_qos().disable_positive_acks.enabled = true;
        // The DisablePositiveACKsQosPolicy is constructed with infinite duration by default
        // Change the duration to 1 second
        writer_qos.reliable_writer_qos().disable_positive_acks.duration = {1, 0};
        reader_qos.reliable_reader_qos().disable_positive_acks.duration = {1, 0};

        // Exclusively, DataWriters can additionally disable the heartbeat piggyback
        writer_qos.reliable_writer_qos().disable_heartbeat_piggyback = true;

        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        reader_ = subscriber_->create_datareader(topic_, reader_qos);
        //!--
    }

    {
        //DDS_CHANGE_RTPS_RELIABLE_WRITER_QOS
        // This example only applies to DataWriter entities
        DataWriterQos writer_qos;
        // The RTPSReliableWriterQos is constructed with initial_heartbeat_delay = 12 ms by default
        // Change the initial_heartbeat_delay to 20 nanoseconds
        writer_qos.reliable_writer_qos().times.initial_heartbeat_delay = {0, 20};
        // The RTPSReliableWriterQos is constructed with heartbeat_period = 3 s by default
        // Change the heartbeat_period to 5 seconds
        writer_qos.reliable_writer_qos().times.heartbeat_period = {5, 0};
        // The RTPSReliableWriterQos is constructed with nack_response_delay = 5 ms by default
        // Change the nack_response_delay to 10 nanoseconds
        writer_qos.reliable_writer_qos().times.nack_response_delay = {0, 10};
        // The RTPSReliableWriterQos is constructed with nack_supression_duration = 0 s by default
        // Change the nack_supression_duration to 20 nanoseconds
        writer_qos.reliable_writer_qos().times.nack_supression_duration = {0, 20};
        // You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
        writer_qos.reliable_writer_qos().disable_positive_acks.enabled = true;
        // The RTPSReliableWriterQos is constructed with disable_heartbeat_piggyback = false by default
        // Disable the heartbeat piggyback mechanism.
        writer_qos.reliable_writer_qos().disable_heartbeat_piggyback = true;
        // Use modified QoS in the creation of the DataWriter entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_RTPS_RELIABLE_READER_QOS
        // This example only applies to DataReader entities
        DataReaderQos reader_qos;
        // The RTPSReliableReaderQos is constructed with initial_acknack_delay = 70 ms by default
        // Change the initialAcknackDelay to 70 nanoseconds
        reader_qos.reliable_reader_qos().times.initial_acknack_delay = {0, 70};
        // The RTPSReliableReaderQos is constructed with heartbeat_response_delay = 5 ms by default
        // Change the heartbeatResponseDelay to 5 nanoseconds
        reader_qos.reliable_reader_qos().times.heartbeat_response_delay = {0, 5};
        // You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
        reader_qos.reliable_reader_qos().disable_positive_acks.enabled = true;
        // Use modified QoS in the creation of the DataReader entity
        reader_ = subscriber_->create_datareader(topic_, reader_qos);
        //!--
    }

    {
        //DDS_CHANGE_PARTICIPANT_RESOURCE_LIMITS_QOS_POLICY
        // This example only applies to DomainParticipant entities
        DomainParticipantQos participant_qos;
        // Set the maximum size of participant resource limits collection to 3 and it allocation configuration to fixed size
        participant_qos.allocation().participants =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(
            3u);
        // Set the maximum size of reader's resource limits collection to 2 and its allocation configuration to fixed size
        participant_qos.allocation().readers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        // Set the maximum size of writer's resource limits collection to 1 and its allocation configuration to fixed size
        participant_qos.allocation().writers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        // Set the maximum size of the partition data to 256
        participant_qos.allocation().data_limits.max_partitions = 256u;
        // Set the maximum size of the user data to 256
        participant_qos.allocation().data_limits.max_user_data = 256u;
        // Set the maximum size of the properties data to 512
        participant_qos.allocation().data_limits.max_properties = 512u;
        // Set the preallocated filter expression size to 512
        participant_qos.allocation().content_filter.expression_initial_size = 512u;
        // Set the maximum number of expression parameters to 4 and its allocation configuration to fixed size
        participant_qos.allocation().content_filter.expression_parameters =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
        // Use modified QoS in the creation of the corresponding DomainParticipant
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }

    {
        //DDS_CHANGE_PROPERTY_POLICY_QOS
        // This example uses a DataWriter, but it can also be applied to DomainParticipant and DataReader entities
        DataWriterQos writer_qos;
        // Add new property for the Auth:PKI-DH plugin
        writer_qos.properties().properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");
        // Add new property for the Access:Permissions plugin
        writer_qos.properties().properties().emplace_back(eprosima::fastdds::rtps::Property("dds.sec.access.plugin",
                "builtin.Access-Permissions"));

        // Add new user custom property to send to external Participants
        writer_qos.properties().properties().emplace_back("Custom Property Name", "Custom value", true);
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_PUBLISH_MODE_QOS
        // This example only applies to DataWriter entities
        DataWriterQos writer_qos;
        // The PublishModeQosPolicy is constructed with kind = SYNCHRONOUS by default
        // Change the kind to ASYNCHRONOUS
        writer_qos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
        // Use modified QoS in the creation of the DataWriter entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_READER_RESOURCE_LIMITS_QOS
        // This example only applies to DataReader entities
        DataReaderQos reader_qos;
        // Set the maximum size for writer matched resource limits collection to 1 and its allocation configuration to fixed size
        reader_qos.reader_resource_limits().matched_publisher_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        // Set the maximum size for sample info resource limits to 22 and its allocation configuration to fixed size
        reader_qos.reader_resource_limits().sample_infos_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(22u);
        // Set the maximum size for writer matched resource limits collection to 3 and its allocation configuration to fixed size
        reader_qos.reader_resource_limits().outstanding_reads_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        reader_qos.reader_resource_limits().max_samples_per_read = 42;
        // Use modified QoS in the creation of the DataReader entity
        reader_ = subscriber_->create_datareader(topic_, reader_qos);
        //!--
    }

    {
        //DDS_CHANGE_WRITER_RESOURCE_LIMITS_QOS
        // This example only applies to DataWriter entities
        DataWriterQos writer_qos;
        // Set the maximum size for reader matched resource limits collection to 3 and its allocation configuration to fixed size
        writer_qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // Set the maximum number of writer side content filters to 1 and its allocation configuration to fixed size
        writer_qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        // Use modified QoS in the creation of the DataWriter entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_RTPS_ENDPOINT_QOS
        // This example uses a DataWriter, but it can also be applied to DataReader entities
        DataWriterQos writer_qos;
        // Add new unicast locator with port 7800
        eprosima::fastdds::rtps::Locator_t new_unicast_locator;
        new_unicast_locator.port = 7800;
        writer_qos.endpoint().unicast_locator_list.push_back(new_unicast_locator);
        // Add new multicast locator with IP 239.255.0.4 and port 7900
        eprosima::fastdds::rtps::Locator_t new_multicast_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
        new_multicast_locator.port = 7900;
        writer_qos.endpoint().multicast_locator_list.push_back(new_multicast_locator);
        // Add an external locator with IP 100.100.100.10, port 12345, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask external_locator;
        external_locator.kind = LOCATOR_KIND_UDPv4;
        external_locator.port = 12345;
        external_locator.mask(24);
        writer_qos.endpoint().external_unicast_locators[1][0].push_back(external_locator);
        // Drop non matching locators
        writer_qos.endpoint().ignore_non_matching_locators = true;
        // Set 3 as user defined id
        writer_qos.endpoint().user_defined_id = 3;
        // Set 4 as entity id
        writer_qos.endpoint().entity_id = 4;
        // The RTPSWriterQos is constructed with history_memory_policy = PREALLOCATED by default
        // Change the history_memory_policy to DYNAMIC_RESERVE
        writer_qos.endpoint().history_memory_policy = eprosima::fastdds::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        using namespace eprosima::fastdds::rtps;
        // For port thread settings
        class CustomPortBasedTransportDescriptor : public PortBasedTransportDescriptor
        {
        public:

            CustomPortBasedTransportDescriptor()
                : PortBasedTransportDescriptor(0, 0)
            {
            }

            TransportInterface* create_transport() const override
            {
                return nullptr;
            }

            //! Returns the minimum size required for a send operation.
            uint32_t min_send_buffer_size() const override
            {
                return 0;
            }

        };
        //DDS_CHANGE_THREAD_SETTINGS
        // This example uses a specific thread of the DomainParticipantQos, but it can also be applied to most of
        // the threads used by FastDDS in DomainParticipantFactoryQos, DomainParticipantQos or DataSharingQosPolicy.
        // Each thread has its own getter and setter function. See Fast DDS documentation for further details.
        DomainParticipantQos participant_qos;
        participant_qos.timed_events_thread().scheduling_policy = 2;
        participant_qos.timed_events_thread().priority = 10;
        participant_qos.timed_events_thread().affinity = 4;
        participant_qos.timed_events_thread().stack_size = 2000;

        // Use modified QoS in the creation of the corresponding entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--

        //DDS_RECEPTION_THREADS_SETTINGS
        // Implement a Custom class, derived from PortBasedTransportDescriptor, to set the reception threads configuration
        CustomPortBasedTransportDescriptor descriptor;
        PortBasedTransportDescriptor::ReceptionThreadsConfigMap reception_threads_config;
        reception_threads_config[20000].scheduling_policy = 1;
        reception_threads_config[20000].priority = 30;
        reception_threads_config[20000].affinity = 2;
        reception_threads_config[20000].stack_size = 1024;
        reception_threads_config[20001].scheduling_policy = 2;
        reception_threads_config[20001].priority = 10;
        reception_threads_config[20001].affinity = 6;
        reception_threads_config[20001].stack_size = 4096;

        // Use setter to set the reception threads configuration
        descriptor.reception_threads(reception_threads_config);
        //!--
    }

    {
        //DDS_CHANGE_TRANSPORT_CONFIG_QOS
        // This example only applies to DomainParticipant entities
        DomainParticipantQos participant_qos;
        // Add new transport to the list of user transports
        std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        descriptor->sendBufferSize = 9126;
        descriptor->receiveBufferSize = 9126;
        participant_qos.transport().user_transports.push_back(descriptor);
        // Set use_builtin_transports to false
        participant_qos.transport().use_builtin_transports = false;
        // [OPTIONAL] Set ThreadSettings for the builtin transports reception threads
        participant_qos.transport().builtin_transports_reception_threads_ =
                eprosima::fastdds::rtps::ThreadSettings{2, 2, 2, 2};
        // Set max_msg_size_no_frag to a value > 65500 KB
        participant_qos.transport().max_msg_size_no_frag = 70000;
        // Configure netmask filter
        participant_qos.transport().netmask_filter = eprosima::fastdds::rtps::NetmaskFilterKind::ON;
        // Use modified QoS in the creation of the DomainParticipant entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }

    {
        //DDS_CHANGE_WIRE_PROTOCOL_CONFIG_QOS
        // This example only applies to DomainParticipant entities
        DomainParticipantQos participant_qos;
        // Set the guid prefix
        std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> participant_qos.wire_protocol().prefix;
        // Manually set the participantId
        participant_qos.wire_protocol().participant_id = 11;
        // Configure Builtin Attributes
        participant_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                eprosima::fastdds::rtps::DiscoveryProtocol::SERVER;
        // Add locator to unicast list
        eprosima::fastdds::rtps::Locator_t server_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
        server_locator.port = 56542;
        participant_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);
        // Add a metatraffic external locator with IP 100.100.100.10, port 34567, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask meta_external_locator;
        meta_external_locator.kind = LOCATOR_KIND_UDPv4;
        meta_external_locator.port = 34567;
        meta_external_locator.mask(24);
        participant_qos.wire_protocol().builtin.metatraffic_external_unicast_locators[1][0].push_back(
            meta_external_locator);
        // Add locator to default unicast locator list
        eprosima::fastdds::rtps::Locator_t unicast_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(unicast_locator, 192, 168, 1, 41);
        unicast_locator.port = 7400;
        participant_qos.wire_protocol().default_unicast_locator_list.push_back(unicast_locator);
        // Add locator to default multicast locator list
        eprosima::fastdds::rtps::Locator_t multicast_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(multicast_locator, 192, 168, 1, 41);
        multicast_locator.port = 7400;
        participant_qos.wire_protocol().default_multicast_locator_list.push_back(multicast_locator);
        // Add a default external locator with IP 100.100.100.10, port 23456, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask external_locator;
        external_locator.kind = LOCATOR_KIND_UDPv4;
        external_locator.port = 23456;
        external_locator.mask(24);
        participant_qos.wire_protocol().default_external_unicast_locators[1][0].push_back(external_locator);
        // Drop non matching locators
        participant_qos.wire_protocol().ignore_non_matching_locators = true;
        // Increase mutation tries
        participant_qos.wire_protocol().builtin.mutation_tries = 300u;
        // Use modified QoS in the creation of the DomainParticipant entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }

    {
        //DDS_CHANGE_DATA_REPRESENTATION_QOS
        // This example uses a DataWriter, but it can also be applied to Topic entities.
        // DataRepresentationQosPolicy of DataReaders is contained in the TypeConsistencyQos
        // (for further details see TypeConsistencyQos section).
        DataWriterQos writer_qos;
        // Add XCDR v1 data representation to the list of valid representations
        writer_qos.representation().m_value.push_back(DataRepresentationId_t::XCDR_DATA_REPRESENTATION);
        // Add XML data representation to the list of valid representations
        writer_qos.representation().m_value.push_back(DataRepresentationId_t::XML_DATA_REPRESENTATION);
        // Use modified QoS in the creation of the corresponding entity
        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        //!--
    }

    {
        //DDS_CHANGE_TYPE_CONSISTENCY_ENFORCEMENT_QOS
        // This example only applies to DataReader entities
        DataReaderQos reader_qos;
        // The TypeConsistencyEnforcementQosPolicy is constructed with kind = ALLOW_TYPE_COERCION by default
        // Change the kind to DISALLOW_TYPE_COERCION
        reader_qos.type_consistency().m_kind = TypeConsistencyKind::DISALLOW_TYPE_COERCION;
        //Configures the system to ignore the sequence sizes in assignations
        reader_qos.type_consistency().m_ignore_sequence_bounds = true;
        //Configures the system to ignore the string sizes in assignations
        reader_qos.type_consistency().m_ignore_string_bounds = true;
        //Configures the system to ignore the member names. Members with same ID could have different names
        reader_qos.type_consistency().m_ignore_member_names = true;
        //Configures the system to allow type widening
        reader_qos.type_consistency().m_prevent_type_widening = false;
        //Configures the system to not use the complete Type Information in entities match process
        reader_qos.type_consistency().m_force_type_validation = false;
        // Use modified QoS in the creation of the corresponding entity
        reader_ = subscriber_->create_datareader(topic_, reader_qos);
        //!--
    }

    {
        //DDS_QOS_POLICY_COUNT_SEQ
        DataReader* reader_ =
                subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT);

        // Get how many times ReliabilityQosPolicy was not compatible with a remote writer
        RequestedIncompatibleQosStatus status;
        reader_->get_requested_incompatible_qos_status(status);
        uint32_t incompatible_reliability_count = status.policies[RELIABILITY_QOS_POLICY_ID].count;
        //!--
    }

    {
        //CONF_GUIDPREFIX_OPTION_1
        eprosima::fastdds::rtps::GuidPrefix_t guid_prefix;
        guid_prefix.value[0] = eprosima::fastdds::rtps::octet(0x77);
        guid_prefix.value[1] = eprosima::fastdds::rtps::octet(0x73);
        guid_prefix.value[2] = eprosima::fastdds::rtps::octet(0x71);
        guid_prefix.value[3] = eprosima::fastdds::rtps::octet(0x85);
        guid_prefix.value[4] = eprosima::fastdds::rtps::octet(0x69);
        guid_prefix.value[5] = eprosima::fastdds::rtps::octet(0x76);
        guid_prefix.value[6] = eprosima::fastdds::rtps::octet(0x95);
        guid_prefix.value[7] = eprosima::fastdds::rtps::octet(0x66);
        guid_prefix.value[8] = eprosima::fastdds::rtps::octet(0x65);
        guid_prefix.value[9] = eprosima::fastdds::rtps::octet(0x82);
        guid_prefix.value[10] = eprosima::fastdds::rtps::octet(0x82);
        guid_prefix.value[11] = eprosima::fastdds::rtps::octet(0x79);

        DomainParticipantQos participant_qos;
        participant_qos.wire_protocol().prefix = guid_prefix;
        // Use modified QoS in the creation of the DomainParticipant entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }

    {
        //CONF_GUIDPREFIX_OPTION_2
        DomainParticipantQos participant_qos;
        std::istringstream("77.73.71.85.69.76.95.66.65.82.82.79") >> participant_qos.wire_protocol().prefix;
        // Use modified QoS in the creation of the DomainParticipant entity
        participant_ = factory_->create_participant(domain, participant_qos);
        //!--
    }
}

void log_examples()
{
    //LOG_MESSAGES
    EPROSIMA_LOG_INFO(DOCUMENTATION_CATEGORY, "This is an info message");
    EPROSIMA_LOG_WARNING(DOCUMENTATION_CATEGORY, "This is an warning message");
    EPROSIMA_LOG_ERROR(DOCUMENTATION_CATEGORY, "This is an error message");
    //!--

    //LOG_SET_GET_VERBOSITY
    // Set log verbosity level to Log::Kind::Info
    Log::SetVerbosity(Log::Kind::Info);

    // Get log verbosity level
    Log::Kind verbosity_level = Log::GetVerbosity();
    //!--

    //LOG_REPORT_FILENAMES
    // Enable file name and line number reporting
    Log::ReportFilenames(true);

    // Disable file name and line number reporting
    Log::ReportFilenames(false);
    //!--

    //LOG_REPORT_FUNCTIONS
    // Enable function name reporting
    Log::ReportFunctions(true);

    // Disable function name reporting
    Log::ReportFunctions(false);
    //!--

    //LOG_CATEGORY_FILTER
    // Set filter using regular expression
    Log::SetCategoryFilter(std::regex("(CATEGORY_1)|(CATEGORY_2)"));

    // Would be consumed
    EPROSIMA_LOG_ERROR(CATEGORY_1, "First log entry");
    // Would be consumed
    EPROSIMA_LOG_ERROR(CATEGORY_2, "Second log entry");
    // Would NOT be consumed
    EPROSIMA_LOG_ERROR(CATEGORY_3, "Third log entry");
    //!--

    //LOG_FILENAME_FILTER
    // Filename: example.cpp

    // Enable file name and line number reporting
    Log::ReportFilenames(true);

    // Set filter using regular expression so filename must match "example"
    Log::SetFilenameFilter(std::regex("example"));
    // Would be consumed
    EPROSIMA_LOG_ERROR(CATEGORY, "First log entry");

    // Set filter using regular expression so filename must match "other"
    Log::SetFilenameFilter(std::regex("other"));
    // Would NOT be consumed
    EPROSIMA_LOG_ERROR(CATEGORY, "Second log entry");
    //!--

    //LOG_CONTENT_FILTER
    // Set filter using regular expression so message component must match "First"
    Log::SetErrorStringFilter(std::regex("First"));
    // Would be consumed
    EPROSIMA_LOG_ERROR(CATEGORY, "First log entry");
    // Would NOT be consumed
    EPROSIMA_LOG_ERROR(CATEGORY, "Second log entry");
    //!--

    //LOG_REGISTER_CONSUMER
    // Create a FileConsumer consumer that logs entries in "archive.log"
    std::unique_ptr<FileConsumer> file_consumer(new FileConsumer("archive.log"));
    // Register the consumer. Log entries will be logged to STDOUT and "archive.log"
    Log::RegisterConsumer(std::move(file_consumer));
    //!--

    //LOG_CLEAR_CONSUMERS
    // Clear all the consumers. Log entries are discarded upon consumption.
    Log::ClearConsumers();
    //!--

    //LOG_STDOUT_CONSUMER
    // Create a StdoutConsumer consumer that logs entries to stdout stream.
    std::unique_ptr<StdoutConsumer> stdout_consumer(new StdoutConsumer());

    // Register the consumer.
    Log::RegisterConsumer(std::move(stdout_consumer));
    //!--

    //LOG_STDOUTERR_CONSUMER
    // Create a StdoutErrConsumer consumer that logs entries to stderr only when the Log::Kind is equal to ERROR
    std::unique_ptr<StdoutErrConsumer> stdouterr_consumer(new StdoutErrConsumer());
    stdouterr_consumer->stderr_threshold(Log::Kind::Error);

    // Register the consumer
    Log::RegisterConsumer(std::move(stdouterr_consumer));
    //!--

    //LOG_FILE_CONSUMER
    // Create a FileConsumer consumer that logs entries in "archive_1.log", opening the file in "write" mode.
    std::unique_ptr<FileConsumer> write_file_consumer(new FileConsumer("archive_1.log", false));

    // Create a FileConsumer consumer that logs entries in "archive_2.log", opening the file in "append" mode.
    std::unique_ptr<FileConsumer> append_file_consumer(new FileConsumer("archive_2.log", true));

    // Register the consumers.
    Log::RegisterConsumer(std::move(write_file_consumer));
    Log::RegisterConsumer(std::move(append_file_consumer));
    //!--

    //LOG_FLUSH_AND_KILL
    // Block current thread until the log queue is empty.
    Log::Flush();

    // Stop the loggin thread and free its resources.
    Log::KillThread();

    // Configure ThreadSettings for the logging thread
    Log::SetThreadConfig(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
    //!--

}

void dynamictypes_examples()
{
    {
        //!--CPP_PRIMITIVES
        // Define a struct type with various primitive members
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("PrimitivesStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};

        // Define and add primitive members to the struct
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("my_bool");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_octet");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BYTE));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_char");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_wchar");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_long");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_ulong");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT32));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_int8");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT8));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_uint8");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT8));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_short");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_ushort");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT16));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_longlong");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT64));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_ulonglong");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT64));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_float");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT32));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_double");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT64));
        struct_builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_longdouble");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT128));
        struct_builder->add_member(member_descriptor);

        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for a member of type int32_t
        int32_t in_value {2};
        int32_t out_value {0};
        data->set_int32_value(data->get_member_id_by_name("my_long"), in_value);
        data->get_int32_value(out_value, data->get_member_id_by_name("my_long"));
        //!--
    }
    {
        //!--CPP_STRINGS
        // Define a struct type to contain the strings
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("StringsStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};

        // Define and add string members to the struct
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};

        member_descriptor->name("my_string");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        create_string_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_STRING8);
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
            type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
            member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
         */

        struct_builder->add_member(member_descriptor);
        member_descriptor->name("my_wstring");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        create_wstring_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_STRING16);
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
            type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
            member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
         */

        struct_builder->add_member(member_descriptor);
        member_descriptor->name("my_bounded_string");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        create_string_type(41925)->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_STRING8);
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
            type_descriptor->bound().push_back(41925);
            member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
         */

        struct_builder->add_member(member_descriptor);
        member_descriptor->name("my_bounded_wstring");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        create_wstring_type(20925)->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_STRING16);
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
            type_descriptor->bound().push_back(20925);
            member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
         */

        struct_builder->add_member(member_descriptor);

        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for a string member
        std::string in_value {"helloworld"};
        std::string out_value;
        data->set_string_value(data->get_member_id_by_name("my_string"), in_value);
        data->get_string_value(out_value, data->get_member_id_by_name("my_string"));
        //!--
    }
    {
        //!--CPP_ENUM
        enum MyEnum : int32_t
        {
            A,
            B,
            C
        };

        // Define a struct type to contain an enum
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("EnumStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};
        // Define the enum type
        TypeDescriptor::_ref_type enum_type_descriptor {traits<TypeDescriptor>::make_shared()};
        enum_type_descriptor->kind(TK_ENUM);
        enum_type_descriptor->name("MyEnum");
        DynamicTypeBuilder::_ref_type enum_builder {DynamicTypeBuilderFactory::get_instance()->
                                                            create_type(enum_type_descriptor)};
        // Add enum literals to the enum type
        MemberDescriptor::_ref_type enum_member_descriptor {traits<MemberDescriptor>::make_shared()};
        enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
        enum_member_descriptor->name("A");
        enum_builder->add_member(enum_member_descriptor);
        enum_member_descriptor = traits<MemberDescriptor>::make_shared();
        enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
        enum_member_descriptor->name("B");
        enum_builder->add_member(enum_member_descriptor);
        enum_member_descriptor = traits<MemberDescriptor>::make_shared();
        enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
        enum_member_descriptor->name("C");
        enum_builder->add_member(enum_member_descriptor);
        // Build the enum type
        DynamicType::_ref_type enum_type = enum_builder->build();

        // Add an enum member to the struct
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("my_enum");
        member_descriptor->type(enum_type);
        struct_builder->add_member(member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for an enum member
        MyEnum in_value {MyEnum::C};

        /* Alternative
            uint32_t in_value {2}; // Selecting MyEnum::C
         */

        uint32_t out_value {0};
        data->set_uint32_value(data->get_member_id_by_name("my_enum"), in_value);
        data->get_uint32_value(out_value, data->get_member_id_by_name("my_enum"));
        //!--
    }
    {
        //!--CPP_BITMASK
        // Define a struct type to contain a bitmask
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("BitmaskStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};

        // Define the bitmask type
        DynamicTypeBuilder::_ref_type bitmask_builder {DynamicTypeBuilderFactory::get_instance()->create_bitmask_type(
                                                           8)};

        /* Alternative
            TypeDescriptor::_ref_type bitmask_type_descriptor {traits<TypeDescriptor>::make_shared()};
            bitmask_type_descriptor->kind(TK_BITMASK);
            bitmask_type_descriptor->name("MyBitMask");
            bitmask_type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(
                        TK_BOOLEAN));
            bitmask_type_descriptor->bound().push_back(8);
            DynamicTypeBuilder::_ref_type bitmask_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                            bitmask_type_descriptor)};
         */

        // Add bitfield members to the bitmask type
        MemberDescriptor::_ref_type bitfield_member_descriptor {traits<MemberDescriptor>::make_shared()};
        bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
        bitfield_member_descriptor->name("flag0");
        bitfield_member_descriptor->id(0);
        bitmask_builder->add_member(bitfield_member_descriptor);
        bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
        bitfield_member_descriptor->name("flag1");
        bitmask_builder->add_member(bitfield_member_descriptor);
        bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
        bitfield_member_descriptor->name("flag2");
        bitmask_builder->add_member(bitfield_member_descriptor);
        bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
        bitfield_member_descriptor->name("flag5");
        bitfield_member_descriptor->id(5);
        bitmask_builder->add_member(bitfield_member_descriptor);
        // Build the bitmask type
        DynamicType::_ref_type bitmask_type =  bitmask_builder->build();

        // Add a bitmask member to the struct
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("my_bitmask");
        member_descriptor->type(bitmask_type);
        struct_builder->add_member(member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for bitmask member.
        uint8_t in_value {3}; // Setting both "flag0" and "flag1" simultaneously.
        uint8_t out_value {0};
        data->set_uint8_value(data->get_member_id_by_name("my_bitmask"), in_value);
        data->get_uint8_value(out_value, data->get_member_id_by_name("my_bitmask"));

        // Set and retrieve specific bitflag
        bool in_bitflag_value = true;
        bool out_bitflag_value = false;
        DynamicData::_ref_type bitmask_data = data->loan_value(data->get_member_id_by_name("my_bitmask"));
        bitmask_data->set_boolean_value(bitmask_data->get_member_id_by_name("flag5"), in_bitflag_value);
        bitmask_data->get_boolean_value(out_bitflag_value, bitmask_data->get_member_id_by_name("flag5"));
        //!--
    }
    {
        // Type created as described in enumeration type section.
        enum MyEnum : int32_t
        {
            A,
            B,
            C
        };
        DynamicType::_ref_type enum_type;

        //!--CPP_TYPEDEF
        // Define a struct type to contain the alias
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("AliasStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};

        // Define an alias type for the enum
        TypeDescriptor::_ref_type aliasenum_type_descriptor {traits<TypeDescriptor>::make_shared()};
        aliasenum_type_descriptor->kind(TK_ALIAS);
        aliasenum_type_descriptor->name("MyAliasedEnum");
        aliasenum_type_descriptor->base_type(enum_type);
        DynamicTypeBuilder::_ref_type aliasenum_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                 create_type(aliasenum_type_descriptor)};
        // Build the alias type
        DynamicType::_ref_type aliasenum_type = aliasenum_builder->build();

        // Define an alias type for a bounded string
        TypeDescriptor::_ref_type alias_bounded_string_type_descriptor {traits<TypeDescriptor>::make_shared()};
        alias_bounded_string_type_descriptor->kind(TK_ALIAS);
        alias_bounded_string_type_descriptor->name("MyAliasedBoundedString");
        alias_bounded_string_type_descriptor->base_type(DynamicTypeBuilderFactory::get_instance()->
                        create_string_type(100)->build());
        DynamicTypeBuilder::_ref_type alias_bounded_string_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                            create_type(
                                                                        alias_bounded_string_type_descriptor)};
        // Build the alias type for the bounded string
        DynamicType::_ref_type alias_bounded_string_type = alias_bounded_string_builder->build();

        // Define a recursive alias
        TypeDescriptor::_ref_type recursive_alias_type_descriptor {traits<TypeDescriptor>::make_shared()};
        recursive_alias_type_descriptor->kind(TK_ALIAS);
        recursive_alias_type_descriptor->name("MyRecursiveAlias");
        recursive_alias_type_descriptor->base_type(aliasenum_type);
        DynamicTypeBuilder::_ref_type recursive_alias_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                       create_type(recursive_alias_type_descriptor)};
        // Build the recursive alias type
        DynamicType::_ref_type recursive_alias_type = recursive_alias_builder->build();

        // Add alias enum member to the structure
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("my_aliased_enum");
        member_descriptor->type(aliasenum_type);
        struct_builder->add_member(member_descriptor);
        // Add alias bounded string member to the structure
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_aliased_bounded_string");
        member_descriptor->type(alias_bounded_string_type);
        struct_builder->add_member(member_descriptor);
        // Add recursive alias member to the structure
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->name("my_recursive_alias");
        member_descriptor->type(recursive_alias_type);
        struct_builder->add_member(member_descriptor);

        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for the alias enum member
        MyEnum in_value {MyEnum::C};
        int32_t out_value {0};
        data->set_int32_value(data->get_member_id_by_name("my_alias_enum"), in_value);
        data->get_int32_value(out_value, data->get_member_id_by_name("my_alias_enum"));
        //!--
    }
    {
        // Type created as described in enumeration type section.
        DynamicType::_ref_type bitmask_type;

        //!--CPP_SEQUENCES
        // Define a struct type to contain the sequence
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("SequenceStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};

        // Define a member for the sequence
        MemberDescriptor::_ref_type sequence_member_descriptor {traits<MemberDescriptor>::make_shared()};
        sequence_member_descriptor->name("bitmask_sequence");
        sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_sequence_type(bitmask_type,
                static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_SEQUENCE);
            type_descriptor->element_type(bitmask_type);
            type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
            sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
                    type_descriptor)->build());
         */

        // Add the sequence member to the struct
        struct_builder->add_member(sequence_member_descriptor);

        sequence_member_descriptor = traits<MemberDescriptor>::make_shared();
        sequence_member_descriptor->name("short_sequence");
        sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_sequence_type(
                    DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16), 5)->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_SEQUENCE);
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
            type_descriptor->bound().push_back(5);
            sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
                    type_descriptor)->build());
         */

        // Add the sequence member to the struct
        struct_builder->add_member(sequence_member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for the sequence member
        Int16Seq in_value = {1, 2};
        Int16Seq out_value;
        data->set_int16_values(data->get_member_id_by_name("short_sequence"), in_value);
        data->get_int16_values(out_value, data->get_member_id_by_name("short_sequence"));

        DynamicData::_ref_type sequence_data {data->loan_value(data->get_member_id_by_name("short_sequence"))};
        // Set the two latest possible values on the sequence
        sequence_data->set_int16_values(3, in_value);
        // Read every sequence value from index 1 to the end
        sequence_data->get_int16_values(out_value, 1);

        int16_t in_simple_value = 8;
        int16_t out_simple_value;
        sequence_data->set_int16_value(2, in_simple_value);
        sequence_data->get_int16_value(out_simple_value, 2);

        data->return_loaned_value(sequence_data);
        //!--
    }
    {
        //!--CPP_ARRAYS
        // Define a struct type to contain the array
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("ArrayStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};
        // Define a member for the array
        MemberDescriptor::_ref_type array_member_descriptor {traits<MemberDescriptor>::make_shared()};
        array_member_descriptor->name("long_array");
        array_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_array_type(
                    DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32), { 2, 3, 4 })->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_ARRAY);
            type_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
            type_descriptor->bound().push_back(2);
            type_descriptor->bound().push_back(3);
            type_descriptor->bound().push_back(4);
            array_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
                    type_descriptor)->build());
         */

        // Add the array member to the struct
        struct_builder->add_member(array_member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Set and retrieve values for the array member
        Int32Seq in_value = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24};
        Int32Seq out_value;
        data->set_int32_values(data->get_member_id_by_name("long_array"), in_value);
        data->get_int32_values(out_value, data->get_member_id_by_name("long_array"));

        DynamicData::_ref_type array_data {data->loan_value(data->get_member_id_by_name("long_array"))};
        // Set the two latest possible values on the array
        Int32Seq small_in_value = {0, 1};
        array_data->set_int32_values(22, small_in_value);
        // Read every array value from index 1 to the end
        array_data->get_int32_values(out_value, 1);

        int32_t in_simple_value = 8;
        int32_t out_simple_value;
        array_data->set_int32_value(2, in_simple_value);
        array_data->get_int32_value(out_simple_value, 2);

        data->return_loaned_value(array_data);
        //!--
    }
    {
        // Type created as described in enumeration type section.
        DynamicType::_ref_type alias_bounded_string_type;

        //!--CPP_MAPS
        // Define a struct type to contain the map
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("MapStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                              create_type(type_descriptor)};
        // Define a member for the map
        MemberDescriptor::_ref_type map_member_descriptor {traits<MemberDescriptor>::make_shared()};
        map_member_descriptor->name("string_long_array_unbounded_map");
        map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_map_type(
                    DynamicTypeBuilderFactory::get_instance()->create_string_type(static_cast<uint32_t>(
                        LENGTH_UNLIMITED))->build(), alias_bounded_string_type, static_cast<uint32_t>(
                        LENGTH_UNLIMITED))->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_MAP);
            type_descriptor->key_element_type(DynamicTypeBuilderFactory::get_instance()->create_string_type(
                    static_cast<uint32_t>(LENGTH_UNLIMITED)->build());
            type_descriptor->element_type(alias_bounded_string_type);
            type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
            map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
                    type_descriptor)->build());
         */

        // Add the map member to the struct
        struct_builder->add_member(map_member_descriptor);

        map_member_descriptor = traits<MemberDescriptor>::make_shared();
        map_member_descriptor->name("short_long_map");
        map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_map_type(
                    DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16),
                    DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32), 2)->build());

        /* Alternative
            type_descriptor = traits<TypeDescriptor>::make_shared();
            type_descriptor->kind(TK_MAP);
            type_descriptor->key_element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
            type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
            type_descriptor->bound().push_back(2);
            map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
                    type_descriptor)->build());
         */

        struct_builder->add_member(map_member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

        // Get the data loan of the map member
        DynamicData::_ref_type map_data = data->loan_value(data->get_member_id_by_name("short_long_map"));

        // Set and retrieve values for the map member
        int32_t key {1};
        int32_t in_value {2};
        int32_t out_value;
        map_data->set_int32_value(map_data->get_member_id_by_name(std::to_string(key)), in_value);
        map_data->get_int32_value(out_value, map_data->get_member_id_by_name(std::to_string(key)));

        // Return de data loan of the map member
        data->return_loaned_value(map_data);
        //!--
    }
    {
        //!--CPP_STRUCT
        // Create inner struct type
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(eprosima::fastdds::dds::TK_STRUCTURE);
        type_descriptor->name("InnerStruct");
        DynamicTypeBuilder::_ref_type builder {DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)};
        // Add members to the inner struct type
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        get_primitive_type(eprosima::fastdds::dds::TK_INT32));
        member_descriptor->name("first");
        member_descriptor->id(16);
        builder->add_member(member_descriptor);
        // Build the inner struct type
        DynamicType::_ref_type inner_struct_type {builder->build()};

        // Create parent struct type
        TypeDescriptor::_ref_type parentstruct_type_descriptor {traits<TypeDescriptor>::make_shared()};
        parentstruct_type_descriptor->kind(TK_STRUCTURE);
        parentstruct_type_descriptor->name("ParentStruct");
        DynamicTypeBuilder::_ref_type parentstruct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                    create_type(parentstruct_type_descriptor)};
        // Add members to the parent struct type
        MemberDescriptor::_ref_type parentstruct_member {traits<MemberDescriptor>::make_shared()};
        parentstruct_member->name("first");
        parentstruct_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT32));
        parentstruct_builder->add_member(parentstruct_member);
        parentstruct_member = traits<MemberDescriptor>::make_shared();
        parentstruct_member->name("second");
        parentstruct_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT64));
        parentstruct_builder->add_member(parentstruct_member);
        // Build the parent struct type
        DynamicType::_ref_type parentstruct_type = parentstruct_builder->build();

        // Create complex struct type
        TypeDescriptor::_ref_type complexstruct_type_descriptor {traits<TypeDescriptor>::make_shared()};
        complexstruct_type_descriptor->kind(TK_STRUCTURE);
        complexstruct_type_descriptor->name("ComplexStruct");
        complexstruct_type_descriptor->base_type(parentstruct_type);
        DynamicTypeBuilder::_ref_type complexstruct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                     create_type(complexstruct_type_descriptor)};
        // Add members to the complex struct type
        MemberDescriptor::_ref_type complexstruct_member {traits<MemberDescriptor>::make_shared()};
        complexstruct_member->name("complex_member");
        complexstruct_member->type(inner_struct_type);
        complexstruct_builder->add_member(complexstruct_member);

        // Build the complex struct type
        DynamicType::_ref_type complexstruct_type = complexstruct_builder->build();
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(complexstruct_type)};

        // Set and retrieve values for member of type float
        float in_value {3.14};
        float out_value {0.0};
        data->set_float32_value(data->get_member_id_by_name("first"), in_value);
        data->get_float32_value(out_value, data->get_member_id_by_name("first"));
        //!--
    }

    {
        // Skipped this type creation: same as primitives struct created in previous section.
        DynamicType::_ref_type struct_type;

        //!--CPP_UNION
        // Define a struct type to contain the union
        // Create the inner union
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_UNION);
        type_descriptor->name("InnerUnion");
        type_descriptor->discriminator_type(DynamicTypeBuilderFactory::get_instance()->
                        get_primitive_type(TK_INT16));
        DynamicTypeBuilder::_ref_type builder {DynamicTypeBuilderFactory::get_instance()->
                                                       create_type(type_descriptor)};
        // Add members to the inner union type
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->type(struct_type);
        member_descriptor->name("first");
        member_descriptor->id(16);
        member_descriptor->label({0});
        builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        get_primitive_type(TK_INT64));
        member_descriptor->name("second");
        member_descriptor->label({1});
        member_descriptor->is_default_label(true);
        builder->add_member(member_descriptor);
        // Build the inner union type
        DynamicType::_ref_type inner_union_type {builder->build()};

        // Create a complex union type
        type_descriptor = traits<TypeDescriptor>::make_shared();
        type_descriptor->kind(TK_UNION);
        type_descriptor->name("ComplexUnion");
        type_descriptor->discriminator_type(DynamicTypeBuilderFactory::get_instance()->
                        get_primitive_type(TK_INT32));
        builder = DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor);
        // Add members to the complex union type
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                        get_primitive_type(TK_INT32));
        member_descriptor->name("third");
        member_descriptor->label({0, 1});
        builder->add_member(member_descriptor);
        member_descriptor = traits<MemberDescriptor>::make_shared();
        member_descriptor->type(inner_union_type);
        member_descriptor->name("fourth");
        member_descriptor->is_default_label(true);
        builder->add_member(member_descriptor);
        // Build the complex union type
        DynamicType::_ref_type union_type {builder->build()};

        // Create dynamic data based on the union type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(union_type)};
        // Get the loan for the InnerUnion member
        DynamicData::_ref_type union_data = data->loan_value(data->get_member_id_by_name("InnerUnion"));

        // Set and retrieve values for the long long member within InnerUnion member
        int64_t in_value {2};
        int64_t out_value;
        union_data->set_int64_value(union_data->get_member_id_by_name("second"), in_value);
        union_data->get_int64_value(out_value, union_data->get_member_id_by_name("second"));
        // Return de data loan of the member
        data->return_loaned_value(union_data);
        //!--
    }
    {
        //!--CPP_BITSET
        // Define a struct type to contain the bitset
        TypeDescriptor::_ref_type struct_type_descriptor {traits<TypeDescriptor>::make_shared()};
        struct_type_descriptor->kind(TK_STRUCTURE);
        struct_type_descriptor->name("BitsetStruct");
        DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                          struct_type_descriptor)};

        // Define type for parent bitset
        TypeDescriptor::_ref_type bitset_type_descriptor {traits<TypeDescriptor>::make_shared()};
        bitset_type_descriptor->kind(TK_BITSET);
        bitset_type_descriptor->name("ParentBitSet");
        bitset_type_descriptor->bound({3, 1, 10, 12});
        DynamicTypeBuilder::_ref_type bitset_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                          bitset_type_descriptor)};
        // Add members to the bitset type
        MemberDescriptor::_ref_type bitset_member_descriptor {traits<MemberDescriptor>::make_shared()};
        bitset_member_descriptor->name("a");
        bitset_member_descriptor->id(0);
        bitset_builder->add_member(bitset_member_descriptor);
        bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitset_member_descriptor->name("b");
        bitset_member_descriptor->id(3);
        bitset_builder->add_member(bitset_member_descriptor);
        bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitset_member_descriptor->name("c");
        bitset_member_descriptor->id(8);
        bitset_builder->add_member(bitset_member_descriptor);
        bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
        bitset_member_descriptor->name("d");
        bitset_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
        bitset_member_descriptor->id(18);
        bitset_builder->add_member(bitset_member_descriptor);
        // Build the bitset type
        DynamicType::_ref_type parentbitset_type = bitset_builder->build();

        // Create child bitset type
        TypeDescriptor::_ref_type childbitset_type_descriptor {traits<TypeDescriptor>::make_shared()};
        childbitset_type_descriptor->kind(TK_BITSET);
        childbitset_type_descriptor->name("ChildBitSet");
        childbitset_type_descriptor->base_type(parentbitset_type);
        childbitset_type_descriptor->bound({1, 20});
        DynamicTypeBuilder::_ref_type childbitset_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                               childbitset_type_descriptor)};
        // Add members to the child bitset type
        MemberDescriptor::_ref_type childbitset_member_descriptor {traits<MemberDescriptor>::make_shared()};
        childbitset_member_descriptor->name("e");
        childbitset_member_descriptor->id(30);
        childbitset_builder->add_member(childbitset_member_descriptor);
        childbitset_member_descriptor = traits<MemberDescriptor>::make_shared();
        childbitset_member_descriptor->name("d");
        childbitset_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT32));
        childbitset_member_descriptor->id(31);
        childbitset_builder->add_member(childbitset_member_descriptor);
        // Build the child bitset type
        DynamicType::_ref_type bitset_type = childbitset_builder->build();

        // Add the bitset member to the struct
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("my_bitset");
        member_descriptor->type(bitset_type);
        struct_builder->add_member(member_descriptor);
        // Build the struct type
        DynamicType::_ref_type struct_type {struct_builder->build()};
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};
        // Get the loan for the bitset member
        DynamicData::_ref_type bitset_data = data->loan_value(data->get_member_id_by_name("my_bitset"));

        // Set and retrieve bitfield values
        int16_t in_value {2};
        int16_t out_value;
        bitset_data->set_int16_value(bitset_data->get_member_id_by_name("d"), in_value);
        bitset_data->get_int16_value(out_value, bitset_data->get_member_id_by_name("d"));
        // Return de data loan of the member
        data->return_loaned_value(bitset_data);
        //!--
    }
    {
        //!--CPP_CUSTOM_ANNOTATION
        // Create the structure to annotate
        TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
        type_descriptor->kind(TK_STRUCTURE);
        type_descriptor->name("AnnotatedStruct");
        DynamicTypeBuilder::_ref_type type_builder {DynamicTypeBuilderFactory::get_instance()->
                                                            create_type(type_descriptor)};
        MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
        member_descriptor->name("string_var");
        member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_string_type(static_cast<uint32_t>(
                    LENGTH_UNLIMITED))->build());
        type_builder->add_member(member_descriptor);

        // Create the annotation type
        AnnotationDescriptor::_ref_type annotation_descriptor {traits<AnnotationDescriptor>::make_shared()};
        TypeDescriptor::_ref_type annotation_type {traits<TypeDescriptor>::make_shared()};
        annotation_type->kind(TK_ANNOTATION);
        annotation_type->name("MyAnnotation");
        DynamicTypeBuilder::_ref_type annotation_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                  create_type(annotation_type)};

        // Add members to the annotation type
        MemberDescriptor::_ref_type annotation_parameter {traits<MemberDescriptor>::make_shared()};
        annotation_parameter->name("length");
        annotation_parameter->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
        annotation_builder->add_member(annotation_parameter);
        // Set the annotation type using the annotation descriptor
        annotation_descriptor->type(annotation_builder->build());
        // Set the value of the annotation
        annotation_descriptor->set_value("length", std::to_string(5));
        // Apply the annotation to the structure
        type_builder->apply_annotation(annotation_descriptor);

        // Reuse annotation descriptor to annotate struct member
        annotation_descriptor = traits<AnnotationDescriptor>::make_shared();
        annotation_descriptor->type(annotation_builder->build());
        annotation_descriptor->set_value("length", std::to_string(10));

        DynamicTypeMember::_ref_type member;
        type_builder->get_member_by_name(member, "string_var");
        type_builder->apply_annotation_to_member(member->get_id(), annotation_descriptor);
        //!--
    }
    {
        // Complex struct type defined previously
        DynamicType::_ref_type complexstruct_type;
        //!--CPP_COMPLEX_DATA
        // Create dynamic data based on the struct type
        DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(complexstruct_type)};

        // Get/Set complex API (copy)
        DynamicData::_ref_type complex_data;
        data->get_complex_value(complex_data, data->get_member_id_by_name("complex_member"));
        // Set data
        int32_t in_value {10};
        complex_data->set_int32_value(complex_data->get_member_id_by_name("first"), in_value);
        data->set_complex_value(data->get_member_id_by_name("complex_member"), complex_data);

        // Loan API
        DynamicData::_ref_type loan_data = data->loan_value(data->get_member_id_by_name("complex_member"));
        loan_data->set_int32_value(loan_data->get_member_id_by_name("first"), in_value);
        int32_t out_value;
        loan_data->get_int32_value(out_value, loan_data->get_member_id_by_name("first"));
        data->return_loaned_value(loan_data);
        //!--
    }
}

void xml_profiles_examples()
{
    {
        //XML-LOAD-APPLY-PROFILES
        if (RETCODE_OK ==
                DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
        {
            DomainParticipant* participant =
                    DomainParticipantFactory::get_instance()->create_participant_with_profile(
                0, "participant_xml_profile");

            Topic* topic =
                    participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

            Publisher* publisher = participant->create_publisher_with_profile("publisher_xml_profile");
            DataWriter* datawriter = publisher->create_datawriter_with_profile(topic, "datawriter_xml_profile");
            Subscriber* subscriber = participant->create_subscriber_with_profile("subscriber_xml_profile");
            DataReader* datareader = subscriber->create_datareader_with_profile(topic, "datareader_xml_profile");
        }

        // Load XML as string data buffer
        std::string xml_profile =
                "\
                <?xml version=\"1.0\" encoding=\"UTF-8\" ?>\
                <dds>\
                    <profiles xmlns=\"http://www.eprosima.com\" >\
                        <data_writer profile_name=\"test_datawriter_profile\" is_default_profile=\"true\">\
                            <qos>\
                                <durability>\
                                    <kind>TRANSIENT_LOCAL</kind>\
                                </durability>\
                            </qos>\
                        </data_writer>\
                    </profiles>\
                </dds>\
                ";
        if (RETCODE_OK ==
                DomainParticipantFactory::get_instance()->load_XML_profiles_string(xml_profile.c_str(),
                xml_profile.length()))
        {
            // Create DDS entities with profiles
        }
        //!--
    }
    {
        //XML-USAGE
        // Create a DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Load the XML File
        if (RETCODE_OK ==
                DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
        {
            // Retrieve instance of the desired type
            DynamicTypeBuilder::_ref_type my_struct_type;
            if (RETCODE_OK !=
                    DomainParticipantFactory::get_instance()->get_dynamic_type_builder_from_xml_by_name(
                        "MyStruct", my_struct_type))
            {
                // Error
                return;
            }

            // Register MyStruct type
            TypeSupport my_struct_type_support(new DynamicPubSubType(my_struct_type->build()));
            my_struct_type_support.register_type(participant, nullptr);
        }
        else
        {
            std::cout << "Cannot open XML file \"types.xml\". "
                      << "Please, set the correct path to the XML file"
                      << std::endl;
        }
        //!--
    }
    {
        std::string custom_name;
        //XML-MIX-WITH-CODE
        if (RETCODE_OK ==
                DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
        {
            DomainParticipantQos participant_qos;
            DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                "participant_xml_profile",
                participant_qos);

            // Name obtained in another section of the code
            participant_qos.name() = custom_name;

            // Modify number of preallocations (this overrides the one set in the XML profile)
            participant_qos.allocation().send_buffers.preallocated_number = 10;

            // Create participant using the modified XML Qos
            DomainParticipant* participant =
                    DomainParticipantFactory::get_instance()->create_participant(
                0, participant_qos);
        }
        //!--
    }
    {
        //XML-GET-QOS-FROM-XML
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);

        Topic* topic =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

        Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);

        // Load XML as string data buffer
        std::string xml_profile =
                "\
                <?xml version=\"1.0\" encoding=\"UTF-8\" ?>\
                <dds>\
                    <profiles xmlns=\"http://www.eprosima.com\" >\
                        <data_writer profile_name=\"test_datawriter_profile\" is_default_profile=\"true\">\
                            <qos>\
                                <durability>\
                                    <kind>TRANSIENT_LOCAL</kind>\
                                </durability>\
                            </qos>\
                        </data_writer>\
                    </profiles>\
                </dds>\
                ";

        // Extract Qos from XML
        DataWriterQos qos;
        if (RETCODE_OK == publisher->get_datawriter_qos_from_xml(xml_profile, qos, "test_datawriter_profile"))
        {
            // Modify extracted qos and use it to create DDS entities
        }
        //!--
    }
}

void dds_transport_examples ()
{
    using UDPv4TransportDescriptor = eprosima::fastdds::rtps::UDPv4TransportDescriptor;
    using UDPv6TransportDescriptor = eprosima::fastdds::rtps::UDPv6TransportDescriptor;
    using TCPv4TransportDescriptor = eprosima::fastdds::rtps::TCPv4TransportDescriptor;
    using TCPv6TransportDescriptor = eprosima::fastdds::rtps::TCPv6TransportDescriptor;
    using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;
    using Locator_t = eprosima::fastdds::rtps::Locator_t;
    using IPLocator = eprosima::fastdds::rtps::IPLocator;

    {
        //CONF-IPLOCATOR-USAGE
        // We will configure a TCP locator with IPLocator
        Locator_t locator;

        // Get & set the physical port
        uint16_t physical_port = IPLocator::getPhysicalPort(locator);
        IPLocator::setPhysicalPort(locator, 5555);

        // On TCP locators, we can get & set the logical port
        uint16_t logical_port = IPLocator::getLogicalPort(locator);
        IPLocator::setLogicalPort(locator, 7400);

        // Set WAN address
        IPLocator::setWan(locator, "80.88.75.55");
        //!--
    }

    {
        //CONF-TRANSPORT_METAMULTICASTLOCATOR
        DomainParticipantQos qos;

        // This locator will open a socket to listen network messages
        // on UDPv4 port 22222 over multicast address 239.255.0.1
        eprosima::fastdds::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 239, 255, 0, 1);
        locator.port = 22222;

        // Add the locator to the DomainParticipantQos
        qos.wire_protocol().builtin.metatrafficMulticastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF-TRANSPORT_METAUNICASTLOCATOR
        DomainParticipantQos qos;

        // This locator will open a socket to listen network messages
        // on UDPv4 port 22223 over address 192.168.0.1
        eprosima::fastdds::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 0, 1);
        locator.port = 22223;

        // Add the locator to the DomainParticipantQos
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF-TRANSPORT_USERMULTICASTLOCATOR
        DomainParticipantQos qos;

        // This locator will open a socket to listen network messages
        // on UDPv4 port 22224 over multicast address 239.255.0.1
        eprosima::fastdds::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 239, 255, 0, 1);
        locator.port = 22224;

        // Add the locator to the DomainParticipantQos
        qos.wire_protocol().default_multicast_locator_list.push_back(locator);
        //!--
    }

    {
        //CONF-TRANSPORT_USERUNICASTLOCATOR
        DomainParticipantQos qos;

        // This locator will open a socket to listen network messages
        // on UDPv4 port 22225 over address 192.168.0.1
        eprosima::fastdds::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 0, 1);
        locator.port = 22225;

        // Add the locator to the DomainParticipantQos
        qos.wire_protocol().default_unicast_locator_list.push_back(locator);
        //!--
    }

    {
        //CONF-UDP-TRANSPORT-SETTING
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        udp_transport->sendBufferSize = 9216;
        udp_transport->receiveBufferSize = 9216;
        udp_transport->non_blocking_send = true;

        // [OPTIONAL] ThreadSettings configuration
        udp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{2, 2, 2, 2});
        udp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{3, 3, 3, 3});

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(udp_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //CONF-TCP-TRANSPORT-BUILTIN-TRANSPORT
        eprosima::fastdds::dds::DomainParticipantQos qos;
        qos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);
        //!--
    }

    {
        //CONF-TCP-TRANSPORT-SETTING-SERVER
        eprosima::fastdds::dds::DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        tcp_transport->add_listener_port(5100);

        // [OPTIONAL] ThreadSettings configuration
        tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
        tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;

        // [OPTIONAL] Set unicast locators
        eprosima::fastdds::rtps::Locator_t locator;
        locator.kind = LOCATOR_KIND_TCPv4;
        eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "192.168.1.10");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(locator, 5100);
        // [OPTIONAL] Logical port default value is 0, automatically assigned.
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(locator, 5100);

        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
        qos.wire_protocol().default_unicast_locator_list.push_back(locator);
        //!--
    }

    {
        //CONF-TCP-TRANSPORT-SETTING-CLIENT
        eprosima::fastdds::dds::DomainParticipantQos qos;

        // Disable the built-in Transport Layer.
        qos.transport().use_builtin_transports = false;

        // Create a descriptor for the new transport.
        // Do not configure any listener port
        auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        qos.transport().user_transports.push_back(tcp_transport);

        // [OPTIONAL] ThreadSettings configuration
        tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
        tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

        // Set initial peers.
        eprosima::fastdds::rtps::Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
        eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
        // If the logical port is set in the server side, it must be also set here with the same value.
        // If not set in the server side in a unicast locator, do not set it here.
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
        //!--
    }

    {
        //CONF-TCP-TRANSPORT-SETTING-WAN-SERVER
        eprosima::fastdds::dds::DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        tcp_transport->add_listener_port(5100);
        tcp_transport->set_WAN_address("80.80.99.45");

        // [OPTIONAL] ThreadSettings configuration
        tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
        tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;

        // [OPTIONAL] Set unicast locators (do not use setWAN(), set_WAN_address() overwrites it)
        eprosima::fastdds::rtps::Locator_t locator;
        locator.kind = LOCATOR_KIND_TCPv4;
        // [RECOMMENDED] Use the LAN address of the server
        eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "192.168.1.10");
        // [ALTERNATIVE] Use server's WAN address. In that case, initial peers must be configured
        // only with server's WAN address.
        // eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "80.80.99.45");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(locator, 5100);
        // [OPTIONAL] Logical port default value is 0, automatically assigned.
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(locator, 5100);

        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
        qos.wire_protocol().default_unicast_locator_list.push_back(locator);
        //!--
    }

    {
        //CONF-TCP-TRANSPORT-SETTING-WAN-CLIENT
        eprosima::fastdds::dds::DomainParticipantQos qos;

        // Disable the built-in Transport Layer.
        qos.transport().use_builtin_transports = false;

        // Create a descriptor for the new transport.
        // Do not configure any listener port
        auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        // [RECOMMENDED] Use client's WAN address if there are more clients in other local networks.
        tcp_transport->set_WAN_address("80.80.99.47");
        qos.transport().user_transports.push_back(tcp_transport);

        // [OPTIONAL] ThreadSettings configuration
        tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
        tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

        // Set initial peers.
        eprosima::fastdds::rtps::Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
        // [RECOMMENDED] Use both WAN and LAN server addresses
        eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
        eprosima::fastdds::rtps::IPLocator::setWan(initial_peer_locator, "80.80.99.45");
        // [ALTERNATIVE] Use server's WAN address only. Valid if server specified its unicast locators
        // with its LAN or WAN address.
        // eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "80.80.99.45");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
        // If the logical port is set in the server side, it must be also set here with the same value.
        // If not set in the server side in a unicast locator, do not set it here.
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
        //!--
    }

    {
        //CONF-SHM-TRANSPORT-SETTING
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        std::shared_ptr<SharedMemTransportDescriptor> shm_transport =
                std::make_shared<SharedMemTransportDescriptor>();

        // [OPTIONAL] ThreadSettings configuration
        shm_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        shm_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
        shm_transport->dump_thread(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(shm_transport);
        //!--
    }

    {
        //CONF-SHM-TRANSPORT-DISABLE-BUILTIN-TRANSPORTS
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        std::shared_ptr<SharedMemTransportDescriptor> shm_transport =
                std::make_shared<SharedMemTransportDescriptor>();

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(shm_transport);

        // Explicit configuration of SharedMem transport
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //CONF-TCP-TLS-SERVER
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();
        tls_transport->sendBufferSize = 9216;
        tls_transport->receiveBufferSize = 9216;
        tls_transport->add_listener_port(5100);

        // Create the TLS configuration
        using TLSOptions = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions;
        tls_transport->apply_security = true;
        tls_transport->tls_config.password = "test";
        tls_transport->tls_config.cert_chain_file = "server.pem";
        tls_transport->tls_config.private_key_file = "serverkey.pem";
        tls_transport->tls_config.tmp_dh_file = "dh2048.pem";
        tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
        tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
        tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tls_transport);
        //!--
    }

    {
        //CONF-TCP-TLS-CLIENT
        DomainParticipantQos qos;

        // Set initial peers.
        Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
        IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
        initial_peer_locator.port = 5100;
        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);

        // Create a descriptor for the new transport.
        auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();

        // Create the TLS configuration
        using TLSOptions = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions;
        using TLSVerifyMode = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode;
        tls_transport->apply_security = true;
        tls_transport->tls_config.verify_file = "ca.pem";
        tls_transport->tls_config.add_verify_mode(TLSVerifyMode::VERIFY_PEER);
        tls_transport->tls_config.add_verify_mode(TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT);
        tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
        tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
        tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);
        tls_transport->tls_config.server_name = "my_server.com";

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tls_transport);
        //!--
    }

    {
        //TRANSPORT-DESCRIPTORS
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();

        // Add loopback to the whitelist by IP address
        tcp_transport->interfaceWhiteList.emplace_back("127.0.0.1");

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

        // Avoid using the builtin transports
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //WHITELIST-NAME
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();

        // Add loopback to the whitelist by interface name
        tcp_transport->interfaceWhiteList.emplace_back("lo");

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

        // Avoid using the builtin transports
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        using namespace eprosima::fastdds::rtps;
        //CONF-NETMASK-FILTER
        DomainParticipantQos qos;

        // Configure netmask filtering at participant level
        qos.transport().netmask_filter = NetmaskFilterKind::AUTO;
        qos.wire_protocol().ignore_non_matching_locators = true; // Required if not defining an allowlist or blocklist

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Configure netmask filtering at transport level
        udp_transport->netmask_filter = NetmaskFilterKind::AUTO;
        qos.wire_protocol().ignore_non_matching_locators = true; // Required if not defining an allowlist or blocklist

        // Configure netmask filtering at interface level
        udp_transport->interface_allowlist.emplace_back("wlp59s0", NetmaskFilterKind::ON);

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(udp_transport);

        // Avoid using the builtin transports
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        using namespace eprosima::fastdds::rtps;
        //CONF-INTERFACES-ALLOWLIST
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Add allowed interface by device name
        udp_transport->interface_allowlist.emplace_back("eth0", NetmaskFilterKind::OFF);

        // Add allowed interface by IP address (using default netmask filter AUTO)
        udp_transport->interface_allowlist.emplace_back("127.0.0.1");

        // Add allowed interface with explicit AllowedNetworkInterface construction
        AllowedNetworkInterface another_allowed_interface("docker0", NetmaskFilterKind::OFF);
        udp_transport->interface_allowlist.emplace_back(another_allowed_interface);

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(udp_transport);

        // Avoid using the builtin transports
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        using namespace eprosima::fastdds::rtps;
        //CONF-INTERFACES-BLOCKLIST
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Add blocked interface by device name
        udp_transport->interface_blocklist.emplace_back("docker0");

        // Add blocked interface by IP address
        udp_transport->interface_blocklist.emplace_back("127.0.0.1");

        // Add blocked interface with explicit BlockedNetworkInterface construction
        BlockedNetworkInterface another_blocked_interface("eth0");
        udp_transport->interface_blocklist.emplace_back(another_blocked_interface);

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(udp_transport);

        // Avoid using the builtin transports
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //CONF-DISABLE-MULTICAST
        DomainParticipantQos qos;

        // Metatraffic Multicast Locator List will be empty.
        // Metatraffic Unicast Locator List will contain one locator, with null address and null port.
        // Then Fast DDS will use all network interfaces to receive network messages using a well-known port.
        Locator_t default_unicast_locator;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(default_unicast_locator);

        // Initial peer will be UDPv4 address 192.168.0.1. The port will be a well-known port.
        // Initial discovery network messages will be sent to this UDPv4 address.
        Locator_t initial_peer;
        IPLocator::setIPv4(initial_peer, 192, 168, 0, 1);
        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer);
        //!--
    }

    {
        //CONF-CUSTOM-CHAINING-TRANSPORT-SETTING
        DomainParticipantQos qos;

        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Create a descriptor for the new transport.
        // The low level transport will be a UDPv4Transport.
        auto custom_transport = std::make_shared<CustomChainingTransportDescriptor>(udp_transport);

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(custom_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;
        //!--
    }
}

void dds_usecase_examples()
{
    using Locator_t = eprosima::fastdds::rtps::Locator_t;
    using IPLocator = eprosima::fastdds::rtps::IPLocator;
    using DiscoveryProtocol = eprosima::fastdds::rtps::DiscoveryProtocol;

    {
        //CONF_INITIAL_PEERS_BASIC
        DomainParticipantQos qos;

        // configure an initial peer on host 192.168.10.13.
        // The port number corresponds to the well-known port for metatraffic unicast
        // on participant ID `1` and domain `0`.
        Locator_t initial_peer;
        IPLocator::setIPv4(initial_peer, "192.168.10.13");
        initial_peer.port = 7412;
        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer);
        //!--
    }

    {
        //CONF_INITIAL_PEERS_METAUNICAST
        DomainParticipantQos qos;

        // configure one metatraffic unicast locator on interface 192.168.10.13.
        // on participant ID `1` and domain `0`.
        Locator_t meta_unicast_locator;
        IPLocator::setIPv4(meta_unicast_locator, "192.168.10.13");
        meta_unicast_locator.port = 7412;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(meta_unicast_locator);
        //!--
    }

    {
        //CONF_DS_MAIN_SCENARIO_SERVER
        DomainParticipantQos qos;

        // Configure the current participant as SERVER
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;

        // Define the listening locator to be on interface 192.168.10.57 and port 56542
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.57");
        server_locator.port = 56542;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);
        //!--
    }

    {
        //CONF_DS_MAIN_SCENARIO_CLIENT
        DomainParticipantQos qos;

        // Configure the current participant as CLIENT
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::CLIENT;

        // Define a locator for the SERVER Participant on address 192.168.10.57 and port 56542
        Locator_t remote_server_locator;
        IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
        remote_server_locator.port = 56542;

        // Connect to the SERVER at the previous locator
        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator);
        //!--
    }

    {
        //CONF_DS_REDUNDANCY_SCENARIO_SERVER
        // Configure first server's locator on interface 192.168.10.57 and port 56542
        Locator_t server_locator_1;
        IPLocator::setIPv4(server_locator_1, "192.168.10.57");
        server_locator_1.port = 56542;

        // Configure participant_1 as SERVER listening on the previous locator
        DomainParticipantQos server_1_qos;
        server_1_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
        // Optional GUID
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> server_1_qos.wire_protocol().prefix;
        server_1_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator_1);

        // Configure second server's locator on interface 192.168.10.60 and port 56543
        Locator_t server_locator_2;
        IPLocator::setIPv4(server_locator_2, "192.168.10.60");
        server_locator_2.port = 56543;

        // Configure participant_2 as SERVER listening on the previous locator
        DomainParticipantQos server_2_qos;
        server_2_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
        // Optional GUID
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.32") >> server_2_qos.wire_protocol().prefix;
        server_2_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator_2);
        //!--
    }

    {
        //CONF_DS_REDUNDANCY_SCENARIO_CLIENT
        // Define a locator for the first SERVER Participant
        Locator_t remote_server_locator_1;
        IPLocator::setIPv4(remote_server_locator_1, "192.168.10.57");
        remote_server_locator_1.port = 56542;

        // Define a locator for the second SERVER Participant
        Locator_t remote_server_locator_2;
        IPLocator::setIPv4(remote_server_locator_2, "192.168.10.60");
        remote_server_locator_2.port = 56543;

        // Configure the current participant as CLIENT connecting to the SERVERS at the previous locators
        DomainParticipantQos client_qos;
        client_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::CLIENT;
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_1);
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_2);
        //!--
    }

    {
        //CONF_DS_PARTITION_2
        DomainParticipantQos qos;

        // Configure current Participant as SERVER on address 192.168.10.60
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.60");
        server_locator.port = 56543;

        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
        // Optional GUID
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> qos.wire_protocol().prefix;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        // Add the connection attributes to the remote server.
        Locator_t remote_server_locator;
        IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
        remote_server_locator.port = 56542;

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator);
        //!--
    }

    {
        //CONF_DS_PARTITION_3
        DomainParticipantQos qos;

        // Configure current Participant as SERVER on address 192.168.10.60
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.54");
        server_locator.port = 56541;

        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
        // Optional GUID
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.33") >> qos.wire_protocol().prefix;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        // Add the connection attributes to the remote server A.
        Locator_t remote_server_locator_A;
        IPLocator::setIPv4(remote_server_locator_A, "192.168.10.60");
        remote_server_locator_A.port = 56543;

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_A);

        // Add the connection attributes to the remote server B.
        Locator_t remote_server_locator_B;
        IPLocator::setIPv4(remote_server_locator_B, "192.168.10.57");
        remote_server_locator_B.port = 56542;

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_B);
        //!--
    }

    {
        //STATIC_DISCOVERY_USE_CASE_PUB
        // Participant configuration
        DomainParticipantQos participant_qos;
        participant_qos.name("HelloWorldPublisher");
        participant_qos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
        participant_qos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
        participant_qos.wire_protocol().builtin.discovery_config.static_edp_xml_config("HelloWorldSubscriber.xml");

        // DataWriter configuration
        DataWriterQos writer_qos;
        writer_qos.endpoint().user_defined_id = 1;
        writer_qos.endpoint().entity_id = 2;

        // Create the DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create the Publisher
        Publisher* publisher =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        // Create the Topic with the appropriate name and data type
        std::string topic_name = "HelloWorldTopic";
        std::string data_type = "HelloWorld";
        Topic* topic =
                participant->create_topic(topic_name, data_type, TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create the DataWriter
        DataWriter* writer =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == writer)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //STATIC_DISCOVERY_USE_CASE_SUB
        // Participant configuration
        DomainParticipantQos participant_qos;
        participant_qos.name("HelloWorldSubscriber");
        participant_qos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
        participant_qos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
        participant_qos.wire_protocol().builtin.discovery_config.static_edp_xml_config("HelloWorldPublisher.xml");

        // DataWriter configuration
        DataWriterQos writer_qos;
        writer_qos.endpoint().user_defined_id = 3;
        writer_qos.endpoint().entity_id = 4;

        // Create the DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create the Subscriber
        Subscriber* subscriber =
                participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // Create the Topic with the appropriate name and data type
        std::string topic_name = "HelloWorldTopic";
        std::string data_type = "HelloWorld";
        Topic* topic =
                participant->create_topic(topic_name, data_type, TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        // Create the DataReader
        DataReader* reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
        if (nullptr == reader)
        {
            // Error
            return;
        }
        //!--
    }

    {
        //CONF-QOS-INCREASE-SOCKETBUFFERS
        DomainParticipantQos participant_qos;

        // Increase the sending buffer size
        participant_qos.transport().send_socket_buffer_size = 1048576;

        // Increase the receiving buffer size
        participant_qos.transport().listen_socket_buffer_size = 4194304;
        //!--
    }

    {
        //CONF-QOS-FLOWCONTROLLER
        // Limit to 300kb per second.
        static const char* flow_controller_name = "example_flow_controller";
        auto flow_control_300k_per_sec = std::make_shared<eprosima::fastdds::rtps::FlowControllerDescriptor>();
        flow_control_300k_per_sec->name = flow_controller_name;
        flow_control_300k_per_sec->scheduler = eprosima::fastdds::rtps::FlowControllerSchedulerPolicy::FIFO;
        flow_control_300k_per_sec->max_bytes_per_period = 300 * 1000;
        flow_control_300k_per_sec->period_ms = 1000;

        // [OPTIONAL] Configure sender thread settings
        flow_control_300k_per_sec->sender_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

        // Register flow controller on participant
        DomainParticipantQos participant_qos;
        participant_qos.flow_controllers().push_back(flow_control_300k_per_sec);

        // .... create participant and publisher

        // Link writer to the registered flow controller.
        // Note that ASYNCHRONOUS_PUBLISH_MODE must be used
        DataWriterQos qos;
        qos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
        qos.publish_mode().flow_controller_name = flow_controller_name;
        //!--
    }

    {
        //CONF_QOS_TUNING_RELIABLE_WRITER
        DataWriterQos qos;
        qos.reliable_writer_qos().times.heartbeat_period.seconds = 0;
        qos.reliable_writer_qos().times.heartbeat_period.nanosec = 500000000;     //500 ms
        //!--
    }

    {
        //DDS_MULTICAST_DELIVERY
        DataReaderQos qos;

        // Add new multicast locator with IP 239.255.0.4 and port 7900
        eprosima::fastdds::rtps::Locator_t new_multicast_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
        new_multicast_locator.port = 7900;
        qos.endpoint().multicast_locator_list.push_back(new_multicast_locator);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-PARTICIPANTS
        DomainParticipantQos qos;

        // Fix the size of discovered participants to 3
        // This will effectively preallocate the memory during initialization
        qos.allocation().participants =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);

        // Fix the size of discovered DataWriters to 1 per DomainParticipant
        // Fix the size of discovered DataReaders to 3 per DomainParticipant
        // This will effectively preallocate the memory during initialization
        qos.allocation().writers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        qos.allocation().readers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-PARAMETERS
        DomainParticipantQos qos;

        // Fix the size of the complete user data field to 256 octets
        qos.allocation().data_limits.max_user_data = 256u;
        // Fix the size of the complete partitions field to 256 octets
        qos.allocation().data_limits.max_partitions = 256u;
        // Fix the size of the complete properties field to 512 octets
        qos.allocation().data_limits.max_properties = 512u;
        // Set the preallocated filter expression size to 512 characters
        qos.allocation().content_filter.expression_initial_size = 512u;
        // Set the maximum number of expression parameters to 4 and its allocation configuration to fixed size
        qos.allocation().content_filter.expression_parameters =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-WRITER
        DataWriterQos qos;

        // Fix the size of matched DataReaders to 3
        // This will effectively preallocate the memory during initialization
        qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // Fix the size of writer side content filters to 1
        // This will effectively preallocate the memory during initialization
        qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-READER
        DataReaderQos qos;

        // Fix the size of matched DataWriters to 1
        // This will effectively preallocate the memory during initialization
        qos.reader_resource_limits().matched_publisher_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-EXAMPLE
        // DomainParticipant configuration
        //////////////////////////////////
        DomainParticipantQos participant_qos;

        // We know we have 3 participants on the domain
        participant_qos.allocation().participants =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // We know we have at most 2 readers on each participant
        participant_qos.allocation().readers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        // We know we have at most 1 writer on each participant
        participant_qos.allocation().writers =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

        // We know the maximum size of partition data
        participant_qos.allocation().data_limits.max_partitions = 256u;
        // We know the maximum size of user data
        participant_qos.allocation().data_limits.max_user_data = 256u;
        // We know the maximum size of properties data
        participant_qos.allocation().data_limits.max_properties = 512u;

        // Content filtering is not being used
        participant_qos.allocation().content_filter.expression_initial_size = 0u;
        participant_qos.allocation().content_filter.expression_parameters =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

        // DataWriter configuration for Topic 1
        ///////////////////////////////////////
        DataWriterQos writer1_qos;

        // We know we will only have three matching subscribers, and no content filters
        writer1_qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        writer1_qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

        // DataWriter configuration for Topic 2
        ///////////////////////////////////////
        DataWriterQos writer2_qos;

        // We know we will only have two matching subscribers
        writer2_qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        writer2_qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);


        // DataReader configuration for both Topics
        ///////////////////////////////////////////
        DataReaderQos reader_qos;

        // We know we will only have one matching publisher
        reader_qos.reader_resource_limits().matched_publisher_allocation =
                eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-MEMORY-QOS-PUBSUB
        ResourceLimitsQosPolicy resource_limits;

        // The ResourceLimitsQosPolicy is constructed with max_samples = 5000 by default
        // Change max_samples to the minimum
        resource_limits.max_samples = 1;

        // The ResourceLimitsQosPolicy is constructed with max_instances = 10 by default
        // Change max_instances to the minimum
        resource_limits.max_instances = 1;

        // The ResourceLimitsQosPolicy is constructed with max_samples_per_instance = 400 by default
        // Change max_samples_per_instance to the minimum
        resource_limits.max_samples_per_instance = 1;

        // The ResourceLimitsQosPolicy is constructed with allocated_samples = 100 by default
        // No allocated samples
        resource_limits.allocated_samples = 0;
        //!--
    }


    {
        //CONF-MEMORY-QOS-ENDPOINTS
        RTPSEndpointQos endpoint;
        endpoint.history_memory_policy = eprosima::fastdds::rtps::DYNAMIC_REUSABLE_MEMORY_MODE;
        //!--
    }

    {
        // Create the DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Create the Publisher
        Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr == publisher)
        {
            // Error
            return;
        }

        // Create the Subscriber
        Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (nullptr == subscriber)
        {
            // Error
            return;
        }

        // Create the Topic with the appropriate name and data type
        std::string topic_name = "HelloWorldTopic";
        std::string data_type = "HelloWorld";
        Topic* topic = participant->create_topic(topic_name, data_type, TOPIC_QOS_DEFAULT);
        if (nullptr == topic)
        {
            // Error
            return;
        }

        //UNIQUE_NETWORK_FLOWS_USE_CASE
        // Create the DataWriter
        DataWriter* writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr == writer)
        {
            // Error
            return;
        }

        // Create DataReader with unique flows
        DataReaderQos drqos = DATAREADER_QOS_DEFAULT;
        drqos.properties().properties().emplace_back("fastdds.unique_network_flows", "");
        DataReader* reader = subscriber->create_datareader(topic, drqos);

        // Print locators information
        eprosima::fastdds::rtps::LocatorList locators;
        writer->get_sending_locators(locators);
        std::cout << "Writer is sending from the following locators:" << std::endl;
        for (const auto& locator : locators)
        {
            std::cout << "  " << locator << std::endl;
        }

        reader->get_listening_locators(locators);
        std::cout << "Reader is listening on the following locators:" << std::endl;
        for (const Locator_t& locator : locators)
        {
            std::cout << "  " << locator << std::endl;
        }
        //!--
    }

    {
        //DYNAMIC_NETWORK_INTERFACES_USE_CASE
        // Create the DomainParticipant
        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // User application

        // Notify Fast DDS a new network interface is available
        participant->set_qos(PARTICIPANT_QOS_DEFAULT);
        //!--
    }
}

void dds_persistence_examples()
{
    //CONF-PERSISTENCE-SERVICE-SQLITE3-EXAMPLE
    /*
     * In order for this example to be self-contained, all the entities are created programatically, including the data
     * type and type support. This has been done using Fast DDS Dynamic Types API, but it could be substituted with a
     * Fast DDS-Gen generated type support if an IDL file is available. The Dynamic Type created here is the equivalent
     * of the following IDL:
     *
     *     struct persistence_topic_type
     *     {
     *         unsigned long index;
     *         string message;
     *     };
     */

    // Configure persistence service plugin for DomainParticipant
    DomainParticipantQos pqos;
    pqos.properties().properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");
    pqos.properties().properties().emplace_back("dds.persistence.sqlite3.filename", "persistence.db");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

    /********************************************************************************************************
    * CREATE TYPE AND TYPE SUPPORT
    *********************************************************************************************************
    * This part could be replaced if IDL file and Fast DDS-Gen are available.
    * The type is created with name "persistence_topic_type"
    * Additionally, create a data object and populate it, just to show how to do it
    ********************************************************************************************************/
    // Create a struct builder for a type with name "persistence_topic_type"
    const std::string topic_type_name = "persistence_topic_type";

    TypeDescriptor::_ref_type struct_type_descriptor {traits<TypeDescriptor>::make_shared()};
    struct_type_descriptor->kind(TK_STRUCTURE);
    struct_type_descriptor->name(topic_type_name);
    DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                          create_type(struct_type_descriptor)};

    // The type consists of two members, and index and a message. Add members to the struct.
    MemberDescriptor::_ref_type index_member_descriptor {traits<MemberDescriptor>::make_shared()};
    index_member_descriptor->name("index");
    index_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                    get_primitive_type(TK_UINT32));
    struct_builder->add_member(index_member_descriptor);

    MemberDescriptor::_ref_type message_member_descriptor {traits<MemberDescriptor>::make_shared()};
    message_member_descriptor->name("message");
    message_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                    create_string_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());
    struct_builder->add_member(message_member_descriptor);

    // Build the type
    DynamicType::_ref_type struct_type {struct_builder->build()};

    // Create type support and register the type
    TypeSupport type_support(new DynamicPubSubType(struct_type));
    type_support.register_type(participant);

    // Create data sample a populate data. This is to be used when calling `writer->write()`
    DynamicData::_ref_type dyn_helloworld {DynamicDataFactory::get_instance()->create_data(struct_type)};

    dyn_helloworld->set_uint32_value(0, 0);
    dyn_helloworld->set_string_value(1, "HelloWorld");
    /********************************************************************************************************
    * END CREATE TYPE AND TYPE SUPPORT
    ********************************************************************************************************/

    // Create a topic
    Topic* topic = participant->create_topic("persistence_topic_name", topic_type_name, TOPIC_QOS_DEFAULT);

    // Create a publisher and a subscriber with default QoS
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

    // Configure DataWriter's durability and persistence GUID so it can use the persistence service
    DataWriterQos dwqos = DATAWRITER_QOS_DEFAULT;
    dwqos.durability().kind = TRANSIENT_DURABILITY_QOS;
    dwqos.properties().properties().emplace_back("dds.persistence.guid",
            "77.72.69.74.65.72.5f.70.65.72.73.5f|67.75.69.64");
    DataWriter* writer = publisher->create_datawriter(topic, dwqos);

    // Configure DataReaders's durability and persistence GUID so it can use the persistence service
    DataReaderQos drqos = DATAREADER_QOS_DEFAULT;
    drqos.durability().kind = TRANSIENT_DURABILITY_QOS;
    drqos.properties().properties().emplace_back("dds.persistence.guid",
            "72.65.61.64.65.72.5f.70.65.72.73.5f|67.75.69.64");
    DataReader* reader = subscriber->create_datareader(topic, drqos);
    //!--
}

class LoanableHelloWorldPubSubType : public eprosima::fastdds::dds::TopicDataType
{
public:

    LoanableHelloWorldPubSubType()
        : TopicDataType()
    {
        set_name("LoanableHelloWorld");
    }

    bool serialize(
            const void* const data,
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override
    {
        return true;
    }

    bool deserialize(
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            void* data) override
    {
        return true;
    }

    uint32_t calculate_serialized_size(
            const void* const data,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override
    {
        return 0;
    }

    void* create_data() override
    {
        return nullptr;
    }

    void delete_data(
            void* data) override
    {
    }

    bool compute_key(
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
            bool force_md5) override
    {
        return true;
    }

    bool compute_key(
            const void* const data,
            eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
            bool force_md5) override
    {
        return true;
    }

};

class LoanableHelloWorld
{
public:

    LoanableHelloWorld()
    {
        m_index = 0;
        memset(&m_message, 0, (256) * 1);
    }

    uint32_t& index()
    {
        return m_index;
    }

    uint32_t index() const
    {
        return m_index;
    }

    const std::array<char, 256>& message() const
    {
        return m_message;
    }

    std::array<char, 256>& message()
    {
        return m_message;
    }

private:

    uint32_t m_index;
    std::array<char, 256> m_message;
};

void dds_zero_copy_example()
{
    {
        //LOANABLE_HELLOWORLD_EXAMPLE_WRITER
        // CREATE THE PARTICIPANT
        DomainParticipantQos pqos;
        pqos.name("Participant_pub");
        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

        // REGISTER THE TYPE
        TypeSupport type(new LoanableHelloWorldPubSubType());
        type.register_type(participant);

        // CREATE THE PUBLISHER
        Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        // CREATE THE TOPIC
        Topic* topic = participant->create_topic(
            "LoanableHelloWorldTopic",
            type.get_type_name(),
            TOPIC_QOS_DEFAULT);

        // CREATE THE WRITER
        DataWriterQos wqos = publisher->get_default_datawriter_qos();
        wqos.history().depth = 10;
        wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        // DataSharingQosPolicy has to be set to AUTO (the default) or ON to enable Zero-Copy
        wqos.data_sharing().on("shared_directory");

        DataWriter* writer = publisher->create_datawriter(topic, wqos);

        std::cout << "LoanableHelloWorld DataWriter created." << std::endl;

        int msgsent = 0;
        void* sample = nullptr;
        // Always call loan_sample() before writing a new sample.
        // This function will provide the user with a pointer to an internal buffer where the data type can be
        // prepared for sending.
        if (RETCODE_OK == writer->loan_sample(sample))
        {
            // Modify the sample data
            LoanableHelloWorld* data = static_cast<LoanableHelloWorld*>(sample);
            data->index() = msgsent + 1;
            memcpy(data->message().data(), "LoanableHelloWorld ", 20);

            std::cout << "Sending sample (count=" << msgsent
                      << ") at address " << &data << std::endl
                      << "  index=" << data->index() << std::endl
                      << "  message=" << data->message().data() << std::endl;

            // Write the sample.
            // After this function returns, the middleware owns the sample.
            writer->write(sample);
        }
        //!--
    }
    {
        class SubListener : public eprosima::fastdds::dds::DataReaderListener
        {
        public:

            SubListener() = default;

            ~SubListener() override = default;

            //LOANABLE_HELLOWORLD_EXAMPLE_LISTENER_READER
            void on_data_available(
                    eprosima::fastdds::dds::DataReader* reader) override
            {
                // Declare a LoanableSequence for a data type
                FASTDDS_SEQUENCE(DataSeq, LoanableHelloWorld);

                DataSeq data;
                SampleInfoSeq infos;
                // Access to the collection of data-samples and its corresponding collection of SampleInfo structures
                while (RETCODE_OK == reader->take(data, infos))
                {
                    // Iterate over each LoanableCollection in the SampleInfo sequence
                    for (LoanableCollection::size_type i = 0; i < infos.length(); ++i)
                    {
                        // Check whether the DataSample contains data or is only used to communicate of a
                        // change in the instance
                        if (infos[i].valid_data)
                        {
                            // Print the data.
                            const LoanableHelloWorld& sample = data[i];

                            ++samples;
                            std::cout << "Sample received (count=" << samples
                                      << ") at address " << &sample
                                      << (reader->is_sample_valid(&sample,
                            &infos[i]) ? " is valid" : " was replaced" ) << std::endl
                                      << "  index=" << sample.index() << std::endl
                                      << "  message=" << sample.message().data() << std::endl;
                        }
                    }
                    // Indicate to the DataReader that the application is done accessing the collection of
                    // data values and SampleInfo, obtained by some earlier invocation of read or take on the
                    // DataReader.
                    reader->return_loan(data, infos);
                }
            }

            //!--

            int matched = 0;
            uint32_t samples = 0;
        }
        datareader_listener;

        //LOANABLE_HELLOWORLD_EXAMPLE_READER
        // CREATE THE PARTICIPANT
        DomainParticipantQos pqos;
        pqos.name("Participant_sub");
        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

        // REGISTER THE TYPE
        TypeSupport type(new LoanableHelloWorldPubSubType());
        type.register_type(participant);

        // CREATE THE SUBSCRIBER
        Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        // CREATE THE TOPIC
        Topic* topic = participant->create_topic(
            "LoanableHelloWorldTopic",
            type.get_type_name(),
            TOPIC_QOS_DEFAULT);

        // CREATE THE READER
        DataReaderQos rqos = subscriber->get_default_datareader_qos();
        rqos.history().depth = 10;
        rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        // DataSharingQosPolicy has to be set to AUTO (the default) or ON to enable Zero-Copy
        rqos.data_sharing().automatic();

        DataReader* reader = subscriber->create_datareader(topic, rqos, &datareader_listener);
        //!--
    }
}

class RequestType
{
};

class ReplyType
{
};

void dds_request_reply_example_client()
{
    class Listener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        //REQUEST_REPLY_EXAMPLE_CLIENT_RECEIVE_REPLY
        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override
        {
            ReplyType reply;
            eprosima::fastdds::dds::SampleInfo sample_info;

            reader->take_next_sample(&reply, &sample_info);

            if (eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == sample_info.instance_state)
            {
                if (sample_info.related_sample_identity == my_request_sample_identity)
                {
                    // Work to do
                }
            }
        }

        //!

        eprosima::fastdds::rtps::SampleIdentity my_request_sample_identity;

    }
    listener;

    eprosima::fastdds::dds::DomainParticipantQos participant_qos;
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_default_participant_qos(participant_qos);

    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);

    TypeSupport request_type;
    TypeSupport reply_type;

    Publisher* publisher = participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
    Subscriber* subscriber = participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);

    //REQUEST_REPLY_EXAMPLE_CLIENT_CREATE_ENTITIES
    participant->register_type(request_type);
    participant->register_type(reply_type);

    Topic* request_topic = participant->create_topic("CalculatorRequest",
                    request_type.get_type_name(), TOPIC_QOS_DEFAULT);

    Topic* reply_topic =
            participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

    DataWriter* request_writer = publisher->create_datawriter(request_topic, DATAWRITER_QOS_DEFAULT);

    DataReader* reply_reader = subscriber->create_datareader(reply_topic, DATAREADER_QOS_DEFAULT, &listener);
    //!

    //REQUEST_REPLY_EXAMPLE_CLIENT_RETRIEVE_ID
    eprosima::fastdds::rtps::SampleIdentity my_request_sample_identity;
    RequestType request;

    // Fill the request

    // Publish request
    eprosima::fastdds::rtps::WriteParams write_params;
    request_writer->write(static_cast<void*>(&request), write_params);

    // Store sample identity
    my_request_sample_identity = write_params.sample_identity();
    //!
}

void dds_request_reply_example_server()
{
    class Listener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        //REQUEST_REPLY_EXAMPLE_SERVER_GET_ID
        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override
        {
            RequestType request;
            eprosima::fastdds::dds::SampleInfo sample_info;

            reader->take_next_sample(&request, &sample_info);

            if (eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == sample_info.instance_state)
            {
                // Store the request identity.
                eprosima::fastdds::rtps::SampleIdentity client_request_identity = sample_info.sample_identity;
            }
        }

        //!
    }
    listener;

    eprosima::fastdds::dds::DomainParticipantQos participant_qos;
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_default_participant_qos(participant_qos);

    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);

    TypeSupport request_type;
    TypeSupport reply_type;

    Publisher* publisher = participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);

    Subscriber* subscriber = participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);

    //REQUEST_REPLY_EXAMPLE_SERVER_CREATE_ENTITIES
    participant->register_type(request_type);
    participant->register_type(reply_type);

    Topic* request_topic = participant->create_topic("CalculatorRequest",
                    request_type.get_type_name(), TOPIC_QOS_DEFAULT);

    Topic* reply_topic =
            participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

    DataWriter* reply_writer = publisher->create_datawriter(reply_topic, DATAWRITER_QOS_DEFAULT);

    DataReader* request_reader = subscriber->create_datareader(request_topic, DATAREADER_QOS_DEFAULT, &listener);
    //!

    eprosima::fastdds::rtps::SampleIdentity client_request_identity;
    //REQUEST_REPLY_EXAMPLE_SERVER_SEND_REPLY
    ReplyType reply;

    // Fill reply

    // Send reply associating it with the client request.
    eprosima::fastdds::rtps::WriteParams write_params;
    write_params.related_sample_identity() = client_request_identity;
    reply_writer->write(reinterpret_cast<void*>(&reply), write_params);
    //!
}

void dds_waitset_example()
{
    auto create_dds_application = [](std::vector<DataReader*>&, std::vector<DataWriter*>&) -> ReturnCode_t
            {
                return RETCODE_OK;
            };

    auto destroy_dds_application = []() -> void
            {
            };

    //DDS_WAITSET_EXAMPLE
    class ApplicationJob
    {
        WaitSet wait_set_;
        GuardCondition terminate_condition_;
        std::thread thread_;

        void main_loop()
        {
            // Main loop is repeated until the terminate condition is triggered
            while (false == terminate_condition_.get_trigger_value())
            {
                // Wait for any of the conditions to be triggered
                ReturnCode_t ret_code;
                ConditionSeq triggered_conditions;
                ret_code = wait_set_.wait(triggered_conditions, eprosima::fastdds::dds::c_TimeInfinite);
                if (RETCODE_OK != ret_code)
                {
                    // ... handle error
                    continue;
                }

                // Process triggered conditions
                for (Condition* cond : triggered_conditions)
                {
                    StatusCondition* status_cond = dynamic_cast<StatusCondition*>(cond);
                    if (nullptr != status_cond)
                    {
                        Entity* entity = status_cond->get_entity();
                        StatusMask changed_statuses = entity->get_status_changes();

                        // Process status. Liveliness changed and data available are depicted as an example
                        if (changed_statuses.is_active(StatusMask::liveliness_changed()))
                        {
                            std::cout << "Liveliness changed reported for entity " <<
                                entity->get_instance_handle() <<
                                std::endl;
                        }

                        if (changed_statuses.is_active(StatusMask::data_available()))
                        {
                            std::cout << "Data avilable on reader " << entity->get_instance_handle() << std::endl;

                            FooSeq data_seq;
                            SampleInfoSeq info_seq;
                            DataReader* reader = static_cast<DataReader*>(entity);

                            // Process all the samples until no one is returned
                            while (RETCODE_OK == reader->take(data_seq, info_seq,
                                    LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                                    ANY_VIEW_STATE, ANY_INSTANCE_STATE))
                            {
                                // Both info_seq.length() and data_seq.length() will have the number of samples returned
                                for (FooSeq::size_type n = 0; n < info_seq.length(); ++n)
                                {
                                    // Only samples with valid data should be accessed
                                    if (info_seq[n].valid_data &&
                                            reader->is_sample_valid(&data_seq[n], &info_seq[n]))
                                    {
                                        // Process sample on data_seq[n]
                                    }
                                }

                                // must return the loaned sequences when done processing
                                reader->return_loan(data_seq, info_seq);
                            }
                        }
                    }
                }
            }
        }

    public:

        ApplicationJob(
                const std::vector<DataReader*>& readers,
                const std::vector<DataWriter*>& writers)
        {
            // Add a GuardCondition, so we can signal the processing thread to stop
            wait_set_.attach_condition(terminate_condition_);

            // Add the status condition of every reader and writer
            for (DataReader* reader : readers)
            {
                wait_set_.attach_condition(reader->get_statuscondition());
            }
            for (DataWriter* writer : writers)
            {
                wait_set_.attach_condition(writer->get_statuscondition());
            }

            thread_ = std::thread(&ApplicationJob::main_loop, this);
        }

        ~ApplicationJob()
        {
            // Signal the GuardCondition to force the WaitSet to wake up
            terminate_condition_.set_trigger_value(true);
            // Wait for the thread to finish
            thread_.join();
        }

    };

    // Application initialization
    ReturnCode_t ret_code;
    std::vector<DataReader*> application_readers;
    std::vector<DataWriter*> application_writers;

    // Create the participant, topics, readers, and writers.
    ret_code = create_dds_application(application_readers, application_writers);
    if (RETCODE_OK != ret_code)
    {
        // ... handle error
        return;
    }

    {
        ApplicationJob main_loop_thread(application_readers, application_writers);

        // ... wait for application termination signaling (signal handler, user input, etc)

        // ... Destructor of ApplicationJob takes care of stopping the processing thread
    }

    // Destroy readers, writers, topics, and participant
    destroy_dds_application();
    //!
}

void tcp_use_cases()
{
    {
        //LARGE_DATA_BUILTIN_TRANSPORTS
        eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

        /* Transports configuration */
        // UDPv4 transport for PDP over multicast and SHM / TCPv4 transport for EDP and application data
        pqos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);

        /* Create participant as usual */
        eprosima::fastdds::dds::DomainParticipant* participant =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        //!
    }

    {
        //LARGE_DATA_BUILTIN_TRANSPORTS_OPTIONS
        eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

        /* Transports configuration */
        // UDPv4 transport for PDP over multicast and SHM / TCPv4 transport for EDP and application data
        // Message Size and Sockets sizes of 200 KB + Non-blocking send + 50ms negotiation timeout
        eprosima::fastdds::rtps::BuiltinTransportsOptions large_data_options;
        large_data_options.maxMessageSize = 200000;
        large_data_options.sockets_buffer_size = 200000;
        large_data_options.non_blocking_send = true;
        large_data_options.tcp_negotiation_timeout = 50;
        pqos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA, large_data_options);

        /* Create participant as usual */
        eprosima::fastdds::dds::DomainParticipant* participant =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        //!
    }

    {
        //PDP-MULTICAST-DATA-TCP
        eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

        /* Transports configuration */
        // UDPv4 transport for PDP over multicast
        auto pdp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        pqos.transport().user_transports.push_back(pdp_transport);

        // TCPv4 transport for EDP and application data (The listening port must to be unique for
        // each participant in the same host)
        constexpr uint16_t tcp_listening_port = 0;
        auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        data_transport->add_listener_port(tcp_listening_port);
        pqos.transport().user_transports.push_back(data_transport);

        pqos.transport().use_builtin_transports = false;

        /* Locators */
        // Define locator for PDP over multicast
        eprosima::fastdds::rtps::Locator_t pdp_locator;
        pdp_locator.kind = LOCATOR_KIND_UDPv4;
        eprosima::fastdds::rtps::IPLocator::setIPv4(pdp_locator, "239.255.0.1");
        pqos.wire_protocol().builtin.metatrafficMulticastLocatorList.push_back(pdp_locator);

        // Define locator for EDP and user data
        eprosima::fastdds::rtps::Locator_t tcp_locator;
        tcp_locator.kind = LOCATOR_KIND_TCPv4;
        eprosima::fastdds::rtps::IPLocator::setIPv4(tcp_locator, "0.0.0.0");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(tcp_locator, tcp_listening_port);
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(tcp_locator, tcp_listening_port);
        pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(tcp_locator);
        pqos.wire_protocol().default_unicast_locator_list.push_back(tcp_locator);

        /* Create participant as usual */
        eprosima::fastdds::dds::DomainParticipant* participant =
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        //!
    }

    {
        //TCP-AND-DISCOVERY-SERVER-SERVER
        eprosima::fastdds::dds::DomainParticipantQos qos = PARTICIPANT_QOS_DEFAULT;

        // Configure the current participant as SERVER
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                eprosima::fastdds::rtps::DiscoveryProtocol::SERVER;

        // Add custom user transport with TCP port 12345
        auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        data_transport->add_listener_port(12345);
        qos.transport().user_transports.push_back(data_transport);

        // Define the listening locator to be on interface 192.168.10.57 and port 12345
        constexpr uint16_t tcp_listening_port = 12345;
        eprosima::fastdds::rtps::Locator_t listening_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(listening_locator, "192.168.10.57");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(listening_locator, tcp_listening_port);
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(listening_locator, tcp_listening_port);
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(listening_locator);
        //!--
    }

    {
        //TCP-AND-DISCOVERY-SERVER-CLIENT
        eprosima::fastdds::dds::DomainParticipantQos qos = PARTICIPANT_QOS_DEFAULT;

        // Configure the current participant as SERVER
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                eprosima::fastdds::rtps::DiscoveryProtocol::CLIENT;

        // Add custom user transport with TCP port 0 (automatic port assignation)
        auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        data_transport->add_listener_port(0);
        qos.transport().user_transports.push_back(data_transport);

        // Define the server locator to be on interface 192.168.10.57 and port 12345
        constexpr uint16_t server_port = 12345;
        eprosima::fastdds::rtps::Locator_t server_locator;
        eprosima::fastdds::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
        eprosima::fastdds::rtps::IPLocator::setPhysicalPort(server_locator, server_port);
        eprosima::fastdds::rtps::IPLocator::setLogicalPort(server_locator, server_port);

        // Add the server
        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server_locator);
        //!--
    }
}

bool dds_permissions_test(
        std::string main_ca_file,
        std::string appcert_file,
        std::string appkey_file,
        std::string governance_file,
        std::string permissions_file)
{
    DomainParticipantQos pqos;

    // Activate Auth:PKI-DH plugin
    pqos.properties().properties().emplace_back("dds.sec.auth.plugin",
            "builtin.PKI-DH");

    // Configure Auth:PKI-DH plugin
    pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
            main_ca_file);
    pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
            appcert_file);
    pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key",
            appkey_file);

    pqos.properties().properties().emplace_back("dds.sec.access.plugin",
            "builtin.Access-Permissions");

    // Configure DDS:Access:Permissions plugin
    pqos.properties().properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions_ca",
        main_ca_file);
    pqos.properties().properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.governance",
        governance_file);
    pqos.properties().properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions",
        permissions_file);

    DomainParticipant* domain_participant =
            DomainParticipantFactory::get_instance()->create_participant(1, pqos);
    if (nullptr != domain_participant)
    {
        return true;
    }
    return false;
}

bool dds_rosbag_example()
{
    //CREATE THE PARTICIPANT
    DomainParticipant* participant_;
    Topic* topic_;
    TypeSupport type_;

    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);

    //CREATE THE TOPIC FOR ROSBAG
    topic_ = participant_->create_topic(
        "rt/HelloWorldTopic",
        type_.get_type_name(),
        TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr)
    {
        return false;
    }
    //!
    return true;
}

void pubsub_api_example_create_entities()
{
    //PUBSUB_API_CREATE_PARTICIPANT
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    //!--

    //PUBSUB_API_CREATE_DEFAULT_PARTICIPANT
    DomainParticipant* default_participant =
            DomainParticipantFactory::get_instance()->create_participant_with_default_profile();
    //!--

    //PUBSUB_API_CREATE_PUBLISHER
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    //!--

    //CREATE TOPIC
    Topic* custom_topic =
            participant->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);
    //!--

    //PUBSUB_API_CREATE_DATAWRITER
    DataWriter* data_writer =
            publisher->create_datawriter(custom_topic, DATAWRITER_QOS_DEFAULT);
    //!--

    //PUBSUB_API_WRITE_SAMPLE
    HelloWorld sample;     //Auto-generated container class for topic data from Fast DDS-Gen
    sample.msg("Hello there!");     // Add contents to the message
    data_writer->write(&sample);     //Publish
    //!--

    //PUBSUB_API_CREATE_SUBSCRIBER
    Subscriber* subscriber = participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
    //!--
}

int main(
        int argc,
        const char** argv)
{
    // Also show log warnings to spot potential mistakes
    Log::SetVerbosity(Log::Kind::Warning);
    // Report filename and line number for debugging
    Log::ReportFilenames(true);

    int exit_code = 0;
    if (argc == 6)
    {
        if (!dds_permissions_test(argv[1], argv[2], argv[3], argv[4], argv[5]))
        {
            std::cout << "Error parsing permissions xml file" << std::endl;
            exit_code = -1;
        }
    }
    else if (argc == 2)
    {
        if (strncmp(argv[1], "Static", 6) == 0)
        {
            std::string file = argv[1];
            DomainParticipantFactory* factory = DomainParticipantFactory::get_instance();
            if (RETCODE_OK != factory->check_xml_static_discovery(file))
            {
                printf("Error parsing xml file %s\n", argv[1]);
                exit_code = -1;
            }

            std::string fileData = "data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
                    "<staticdiscovery>" \
                    "<participant>" \
                    "<name>HelloWorldPublisher</name>" \
                    "<writer>" \
                    "<userId>1</userId>" \
                    "<entityID>2</entityID>" \
                    "<topicName>HelloWorldTopic</topicName>" \
                    "<topicDataType>HelloWorld</topicDataType>" \
                    "</writer>" \
                    "</participant>" \
                    "</staticdiscovery>";
            if (RETCODE_OK != factory->check_xml_static_discovery(fileData))
            {
                printf("Error parsing xml file %s\n", argv[1]);
                exit_code = -1;
            }
        }
        else
        {
            if (RETCODE_OK != DomainParticipantFactory::get_instance()->load_XML_profiles_file(argv[1]))
            {
                printf("Error parsing xml file %s\n", argv[1]);
                exit_code = -1;
            }
        }
    }
    else
    {
        printf("Bad number of parameters\n");
        exit_code = -1;
    }

    exit(exit_code);
}
