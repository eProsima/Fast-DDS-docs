#include <fastdds/dds/core/condition/GuardCondition.hpp>
#include <fastdds/dds/core/condition/WaitSet.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/PublisherListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/SubscriberListener.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/SubscriberListener.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/topic/TopicListener.hpp>
#include <fastrtps/xmlparser/XMLProfileManager.h>

#include <fastrtps/types/DynamicTypePtr.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/log/OStreamConsumer.hpp>
#include <fastdds/dds/log/StdoutConsumer.hpp>
#include <fastdds/dds/log/StdoutErrConsumer.hpp>
#include <fastdds/dds/log/FileConsumer.hpp>

#include <fastdds/rtps/transport/TCPTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/TCPv6TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/ChainingTransportDescriptor.h>
#include <fastdds/rtps/transport/ChainingTransport.h>

#include <fastdds/statistics/dds/domain/DomainParticipant.hpp>
#include <fastdds/statistics/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/statistics/topic_names.hpp>

#include <fastrtps/utils/IPLocator.h>

#include <fastcdr/Cdr.h>

#include <sstream>

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
            eprosima::fastrtps::rtps::SenderResource* low_sender_resource,
            const eprosima::fastrtps::rtps::octet* send_buffer,
            uint32_t send_buffer_size,
            eprosima::fastrtps::rtps::LocatorsIterator* destination_locators_begin,
            eprosima::fastrtps::rtps::LocatorsIterator* destination_locators_end,
            const std::chrono::steady_clock::time_point& timeout) override
    {
        //
        // Preprocess outcoming buffer.
        //

        // Call low level transport
        return low_sender_resource->send(send_buffer, send_buffer_size, destination_locators_begin,
                       destination_locators_end, timeout);
    }

    void receive(
            eprosima::fastdds::rtps::TransportReceiverInterface* next_receiver,
            const eprosima::fastrtps::rtps::octet* receive_buffer,
            uint32_t receive_buffer_size,
            const eprosima::fastrtps::rtps::Locator_t& local_locator,
            const eprosima::fastrtps::rtps::Locator_t& remote_locator) override
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
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info,
            bool& should_be_ignored) override
    {
        should_be_ignored = false;
        if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
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
        else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
                info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT)
        {
            std::cout << "New participant lost" << std::endl;
        }
    }

#if HAVE_SECURITY
    void onParticipantAuthentication(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantAuthenticationInfo&& info) override
    {
        if (info.status == eprosima::fastrtps::rtps::ParticipantAuthenticationInfo::AUTHORIZED_PARTICIPANT)
        {
            std::cout << "A participant was authorized" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ParticipantAuthenticationInfo::UNAUTHORIZED_PARTICIPANT)
        {
            std::cout << "A participant failed authorization" << std::endl;
        }
    }

#endif // if HAVE_SECURITY

    void on_subscriber_discovery(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override
    {
        if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            std::cout << "New subscriber discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        {
            std::cout << "New subscriber lost" << std::endl;
        }
    }

    void on_publisher_discovery(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info) override
    {
        if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        {
            std::cout << "New publisher discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        {
            std::cout << "New publisher lost" << std::endl;
        }
    }

    void on_type_discovery(
            DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::string_255& topic,
            const eprosima::fastrtps::types::TypeIdentifier* identifier,
            const eprosima::fastrtps::types::TypeObject* object,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type) override
    {
        static_cast<void>(participant);
        static_cast<void>(request_sample_id);
        static_cast<void>(topic);
        static_cast<void>(identifier);
        static_cast<void>(object);
        static_cast<void>(dyn_type);
        std::cout << "New data type discovered" << std::endl;

    }

    void on_type_dependencies_reply(
            DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq& dependencies) override
    {
        static_cast<void>(participant);
        static_cast<void>(request_sample_id);
        static_cast<void>(dependencies);
        std::cout << "Answer to a request for type dependencies was received" << std::endl;
    }

    void on_type_information_received(
            DomainParticipant* participant,
            const eprosima::fastrtps::string_255 topic_name,
            const eprosima::fastrtps::string_255 type_name,
            const eprosima::fastrtps::types::TypeInformation& type_information) override
    {
        static_cast<void>(participant);
        static_cast<void>(topic_name);
        static_cast<void>(type_name);
        static_cast<void>(type_information);
        std::cout << "New data type information received" << std::endl;
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
        if (participant->set_qos(PARTICIPANT_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (participant->set_qos(DomainParticipantFactory::get_instance()->get_default_participant_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (participant->delete_contained_entities() != ReturnCode_t::RETCODE_OK)
        {
            // DomainParticipant failed to delete the entities it created.
            return;
        }

        // Delete the DomainParticipant
        if (DomainParticipantFactory::get_instance()->delete_participant(participant) != ReturnCode_t::RETCODE_OK)
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
                ReturnCode_t::RETCODE_OK)
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
                ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (DomainParticipantFactory::get_instance()->set_default_participant_qos(DomainParticipantQos())
                != ReturnCode_t::RETCODE_OK)
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
        if (DomainParticipantFactory::get_instance()->set_qos(qos) != ReturnCode_t::RETCODE_OK)
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
        if (DomainParticipantFactory::get_instance()->set_qos(qos) != ReturnCode_t::RETCODE_OK)
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
                eprosima::fastdds::statistics::dds::STATISTICS_DATAWRITER_QOS) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Use the DomainParticipant to communicate
        // (...)

        // Disable statistics DataWriter
        if (statistics_participant->disable_statistics_datawriter(eprosima::fastdds::statistics::GAP_COUNT_TOPIC) !=
                ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Delete DomainParticipant
        if (DomainParticipantFactory::get_instance()->delete_participant(participant) != ReturnCode_t::RETCODE_OK)
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
}

//DOMAINPARTICIPANTLISTENER-DISCOVERY-CALLBACKS
class DiscoveryDomainParticipantListener : public DomainParticipantListener
{
    /* Custom Callback on_participant_discovery */
    void on_participant_discovery(
            DomainParticipant* participant,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) override
    {
        static_cast<void>(participant);
        switch (info.status){
            case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT:
                /* Process the case when a new DomainParticipant was found in the domain */
                std::cout << "New DomainParticipant '" << info.info.m_participantName <<
                    "' with ID '" << info.info.m_guid.entityId << "' and GuidPrefix '" <<
                    info.info.m_guid.guidPrefix << "' discovered." << std::endl;
                break;
            case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::CHANGED_QOS_PARTICIPANT:
                /* Process the case when a DomainParticipant changed its QOS */
                break;
            case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT:
                /* Process the case when a DomainParticipant was removed from the domain */
                std::cout << "New DomainParticipant '" << info.info.m_participantName <<
                    "' with ID '" << info.info.m_guid.entityId << "' and GuidPrefix '" <<
                    info.info.m_guid.guidPrefix << "' left the domain." << std::endl;
                break;
        }
    }

    /* Custom Callback on_subscriber_discovery */
    void on_subscriber_discovery(
            DomainParticipant* participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override
    {
        static_cast<void>(participant);
        switch (info.status){
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER:
                /* Process the case when a new subscriber was found in the domain */
                std::cout << "New DataReader subscribed to topic '" << info.info.topicName() <<
                    "' of type '" << info.info.typeName() << "' discovered";
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER:
                /* Process the case when a subscriber changed its QOS */
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER:
                /* Process the case when a subscriber was removed from the domain */
                std::cout << "New DataReader subscribed to topic '" << info.info.topicName() <<
                    "' of type '" << info.info.typeName() << "' left the domain.";
                break;
        }
    }

    /* Custom Callback on_publisher_discovery */
    void on_publisher_discovery(
            DomainParticipant* participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info) override
    {
        static_cast<void>(participant);
        switch (info.status){
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER:
                /* Process the case when a new publisher was found in the domain */
                std::cout << "New DataWriter publishing under topic '" << info.info.topicName() <<
                    "' of type '" << info.info.typeName() << "' discovered";
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER:
                /* Process the case when a publisher changed its QOS */
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER:
                /* Process the case when a publisher was removed from the domain */
                std::cout << "New DataWriter publishing under topic '" << info.info.topicName() <<
                    "' of type '" << info.info.typeName() << "' left the domain.";
                break;
        }
    }

    /* Custom Callback on_type_discovery */
    void on_type_discovery(
            DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::string_255& topic,
            const eprosima::fastrtps::types::TypeIdentifier* identifier,
            const eprosima::fastrtps::types::TypeObject* object,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type) override
    {
        static_cast<void>(participant);
        static_cast<void>(request_sample_id);
        static_cast<void>(topic);
        static_cast<void>(identifier);
        static_cast<void>(object);
        static_cast<void>(dyn_type);
        std::cout << "New data type of topic '" << topic << "' discovered." << std::endl;
    }

};
//!--

void dds_discovery_examples()
{
    using Locator_t = eprosima::fastrtps::rtps::Locator_t;
    using RemoteServerAttributes = eprosima::fastrtps::rtps::RemoteServerAttributes;
    using IPLocator = eprosima::fastrtps::rtps::IPLocator;
    using DiscoveryProtocol_t = eprosima::fastrtps::rtps::DiscoveryProtocol_t;
    using ParticipantFilteringFlags_t = eprosima::fastrtps::rtps::ParticipantFilteringFlags_t;
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
                DiscoveryProtocol_t::SIMPLE;
        //!--
    }
    {
        //CONF-DISCOVERY-IGNORE-FLAGS
        DomainParticipantQos pqos;

        pqos.wire_protocol().builtin.discovery_config.ignoreParticipantFlags =
                static_cast<eprosima::fastrtps::rtps::ParticipantFilteringFlags_t>(
            ParticipantFilteringFlags_t::FILTER_DIFFERENT_PROCESS |
            ParticipantFilteringFlags_t::FILTER_SAME_PROCESS);
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
                DiscoveryProtocol_t::CLIENT;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol_t::SUPER_CLIENT;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol_t::SERVER;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
                DiscoveryProtocol_t::BACKUP;
        //!--
    }
    {
        //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_1
        eprosima::fastrtps::rtps::GuidPrefix_t serverGuidPrefix;
        serverGuidPrefix.value[0] = eprosima::fastrtps::rtps::octet(0x44);
        serverGuidPrefix.value[1] = eprosima::fastrtps::rtps::octet(0x53);
        serverGuidPrefix.value[2] = eprosima::fastrtps::rtps::octet(0x00);
        serverGuidPrefix.value[3] = eprosima::fastrtps::rtps::octet(0x5f);
        serverGuidPrefix.value[4] = eprosima::fastrtps::rtps::octet(0x45);
        serverGuidPrefix.value[5] = eprosima::fastrtps::rtps::octet(0x50);
        serverGuidPrefix.value[6] = eprosima::fastrtps::rtps::octet(0x52);
        serverGuidPrefix.value[7] = eprosima::fastrtps::rtps::octet(0x4f);
        serverGuidPrefix.value[8] = eprosima::fastrtps::rtps::octet(0x53);
        serverGuidPrefix.value[9] = eprosima::fastrtps::rtps::octet(0x49);
        serverGuidPrefix.value[10] = eprosima::fastrtps::rtps::octet(0x4d);
        serverGuidPrefix.value[11] = eprosima::fastrtps::rtps::octet(0x41);

        DomainParticipantQos serverQos;
        serverQos.wire_protocol().prefix = serverGuidPrefix;
        //!--
    }
    {
        //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_2
        DomainParticipantQos serverQos;
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> serverQos.wire_protocol().prefix;
        //!--
    }
    {
        //CONF_SERVER_CLIENT_GUIDPREFIX
        RemoteServerAttributes server;
        server.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");

        DomainParticipantQos clientQos;
        clientQos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server);
        //!--
    }
    {
        //CONF_SERVER_CLIENT_LOCATORS
        Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 1, 133);
        locator.port = 64863;
        RemoteServerAttributes server;
        server.metatrafficUnicastLocatorList.push_back(locator);

        DomainParticipantQos clientQos;
        clientQos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server);
        //!--
    }

    {
        //CONF_SERVER_SERVER_LOCATORS
        Locator_t locator;
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
        RemoteServerAttributes remote_server_att;

        // Set server's GUID prefix
        remote_server_att.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.42");

        // Set server's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11812;
        remote_server_att.metatrafficUnicastLocatorList.push_back(locator);

        /* Update list of remote servers for this client or server */
        client_or_server_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_att);
        if (ReturnCode_t::RETCODE_OK != client_or_server->set_qos(client_or_server_qos))
        {
            // Error
            return;
        }
        //!--
    }

    {
        //CONF_SERVER_DNS_LOCATORS
        Locator_t locator;
        auto response = eprosima::fastrtps::rtps::IPLocator::resolveNameDNS("localhost");
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
                DiscoveryProtocol_t::SERVER;

        // Set SERVER's GUID prefix
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> server_qos.wire_protocol().prefix;

        // Set SERVER's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11811;
        server_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);

        /* Add a remote serve to which this server will connect */
        // Set remote SERVER's GUID prefix
        RemoteServerAttributes remote_server_att;
        remote_server_att.ReadguidPrefix("44.53.01.5f.45.50.52.4f.53.49.4d.41");

        // Set remote SERVER's listening locator for PDP
        Locator_t remote_locator;
        IPLocator::setIPv4(remote_locator, 127, 0, 0, 1);
        remote_locator.port = 11812;
        remote_server_att.metatrafficUnicastLocatorList.push_back(remote_locator);

        // Add remote SERVER to SERVER's list of SERVERs
        server_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_att);

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
                DiscoveryProtocol_t::CLIENT;

        // Set SERVER's GUID prefix
        RemoteServerAttributes remote_server_att;
        remote_server_att.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");

        // Set SERVER's listening locator for PDP
        Locator_t locator;
        IPLocator::setIPv4(locator, 127, 0, 0, 1);
        locator.port = 11811;
        remote_server_att.metatrafficUnicastLocatorList.push_back(locator);

        // Add remote SERVER to CLIENT's list of SERVERs
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_att);

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
        if (ReturnCode_t::RETCODE_OK != factory->check_xml_static_discovery(file))
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
        if (ReturnCode_t::RETCODE_OK != factory->check_xml_static_discovery(fileData))
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
        setName("Foo");
    }

    bool serialize(
            void* /*data*/,
            eprosima::fastrtps::rtps::SerializedPayload_t* /*payload*/) override
    {
        return true;
    }

    bool deserialize(
            eprosima::fastrtps::rtps::SerializedPayload_t* /*payload*/,
            void* /*data*/) override
    {
        return true;
    }

    std::function<uint32_t()> getSerializedSizeProvider(
            void* /*data*/) override
    {
        return std::function<uint32_t()>();
    }

    void* createData() override
    {
        return nullptr;
    }

    void deleteData(
            void* /*data*/) override
    {
    }

    bool getKey(
            void* /*data*/,
            eprosima::fastrtps::rtps::InstanceHandle_t* /*ihandle*/,
            bool /*force_md5*/) override
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
        if (topic->set_qos(TOPIC_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (topic->set_qos(participant->get_default_topic_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (participant->delete_topic(topic) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_topic_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_topic_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_topic_qos(TopicQos())
                != ReturnCode_t::RETCODE_OK)
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
        eprosima::fastrtps::xmlparser::XMLProfileManager::loadXMLFile("example_type.xml");

        // Retrieve the an instance of the desired type and register it
        eprosima::fastrtps::types::DynamicType_ptr dyn_type =
                eprosima::fastrtps::xmlparser::XMLProfileManager::getDynamicTypeByName("DynamicType")->build();
        TypeSupport dyn_type_support(new eprosima::fastrtps::types::DynamicPubSubType(dyn_type));
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
        if (ReturnCode_t::RETCODE_OK !=
                filter_topic->set_filter_expression("message like %0 or index > %1", {"'Hello*'", "15"}))
        {
            // Error
            return;
        }

        // Print the updated information
        print_filter_info(filter_topic);

        // Update the parameters
        if (ReturnCode_t::RETCODE_OK !=
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
        if (ReturnCode_t::RETCODE_OK != participant->delete_contentfilteredtopic(filter_topic))
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
            eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                    eprosima::fastcdr::Cdr::DDS_CDR);
            // Deserialize encapsulation.
            deser.read_encapsulation();
            int index = 0;

            // Deserialize `index` field.
            try
            {
                deser >> index;
            }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
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
                return ReturnCode_t::RETCODE_BAD_PARAMETER;
            }

            // Check the ContentFilteredTopic is created for the unique type this Custom Filter supports.
            if (0 != strcmp(type_name, "HelloWorld"))
            {
                return ReturnCode_t::RETCODE_BAD_PARAMETER;
            }

            // Check that the two mandatory filter parameters are set.
            if (2 != filter_parameters.length())
            {
                return ReturnCode_t::RETCODE_BAD_PARAMETER;
            }

            // If there is an update, delete previous instance.
            if (nullptr != filter_instance)
            {
                delete(dynamic_cast<MyCustomFilter*>(filter_instance));
            }

            // Instantiation of the Custom Filter.
            filter_instance = new MyCustomFilter(std::stoi(filter_parameters[0]), std::stoi(filter_parameters[1]));

            return ReturnCode_t::RETCODE_OK;
        }

        ReturnCode_t delete_content_filter(
                const char* filter_class_name,
                IContentFilter* filter_instance) override
        {
            // Check the ContentFilteredTopic should be created by my factory.
            if (0 != strcmp(filter_class_name, "MY_CUSTOM_FILTER"))
            {
                return ReturnCode_t::RETCODE_BAD_PARAMETER;
            }

            // Deletion of the Custom Filter.
            delete(dynamic_cast<MyCustomFilter*>(filter_instance));

            return ReturnCode_t::RETCODE_OK;
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
        if (ReturnCode_t::RETCODE_OK !=
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
        if (publisher->set_qos(PUBLISHER_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (publisher->set_qos(participant->get_default_publisher_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (publisher->delete_contained_entities() != ReturnCode_t::RETCODE_OK)
        {
            // Publisher failed to delete the entities it created.
            return;
        }

        // Delete the Publisher
        if (participant->delete_publisher(publisher) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_publisher_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_publisher_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_publisher_qos(PublisherQos())
                != ReturnCode_t::RETCODE_OK)
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
            DataWriter* /*writer*/,
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
        if (data_writer->set_qos(DATAWRITER_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (data_writer->set_qos(publisher->get_default_datawriter_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (publisher->delete_datawriter(data_writer) != ReturnCode_t::RETCODE_OK)
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
        if (publisher->set_default_datawriter_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
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
        if (publisher->set_default_datawriter_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (publisher->set_default_datawriter_qos(DataWriterQos())
                != ReturnCode_t::RETCODE_OK)
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
        void* data = custom_type_support->createData();

        // Fill the data values
        // (...)

        // Publish the new value, deduce the instance handle
        if (data_writer->write(data, eprosima::fastrtps::rtps::InstanceHandle_t()) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The data instance can be reused to publish new values,
        // but delete it at the end to avoid leaks
        custom_type_support->deleteData(data);
        //!--

        {
            //DDS_DATAWRITER_LOAN_SAMPLES
            // Borrow a data instance
            void* data = nullptr;
            if (ReturnCode_t::RETCODE_OK == data_writer->loan_sample(data))
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
                if (data_writer->write(data, eprosima::fastrtps::rtps::InstanceHandle_t()) != ReturnCode_t::RETCODE_OK)
                {
                    // Error
                    return;
                }
            }

            // The data instance can be reused to publish new values,
            // but delete it at the end to avoid leaks
            custom_type_support->deleteData(data);
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
            eprosima::fastrtps::rtps::InstanceHandle_t first_flight_handle =
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
        if (subscriber->set_qos(SUBSCRIBER_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (subscriber->set_qos(participant->get_default_subscriber_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (subscriber->delete_contained_entities() != ReturnCode_t::RETCODE_OK)
        {
            // Subscriber failed to delete the entities it created
            return;
        }

        // Delete the Subscriber
        if (participant->delete_subscriber(subscriber) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_subscriber_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
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
        if (participant->set_default_subscriber_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (participant->set_default_subscriber_qos(SubscriberQos())
                != ReturnCode_t::RETCODE_OK)
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
            const eprosima::fastrtps::RequestedDeadlineMissedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "Some data was not received on time" << std::endl;
    }

    void on_liveliness_changed(
            DataReader* reader,
            const eprosima::fastrtps::LivelinessChangedStatus& info) override
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
            const eprosima::fastrtps::SampleRejectedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "A received data sample was rejected" << std::endl;
    }

    void on_requested_incompatible_qos(
            DataReader* /*reader*/,
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
        if (data_reader->set_qos(DATAREADER_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following:
        if (data_reader->set_qos(subscriber->get_default_datareader_qos())
                != ReturnCode_t::RETCODE_OK)
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
        if (data_reader->delete_contained_entities() != ReturnCode_t::RETCODE_OK)
        {
            // DataReader failed to delete the entities it created.
            return;
        }

        // Delete the DataReader
        if (subscriber->delete_datareader(data_reader) != ReturnCode_t::RETCODE_OK)
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
        if (subscriber->set_default_datareader_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
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
        if (subscriber->set_default_datareader_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
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
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if (subscriber->set_default_datareader_qos(DataReaderQos())
                != ReturnCode_t::RETCODE_OK)
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
        eprosima::fastrtps::Duration_t timeout (5, 0);

        // Loop reading data as it arrives
        // This will make the current thread to be dedicated exclusively to
        // waiting and reading data until the remote DataWriter dies
        while (true)
        {
            ConditionSeq active_conditions;
            if (ReturnCode_t::RETCODE_OK == wait_set.wait(active_conditions, timeout))
            {
                while (ReturnCode_t::RETCODE_OK == data_reader->take_next_sample(&data, &info))
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
        eprosima::fastrtps::Duration_t timeout (5, 0);

        // Loop reading data as it arrives
        // This will make the current thread to be dedicated exclusively to
        // waiting and reading data until the remote DataWriter dies
        while (true)
        {
            if (data_reader->wait_for_unread_message(timeout))
            {
                if (ReturnCode_t::RETCODE_OK == data_reader->take_next_sample(&data, &info))
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
            if (ReturnCode_t::RETCODE_OK == data_reader->take_next_sample(&data, &info))
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
            if (ret_code == ReturnCode_t::RETCODE_OK)
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
        while (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK)
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
    {
        //DDS_CHANGE_DEADLINE_QOS_POLICY
        DeadlineQosPolicy deadline;
        //The DeadlineQosPolicy is default constructed with an infinite period.
        //Change the period to 1 second
        deadline.period.seconds = 1;
        deadline.period.nanosec = 0;
        //!--
    }

    {
        //DDS_CHANGE_DURABILITY_QOS_POLICY
        DurabilityQosPolicy durability;
        //The DurabilityQosPolicy is default constructed with kind = VOLATILE_DURABILITY_QOS
        //Change the kind to TRANSIENT_LOCAL_DURABILITY_QOS
        durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        //!--
    }

    {
        //DDS_CHANGE_ENTITY_FACTORY_QOS_POLICY
        EntityFactoryQosPolicy entity_factory;
        //The EntityFactoryQosPolicy is default constructed with autoenable_created_entities = true
        //Change it to false
        entity_factory.autoenable_created_entities = false;
        //!--
    }

    {
        //DDS_CHANGE_GROUP_DATA_QOS_POLICY
        GroupDataQosPolicy group_data;
        //The GroupDataQosPolicy is default constructed with an empty collection
        //Collection is a private member so you need to use getters and setters to access
        //Add data to the collection
        std::vector<eprosima::fastrtps::rtps::octet> vec;
        vec = group_data.data_vec(); // Getter function

        //Add two new octets to group data vector
        eprosima::fastrtps::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        group_data.data_vec(vec); //Setter function
        //!--
    }

    {
        //DDS_CHANGE_HISTORY_QOS_POLICY
        HistoryQosPolicy history;
        //The HistoryQosPolicy is default constructed with kind = KEEP_LAST and depth = 1.
        //Change the depth to 20
        history.depth = 20;
        //You can also change the kind to KEEP_ALL but after that the depth will not have effect.
        history.kind = KEEP_ALL_HISTORY_QOS;
        //!--
    }

    {
        //DDS_CHANGE_LIFESPAN_QOS_POLICY
        LifespanQosPolicy lifespan;
        //The LifespanQosPolicy is default constructed with duration set to infinite.
        //Change the duration to 5 s
        lifespan.duration = {5, 0};
        //!--
    }

    {
        //DDS_CHANGE_LIVELINESS_QOS_POLICY
        LivelinessQosPolicy liveliness;
        //The LivelinessQosPolicy is default constructed with kind = AUTOMATIC
        //Change the kind to MANUAL_BY_PARTICIPANT
        liveliness.kind = MANUAL_BY_PARTICIPANT_LIVELINESS_QOS;
        //The LivelinessQosPolicy is default constructed with lease_duration set to infinite
        //Change the lease_duration to 1 second
        liveliness.lease_duration = {1, 0};
        //The LivelinessQosPolicy is default constructed with announcement_period set to infinite
        //Change the announcement_period to 1 ms
        liveliness.announcement_period = {0, 1000000};
        //!--
    }

    {
        //DDS_CHANGE_OWNERSHIP_QOS_POLICY
        OwnershipQosPolicy ownership;
        //The OwnershipQosPolicy is default constructed with kind = SHARED.
        //Change the kind to EXCLUSIVE
        ownership.kind = EXCLUSIVE_OWNERSHIP_QOS;
        //!--
    }

    {
        //DDS_CHANGE_OWNERSHIP_STRENGTH_QOS_POLICY
        OwnershipStrengthQosPolicy ownership_strength;
        //The OwnershipStrengthQosPolicy is default constructed with value 0
        //Change the strength to 10
        ownership_strength.value = 10;
        //!--
    }

    {
        //DDS_CHANGE_PARTITION_QOS_POLICY
        PartitionQosPolicy partitions;
        //The PartitionsQosPolicy is default constructed with max_size = 0.
        //Max_size is a private member so you need to use getters and setters to access
        //Change the max_size to 20
        partitions.set_max_size(20); //Setter function
        //The PartitionsQosPolicy is default constructed with an empty list of partitions
        //Partitions is a private member so you need to use getters and setters to access
        //Add new partitions
        std::vector<std::string> part = partitions.names(); //Getter function
        part.push_back("part1");
        part.push_back("part2");
        partitions.names(part); //Setter function
        //!--
    }

    {
        //DDS_CHANGE_RELIABILITY_QOS_POLICY
        ReliabilityQosPolicy reliability;
        //The ReliabilityQosPolicy is default constructed with kind = BEST_EFFORT
        //Change the kind to RELIABLE
        reliability.kind = RELIABLE_RELIABILITY_QOS;
        //The ReliabilityQosPolicy is default constructed with max_blocking_time = 100ms
        //Change the max_blocking_time to 1s
        reliability.max_blocking_time = {1, 0};
        //!--
    }

    {
        //DDS_CHANGE_RESOURCE_LIMITS_QOS_POLICY
        ResourceLimitsQosPolicy resource_limits;
        //The ResourceLimitsQosPolicy is default constructed with max_samples = 5000
        //Change max_samples to 200
        resource_limits.max_samples = 200;
        //The ResourceLimitsQosPolicy is default constructed with max_instances = 10
        //Change max_instances to 20
        resource_limits.max_instances = 20;
        //The ResourceLimitsQosPolicy is default constructed with max_samples_per_instance = 400
        //Change max_samples_per_instance to 100 as it must be lower than max_samples
        resource_limits.max_samples_per_instance = 100;
        //The ResourceLimitsQosPolicy is default constructed with allocated_samples = 100
        //Change allocated_samples to 50
        resource_limits.allocated_samples = 50;
        //!--
    }

    {
        //DDS_CHANGE_TOPIC_DATA_QOS_POLICY
        //The TopicDataQosPolicy is default constructed with an empty vector.
        TopicDataQosPolicy topic_data;
        std::vector<eprosima::fastrtps::rtps::octet> vec;
        vec = topic_data.data_vec(); // Getter Function

        //Add two new octets to topic data vector
        eprosima::fastrtps::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        topic_data.data_vec(vec); //Setter Function
        //!--
    }


    {
        //DDS_CHANGE_USER_DATA_QOS_POLICY
        //The TopicDataQosPolicy is default constructed with an empty vector.
        UserDataQosPolicy user_data;
        std::vector<eprosima::fastrtps::rtps::octet> vec;
        vec = user_data.data_vec(); // Getter Function

        //Add two new octets to user data vector
        eprosima::fastrtps::rtps::octet val = 3;
        vec.push_back(val);
        val = 10;
        vec.push_back(val);
        user_data.data_vec(vec); //Setter Function
        //!--
    }

    {
        //DDS_CHANGE_DISABLE_POSITIVE_ACKS_QOS_POLICY
        DisablePositiveACKsQosPolicy disable_acks;
        //The DisablePositiveACKsQosPolicy is default constructed with enabled = false
        //Change enabled to true
        disable_acks.enabled = true;
        //The DisablePositiveACKsQosPolicy is default constructed with infinite duration
        //Change the duration to 1 second
        disable_acks.duration = {1, 0};

        //!--
    }

    {
        //DDS_CHANGE_DATASHARING_QOS_POLICY
        DataSharingQosPolicy datasharing;

        // Configure the DataSharing as AUTO with two user-defined IDs
        std::vector<uint16_t> ids;
        ids.push_back(0x1234);
        ids.push_back(0xABCD);
        datasharing.automatic(ids);

        // Alternatively, configure with no IDs and add them afterwards
        datasharing.automatic();
        datasharing.add_domain_id(uint16_t(0x1234));
        datasharing.add_domain_id(uint16_t(0xABCD));

        // Or you can leave the IDs empty and the system will create one for you
        // unique for the current machine
        datasharing.automatic();

        //!--
    }

    {
        //DDS_CHANGE_PARTICIPANT_RESOURCE_LIMITS_QOS_POLICY
        ParticipantResourceLimitsQos participant_limits;
        //Set the maximum size of participant resource limits collection to 3 and it allocation configuration to fixed size
        participant_limits.participants = eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(
            3u);
        //Set the maximum size of reader's resource limits collection to 2 and its allocation configuration to fixed size
        participant_limits.readers = eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        //Set the maximum size of writer's resource limits collection to 1 and its allocation configuration to fixed size
        participant_limits.writers = eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //Set the maximum size of the partition data to 256
        participant_limits.data_limits.max_partitions = 256u;
        //Set the maximum size of the user data to 256
        participant_limits.data_limits.max_user_data = 256u;
        //Set the maximum size of the properties data to 512
        participant_limits.data_limits.max_properties = 512u;
        //Set the preallocated filter expression size to 512
        participant_limits.content_filter.expression_initial_size = 512u;
        //Set the maximum number of expression parameters to 4 and its allocation configuration to fixed size
        participant_limits.content_filter.expression_parameters =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
        //!--
    }

    {
        //DDS_CHANGE_PROPERTY_POLICY_QOS
        PropertyPolicyQos property_policy;
        //Add new property for the Auth:PKI-DH plugin
        property_policy.properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");
        //Add new property for the Access:Permissions plugin
        property_policy.properties().emplace_back(eprosima::fastrtps::rtps::Property("dds.sec.access.plugin",
                "builtin.Access-Permissions"));

        //Add new user custom property to send to external Participants
        property_policy.properties().emplace_back("Custom Property Name", "Custom value", true);
        //!--
    }

    {
        //DDS_CHANGE_PUBLISH_MODE_QOS
        PublishModeQosPolicy publish_mode;
        //The PublishModeQosPolicy is default constructed with kind = SYNCHRONOUS
        //Change the kind to ASYNCHRONOUS
        publish_mode.kind = ASYNCHRONOUS_PUBLISH_MODE;
        //!--
    }

    {
        //DDS_CHANGE_READER_RESOURCE_LIMITS_QOS
        ReaderResourceLimitsQos reader_limits;
        //Set the maximum size for writer matched resource limits collection to 1 and its allocation configuration to fixed size
        reader_limits.matched_publisher_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //DDS_CHANGE_WRITER_RESOURCE_LIMITS_QOS
        WriterResourceLimitsQos writer_limits;
        //Set the maximum size for reader matched resource limits collection to 3 and its allocation configuration to fixed size
        writer_limits.matched_subscriber_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // Set the maximum number of writer side content filters to 1 and its allocation configuration to fixed size
        writer_limits.reader_filters_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //DDS_CHANGE_RTPS_ENDPOINT_QOS
        RTPSEndpointQos endpoint;
        //Add new unicast locator with port 7800
        eprosima::fastrtps::rtps::Locator_t new_unicast_locator;
        new_unicast_locator.port = 7800;
        endpoint.unicast_locator_list.push_back(new_unicast_locator);
        //Add new multicast locator with IP 239.255.0.4 and port 7900
        eprosima::fastrtps::rtps::Locator_t new_multicast_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
        new_multicast_locator.port = 7900;
        endpoint.multicast_locator_list.push_back(new_multicast_locator);
        // Add an external locator with IP 100.100.100.10, port 12345, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask external_locator;
        external_locator.kind = LOCATOR_KIND_UDPv4;
        external_locator.port = 12345;
        external_locator.mask(24);
        endpoint.external_unicast_locators[1][0].push_back(external_locator);
        // Drop non matching locators
        endpoint.ignore_non_matching_locators = true;
        //Set 3 as user defined id
        endpoint.user_defined_id = 3;
        //Set 4 as entity id
        endpoint.entity_id = 4;
        //The RTPSEndpointQos is default constructed with history_memory_policy = PREALLOCATED
        //Change the history_memory_policy to DYNAMIC_RESERVE
        endpoint.history_memory_policy = eprosima::fastrtps::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
        //!--
    }

    {
        //DDS_CHANGE_RTPS_RELIABLE_WRITER_QOS
        RTPSReliableWriterQos reliable_writer_qos;
        //The RTPSReliableWriterQos is default constructed with initialHeartbeatDelay = 12 ms
        //Change the initialHeartbeatDelay to 20 nanoseconds
        reliable_writer_qos.times.initialHeartbeatDelay = {0, 20};
        //The RTPSReliableWriterQos is default constructed with heartbeatPeriod = 3 s
        //Change the heartbeatPeriod to 5 seconds
        reliable_writer_qos.times.heartbeatPeriod = {5, 0};
        //The RTPSReliableWriterQos is default constructed with nackResponseDelay = 5 ms
        //Change the nackResponseDelay to 10 nanoseconds
        reliable_writer_qos.times.nackResponseDelay = {0, 10};
        //The RTPSReliableWriterQos is default constructed with nackSupressionDuration = 0 s
        //Change the nackSupressionDuration to 20 nanoseconds
        reliable_writer_qos.times.nackSupressionDuration = {0, 20};
        //You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
        reliable_writer_qos.disable_positive_acks.enabled = true;
        //The RTPSReliableWriterQos is default constructed with disable_heartbeat_piggyback = false
        //Disable the heartbeat piggyback mechanism.
        reliable_writer_qos.disable_heartbeat_piggyback = true;
        //!--
    }

    {
        //DDS_CHANGE_RTPS_RELIABLE_READER_QOS
        RTPSReliableReaderQos reliable_reader_qos;
        //The RTPSReliableReaderQos is default constructed with initialAcknackDelay = 70 ms
        //Change the initialAcknackDelay to 70 nanoseconds
        reliable_reader_qos.times.initialAcknackDelay = {0, 70};
        //The RTPSReliableWriterQos is default constructed with heartbeatResponseDelay = 5 ms
        //Change the heartbeatResponseDelay to 5 nanoseconds
        reliable_reader_qos.times.heartbeatResponseDelay = {0, 5};
        //You can also change the DisablePositiveACKsQosPolicy. For further details see DisablePositiveACKsQosPolicy section.
        reliable_reader_qos.disable_positive_ACKs.enabled = true;
        //!--
    }

    {
        //DDS_CHANGE_TRANSPORT_CONFIG_QOS
        TransportConfigQos transport;
        //Add new transport to the list of user transports
        std::shared_ptr<eprosima::fastdds::rtps::UDPv4TransportDescriptor> descriptor =
                std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        descriptor->sendBufferSize = 9126;
        descriptor->receiveBufferSize = 9126;
        transport.user_transports.push_back(descriptor);
        //Set use_builtin_transports to false
        transport.use_builtin_transports = false;
        //!--
    }

    {
        //DDS_CHANGE_WIRE_PROTOCOL_CONFIG_QOS
        WireProtocolConfigQos wire_protocol;
        //Set the guid prefix
        std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> wire_protocol.prefix;
        //Configure Builtin Attributes
        wire_protocol.builtin.discovery_config.discoveryProtocol =
                eprosima::fastrtps::rtps::DiscoveryProtocol_t::SERVER;
        //Add locator to unicast list
        eprosima::fastrtps::rtps::Locator_t server_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
        server_locator.port = 56542;
        wire_protocol.builtin.metatrafficUnicastLocatorList.push_back(server_locator);
        // Add a metatraffic external locator with IP 100.100.100.10, port 34567, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask meta_external_locator;
        meta_external_locator.kind = LOCATOR_KIND_UDPv4;
        meta_external_locator.port = 34567;
        meta_external_locator.mask(24);
        wire_protocol.builtin.metatraffic_external_unicast_locators[1][0].push_back(meta_external_locator);
        //Add locator to default unicast locator list
        eprosima::fastrtps::rtps::Locator_t unicast_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(unicast_locator, 192, 168, 1, 41);
        unicast_locator.port = 7400;
        wire_protocol.default_unicast_locator_list.push_back(unicast_locator);
        //Add locator to default multicast locator list
        eprosima::fastrtps::rtps::Locator_t multicast_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(multicast_locator, 192, 168, 1, 41);
        multicast_locator.port = 7400;
        wire_protocol.default_multicast_locator_list.push_back(multicast_locator);
        // Add a default external locator with IP 100.100.100.10, port 23456, mask 24, externality 1, and cost 0
        eprosima::fastdds::rtps::LocatorWithMask external_locator;
        external_locator.kind = LOCATOR_KIND_UDPv4;
        external_locator.port = 23456;
        external_locator.mask(24);
        wire_protocol.default_external_unicast_locators[1][0].push_back(external_locator);
        // Drop non matching locators
        wire_protocol.ignore_non_matching_locators = true;
        //!--
    }

    {
        //DDS_CHANGE_DATA_REPRESENTATION_QOS
        DataRepresentationQosPolicy data_representation;
        //Add XCDR v1 data representation to the list of valid representations
        data_representation.m_value.push_back(DataRepresentationId_t::XCDR_DATA_REPRESENTATION);
        //Add XML data representation to the list of valid representations
        data_representation.m_value.push_back(DataRepresentationId_t::XML_DATA_REPRESENTATION);
        //!--
    }

    {
        //DDS_CHANGE_TYPE_CONSISTENCY_ENFORCEMENT_QOS
        TypeConsistencyEnforcementQosPolicy type_enforcement;
        //The TypeConsistencyEnforcementQosPolicy is default constructed with kind = ALLOW_TYPE_COERCION
        //Change the kind to DISALLOW_TYPE_COERCION
        type_enforcement.m_kind = TypeConsistencyKind::DISALLOW_TYPE_COERCION;
        //Configures the system to ignore the sequence sizes in assignations
        type_enforcement.m_ignore_sequence_bounds = true;
        //Configures the system to ignore the string sizes in assignations
        type_enforcement.m_ignore_string_bounds = true;
        //Configures the system to ignore the member names. Members with same ID could have different names
        type_enforcement.m_ignore_member_names = true;
        //Configures the system to allow type widening
        type_enforcement.m_prevent_type_widening = false;
        //Configures the system to not use the complete Type Information in entities match process
        type_enforcement.m_force_type_validation = false;
        //!--
    }

    {
        //DDS_CHANGE_TYPE_CONSISTENCY_QOS
        TypeConsistencyQos consistency_qos;
        //You can change the DataRepresentationQosPolicy. For further details see DataRepresentationQosPolicySection section.
        consistency_qos.representation.m_value.push_back(DataRepresentationId_t::XCDR2_DATA_REPRESENTATION);
        //You can change the TypeConsistencyEnforcementQosPolicy. For further details see TypeConsistencyEnforcementQosPolicy section.
        consistency_qos.type_consistency.m_kind = TypeConsistencyKind::ALLOW_TYPE_COERCION;
        //!--
    }

    // Taken out of the examples to avoid bloating them
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    Subscriber* subscriber =
            participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    Topic* topic =
            participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

    {
        //DDS_QOS_POLICY_COUNT_SEQ
        DataReader* data_reader =
                subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);

        // Get how many times ReliabilityQosPolicy was not compatible with a remote writer
        RequestedIncompatibleQosStatus status;
        data_reader->get_requested_incompatible_qos_status(status);
        uint32_t incompatible_reliability_count = status.policies[RELIABILITY_QOS_POLICY_ID].count;

        //!--
    }

    {
        //CONF_GUIDPREFIX_OPTION_1
        eprosima::fastrtps::rtps::GuidPrefix_t guid_prefix;
        guid_prefix.value[0] = eprosima::fastrtps::rtps::octet(0x77);
        guid_prefix.value[1] = eprosima::fastrtps::rtps::octet(0x73);
        guid_prefix.value[2] = eprosima::fastrtps::rtps::octet(0x71);
        guid_prefix.value[3] = eprosima::fastrtps::rtps::octet(0x85);
        guid_prefix.value[4] = eprosima::fastrtps::rtps::octet(0x69);
        guid_prefix.value[5] = eprosima::fastrtps::rtps::octet(0x76);
        guid_prefix.value[6] = eprosima::fastrtps::rtps::octet(0x95);
        guid_prefix.value[7] = eprosima::fastrtps::rtps::octet(0x66);
        guid_prefix.value[8] = eprosima::fastrtps::rtps::octet(0x65);
        guid_prefix.value[9] = eprosima::fastrtps::rtps::octet(0x82);
        guid_prefix.value[10] = eprosima::fastrtps::rtps::octet(0x82);
        guid_prefix.value[11] = eprosima::fastrtps::rtps::octet(0x79);

        DomainParticipantQos participant_qos;
        participant_qos.wire_protocol().prefix = guid_prefix;
        //!--
    }

    {
        //CONF_GUIDPREFIX_OPTION_2
        DomainParticipantQos participant_qos;
        std::istringstream("77.73.71.85.69.76.95.66.65.82.82.79") >> participant_qos.wire_protocol().prefix;
        //!--
    }
}

void dds_dynamic_types_examples ()
{
    {
        //DDS_TYPELOOKUP_SERVICE_ENABLING
        DomainParticipantQos qos;
        qos.wire_protocol().builtin.typelookup_config.use_client = true;
        qos.wire_protocol().builtin.typelookup_config.use_server = true;
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
    //!--

}

void xml_profiles_examples()
{
    {
        //XML-LOAD-APPLY-PROFILES
        if (ReturnCode_t::RETCODE_OK ==
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
                    <profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\" >\
                        <data_writer profile_name=\"test_publisher_profile\" is_default_profile=\"true\">\
                            <qos>\
                                <durability>\
                                    <kind>TRANSIENT_LOCAL</kind>\
                                </durability>\
                            </qos>\
                        </data_writer>\
                    </profiles>\
                </dds>\
                ";
        if (ReturnCode_t::RETCODE_OK ==
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
        if (ReturnCode_t::RETCODE_OK ==
                DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
        {
            // Retrieve the an instance of MyStruct type
            eprosima::fastrtps::types::DynamicType_ptr my_struct_type =
                    eprosima::fastrtps::xmlparser::XMLProfileManager::getDynamicTypeByName("MyStruct")->build();
            // Register MyStruct type
            TypeSupport my_struct_type_support(new eprosima::fastrtps::types::DynamicPubSubType(my_struct_type));
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
        if (ReturnCode_t::RETCODE_OK ==
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
}

void dds_transport_examples ()
{
    using UDPv4TransportDescriptor = eprosima::fastdds::rtps::UDPv4TransportDescriptor;
    using UDPv6TransportDescriptor = eprosima::fastdds::rtps::UDPv6TransportDescriptor;
    using TCPv4TransportDescriptor = eprosima::fastdds::rtps::TCPv4TransportDescriptor;
    using TCPv6TransportDescriptor = eprosima::fastdds::rtps::TCPv6TransportDescriptor;
    using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;
    using Locator_t = eprosima::fastrtps::rtps::Locator_t;
    using IPLocator = eprosima::fastrtps::rtps::IPLocator;

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
        eprosima::fastrtps::rtps::Locator_t locator;
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
        eprosima::fastrtps::rtps::Locator_t locator;
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
        eprosima::fastrtps::rtps::Locator_t locator;
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
        eprosima::fastrtps::rtps::Locator_t locator;
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
        tcp_transport->set_WAN_address("80.80.99.45");

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;

        eprosima::fastrtps::rtps::Locator_t locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, "80.80.99.45");
        eprosima::fastrtps::rtps::IPLocator::setWan(locator, "80.80.99.45");
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(locator, 5100);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(locator, 5100);

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

        // Set initial peers.
        eprosima::fastrtps::rtps::Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(initial_peer_locator, "80.80.99.45");
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

        qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //CONF-SHM-TRANSPORT-SETTING
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        std::shared_ptr<SharedMemTransportDescriptor> shm_transport = std::make_shared<SharedMemTransportDescriptor>();

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(shm_transport);
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
        tls_transport->set_WAN_address("80.80.99.45");

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
        IPLocator::setIPv4(initial_peer_locator, "80.80.99.45");
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

        // Add loopback to the whitelist
        tcp_transport->interfaceWhiteList.emplace_back("127.0.0.1");

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(tcp_transport);

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
    using Locator_t = eprosima::fastrtps::rtps::Locator_t;
    using RemoteServerAttributes = eprosima::fastrtps::rtps::RemoteServerAttributes;
    using IPLocator = eprosima::fastrtps::rtps::IPLocator;
    using DiscoveryProtocol_t = eprosima::fastrtps::rtps::DiscoveryProtocol_t;

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
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;

        // Define the listening locator to be on interface 192.168.10.57 and port 56542
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.57");
        server_locator.port = 56542;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        // Set the GUID prefix to identify this server
        std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> qos.wire_protocol().prefix;
        //!--
    }

    {
        //CONF_DS_MAIN_SCENARIO_CLIENT
        DomainParticipantQos qos;

        // Configure the current participant as CLIENT
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::CLIENT;

        // Define a locator for the SERVER Participant on address 192.168.10.57 and port 56542
        Locator_t remote_server_locator;
        IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
        remote_server_locator.port = 56542;

        RemoteServerAttributes remote_server_attr;
        remote_server_attr.metatrafficUnicastLocatorList.push_back(remote_server_locator);

        // Set the GUID prefix to identify the remote server
        remote_server_attr.ReadguidPrefix("72.61.73.70.66.61.72.6d.74.65.73.74");

        // Connect to the SERVER at the previous locator
        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
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
        server_1_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> server_1_qos.wire_protocol().prefix;
        server_1_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator_1);

        // Configure second server's locator on interface 192.168.10.60 and port 56543
        Locator_t server_locator_2;
        IPLocator::setIPv4(server_locator_2, "192.168.10.60");
        server_locator_2.port = 56543;

        // Configure participant_2 as SERVER listening on the previous locator
        DomainParticipantQos server_2_qos;
        server_2_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
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

        RemoteServerAttributes remote_server_attr_1;
        remote_server_attr_1.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        remote_server_attr_1.metatrafficUnicastLocatorList.push_back(remote_server_locator_1);

        // Define a locator for the second SERVER Participant
        Locator_t remote_server_locator_2;
        IPLocator::setIPv4(remote_server_locator_2, "192.168.10.60");
        remote_server_locator_2.port = 56543;

        RemoteServerAttributes remote_server_attr_2;
        remote_server_attr_2.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr_2.metatrafficUnicastLocatorList.push_back(remote_server_locator_2);

        // Configure the current participant as CLIENT connecting to the SERVERS at the previous locators
        DomainParticipantQos client_qos;
        client_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::CLIENT;
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_1);
        client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_2);
        //!--
    }

    {
        //CONF_DS_PARTITION_2
        DomainParticipantQos qos;

        // Configure current Participant as SERVER on address 192.168.10.60
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.60");
        server_locator.port = 56543;

        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> qos.wire_protocol().prefix;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        // Add the connection attributes to the remote server.
        Locator_t remote_server_locator;
        IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
        remote_server_locator.port = 56542;

        RemoteServerAttributes remote_server_attr;
        remote_server_attr.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr.metatrafficUnicastLocatorList.push_back(remote_server_locator);

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
        //!--
    }

    {
        //CONF_DS_PARTITION_3
        DomainParticipantQos qos;

        // Configure current Participant as SERVER on address 192.168.10.60
        Locator_t server_locator;
        IPLocator::setIPv4(server_locator, "192.168.10.54");
        server_locator.port = 56541;

        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.33") >> qos.wire_protocol().prefix;
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        // Add the connection attributes to the remote server A.
        Locator_t remote_server_locator_A;
        IPLocator::setIPv4(remote_server_locator_A, "192.168.10.60");
        remote_server_locator_A.port = 56543;

        RemoteServerAttributes remote_server_attr_A;
        remote_server_attr_A.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        remote_server_attr_A.metatrafficUnicastLocatorList.push_back(remote_server_locator_A);

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_A);

        // Add the connection attributes to the remote server B.
        Locator_t remote_server_locator_B;
        IPLocator::setIPv4(remote_server_locator_B, "192.168.10.57");
        remote_server_locator_B.port = 56542;

        RemoteServerAttributes remote_server_attr_B;
        remote_server_attr_B.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr_B.metatrafficUnicastLocatorList.push_back(remote_server_locator_B);

        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_B);
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
        qos.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
        qos.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500000000; //500 ms
        //!--
    }

    {
        //DDS_MULTICAST_DELIVERY
        DataReaderQos qos;

        // Add new multicast locator with IP 239.255.0.4 and port 7900
        eprosima::fastrtps::rtps::Locator_t new_multicast_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
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
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);

        // Fix the size of discovered DataWriters to 1 per DomainParticipant
        // Fix the size of discovered DataReaders to 3 per DomainParticipant
        // This will effectively preallocate the memory during initialization
        qos.allocation().writers =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        qos.allocation().readers =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
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
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-WRITER
        DataWriterQos qos;

        // Fix the size of matched DataReaders to 3
        // This will effectively preallocate the memory during initialization
        qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // Fix the size of writer side content filters to 1
        // This will effectively preallocate the memory during initialization
        qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-READER
        DataReaderQos qos;

        // Fix the size of matched DataWriters to 1
        // This will effectively preallocate the memory during initialization
        qos.reader_resource_limits().matched_publisher_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-ALLOCATION-QOS-EXAMPLE
        // DomainParticipant configuration
        //////////////////////////////////
        DomainParticipantQos participant_qos;

        // We know we have 3 participants on the domain
        participant_qos.allocation().participants =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        // We know we have at most 2 readers on each participant
        participant_qos.allocation().readers =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        // We know we have at most 1 writer on each participant
        participant_qos.allocation().writers =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

        // We know the maximum size of partition data
        participant_qos.allocation().data_limits.max_partitions = 256u;
        // We know the maximum size of user data
        participant_qos.allocation().data_limits.max_user_data = 256u;
        // We know the maximum size of properties data
        participant_qos.allocation().data_limits.max_properties = 512u;

        // Content filtering is not being used
        participant_qos.allocation().content_filter.expression_initial_size = 0u;
        participant_qos.allocation().content_filter.expression_parameters =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

        // DataWriter configuration for Topic 1
        ///////////////////////////////////////
        DataWriterQos writer1_qos;

        // We know we will only have three matching subscribers, and no content filters
        writer1_qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
        writer1_qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

        // DataWriter configuration for Topic 2
        ///////////////////////////////////////
        DataWriterQos writer2_qos;

        // We know we will only have two matching subscribers
        writer2_qos.writer_resource_limits().matched_subscriber_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
        writer2_qos.writer_resource_limits().reader_filters_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(0u);


        // DataReader configuration for both Topics
        ///////////////////////////////////////////
        DataReaderQos reader_qos;

        // We know we will only have one matching publisher
        reader_qos.reader_resource_limits().matched_publisher_allocation =
                eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
        //!--
    }

    {
        //CONF-MEMORY-QOS-PUBSUB
        ResourceLimitsQosPolicy resource_limits;

        // The ResourceLimitsQosPolicy is default constructed with max_samples = 5000
        // Change max_samples to the minimum
        resource_limits.max_samples = 1;

        // The ResourceLimitsQosPolicy is default constructed with max_instances = 10
        // Change max_instances to the minimum
        resource_limits.max_instances = 1;

        // The ResourceLimitsQosPolicy is default constructed with max_samples_per_instance = 400
        // Change max_samples_per_instance to the minimum
        resource_limits.max_samples_per_instance = 1;

        // The ResourceLimitsQosPolicy is default constructed with allocated_samples = 100
        // No allocated samples
        resource_limits.allocated_samples = 0;
        //!--
    }


    {
        //CONF-MEMORY-QOS-ENDPOINTS
        RTPSEndpointQos endpoint;
        endpoint.history_memory_policy = eprosima::fastrtps::rtps::DYNAMIC_REUSABLE_MEMORY_MODE;
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
    eprosima::fastrtps::types::DynamicTypeBuilder_ptr struct_type_builder(
        eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance()->create_struct_builder());
    struct_type_builder->set_name(topic_type_name);

    // The type consists of two members, and index and a message. Add members to the struct.
    struct_type_builder->add_member(0, "index",
            eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance()->create_uint32_type());
    struct_type_builder->add_member(1, "message",
            eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance()->create_string_type());

    // Build the type
    eprosima::fastrtps::types::DynamicType_ptr dyn_type_ptr = struct_type_builder->build();

    // Create type support and register the type
    TypeSupport type_support(new eprosima::fastrtps::types::DynamicPubSubType(dyn_type_ptr));
    type_support.register_type(participant);

    // Create data sample a populate data. This is to be used when calling `writer->write()`
    eprosima::fastrtps::types::DynamicData* dyn_helloworld;
    dyn_helloworld = eprosima::fastrtps::types::DynamicDataFactory::get_instance()->create_data(dyn_type_ptr);
    dyn_helloworld->set_uint32_value(0, 0);
    dyn_helloworld->set_string_value("HelloWorld", 1);
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
        setName("LoanableHelloWorld");
    }

    bool serialize(
            void* /*data*/,
            eprosima::fastrtps::rtps::SerializedPayload_t* /*payload*/) override
    {
        return true;
    }

    bool deserialize(
            eprosima::fastrtps::rtps::SerializedPayload_t* /*payload*/,
            void* /*data*/) override
    {
        return true;
    }

    std::function<uint32_t()> getSerializedSizeProvider(
            void* /*data*/) override
    {
        return std::function<uint32_t()>();
    }

    void* createData() override
    {
        return nullptr;
    }

    void deleteData(
            void* /*data*/) override
    {
    }

    bool getKey(
            void* /*data*/,
            eprosima::fastrtps::rtps::InstanceHandle_t* /*ihandle*/,
            bool /*force_md5*/) override
    {
        return true;
    }

    MD5 m_md5;
    unsigned char* m_keyBuffer;
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
        if (ReturnCode_t::RETCODE_OK == writer->loan_sample(sample))
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
                while (ReturnCode_t::RETCODE_OK == reader->take(data, infos))
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
                                      << (reader->is_sample_valid(&sample, &infos[i]) ? " is valid" : " was replaced" ) << std::endl
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

        eprosima::fastrtps::rtps::SampleIdentity my_request_sample_identity;

    }
    listener;

    eprosima::fastdds::dds::DomainParticipantQos participant_qos;
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_default_participant_qos(participant_qos);

    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);

    TypeSupport request_type;
    TypeSupport reply_type;

    Publisher* publisher = participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
    Subscriber* subscriber = participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);

    //REQUEST_REPLY_EXAMPLE_CLIENT_CREATE_ENTITIES
    participant->register_type(request_type);
    participant->register_type(reply_type);

    Topic* request_topic = participant->create_topic("CalculatorRequest",
                    request_type.get_type_name(), TOPIC_QOS_DEFAULT);

    Topic* reply_topic = participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

    DataWriter* request_writer = publisher->create_datawriter(request_topic, DATAWRITER_QOS_DEFAULT);

    DataReader* reply_reader = subscriber->create_datareader(reply_topic, DATAREADER_QOS_DEFAULT, &listener);
    //!

    //REQUEST_REPLY_EXAMPLE_CLIENT_RETRIEVE_ID
    eprosima::fastrtps::rtps::SampleIdentity my_request_sample_identity;
    RequestType request;

    // Fill the request

    // Publish request
    eprosima::fastrtps::rtps::WriteParams write_params;
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
                eprosima::fastrtps::rtps::SampleIdentity client_request_identity = sample_info.sample_identity;
            }
        }

        //!
    }
    listener;

    eprosima::fastdds::dds::DomainParticipantQos participant_qos;
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_default_participant_qos(participant_qos);

    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);

    TypeSupport request_type;
    TypeSupport reply_type;

    Publisher* publisher = participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);

    Subscriber* subscriber = participant->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);

    //REQUEST_REPLY_EXAMPLE_SERVER_CREATE_ENTITIES
    participant->register_type(request_type);
    participant->register_type(reply_type);

    Topic* request_topic = participant->create_topic("CalculatorRequest",
                    request_type.get_type_name(), TOPIC_QOS_DEFAULT);

    Topic* reply_topic = participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

    DataWriter* reply_writer = publisher->create_datawriter(reply_topic, DATAWRITER_QOS_DEFAULT);

    DataReader* request_reader = subscriber->create_datareader(request_topic, DATAREADER_QOS_DEFAULT, &listener);
    //!

    eprosima::fastrtps::rtps::SampleIdentity client_request_identity;
    //REQUEST_REPLY_EXAMPLE_SERVER_SEND_REPLY
    ReplyType reply;

    // Fill reply

    // Send reply associating it with the client request.
    eprosima::fastrtps::rtps::WriteParams write_params;
    write_params.related_sample_identity() = client_request_identity;
    reply_writer->write(reinterpret_cast<void*>(&reply), write_params);
    //!
}

void dds_waitset_example()
{
    auto create_dds_application = [](std::vector<DataReader*>&, std::vector<DataWriter*>&) -> ReturnCode_t
            {
                return ReturnCode_t::RETCODE_OK;
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
                ret_code = wait_set_.wait(triggered_conditions, eprosima::fastrtps::c_TimeInfinite);
                if (ReturnCode_t::RETCODE_OK != ret_code)
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
                            std::cout << "Liveliness changed reported for entity " << entity->get_instance_handle() <<
                                std::endl;
                        }

                        if (changed_statuses.is_active(StatusMask::data_available()))
                        {
                            std::cout << "Data avilable on reader " << entity->get_instance_handle() << std::endl;

                            FooSeq data_seq;
                            SampleInfoSeq info_seq;
                            DataReader* reader = static_cast<DataReader*>(entity);

                            // Process all the samples until no one is returned
                            while (ReturnCode_t::RETCODE_OK == reader->take(data_seq, info_seq,
                                    LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                                    ANY_VIEW_STATE, ANY_INSTANCE_STATE))
                            {
                                // Both info_seq.length() and data_seq.length() will have the number of samples returned
                                for (FooSeq::size_type n = 0; n < info_seq.length(); ++n)
                                {
                                    // Only samples with valid data should be accessed
                                    if (info_seq[n].valid_data && reader->is_sample_valid(&data_seq[n], &info_seq[n]))
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
    if (ReturnCode_t::RETCODE_OK != ret_code)
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
        eprosima::fastrtps::rtps::Locator_t pdp_locator;
        pdp_locator.kind = LOCATOR_KIND_UDPv4;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(pdp_locator, "239.255.0.1");
        pqos.wire_protocol().builtin.metatrafficMulticastLocatorList.push_back(pdp_locator);

        // Define locator for EDP and user data
        eprosima::fastrtps::rtps::Locator_t tcp_locator;
        tcp_locator.kind = LOCATOR_KIND_TCPv4;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(tcp_locator, "0.0.0.0");
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(tcp_locator, tcp_listening_port);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(tcp_locator, tcp_listening_port);
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
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol_t::SERVER;

        // Add custom user transport with TCP port 12345
        auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        data_transport->add_listener_port(12345);
        qos.transport().user_transports.push_back(data_transport);

        // Define the listening locator to be on interface 192.168.10.57 and port 12345
        constexpr uint16_t tcp_listening_port = 12345;
        eprosima::fastrtps::rtps::Locator_t listening_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(listening_locator, "192.168.10.57");
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(listening_locator, tcp_listening_port);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(listening_locator, tcp_listening_port);
        qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(listening_locator);

        // Set the GUID prefix to identify this server
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> qos.wire_protocol().prefix;
        //!--
    }

    {
        //TCP-AND-DISCOVERY-SERVER-CLIENT
        eprosima::fastdds::dds::DomainParticipantQos qos = PARTICIPANT_QOS_DEFAULT;

        // Configure the current participant as SERVER
        qos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol_t::CLIENT;

        // Add custom user transport with TCP port 0 (automatic port assignation)
        auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
        data_transport->add_listener_port(0);
        qos.transport().user_transports.push_back(data_transport);

        // Define the server locator to be on interface 192.168.10.57 and port 12345
        constexpr uint16_t server_port = 12345;
        eprosima::fastrtps::rtps::Locator_t server_locator;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
        eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(server_locator, server_port);
        eprosima::fastrtps::rtps::IPLocator::setLogicalPort(server_locator, server_port);

        // Define the server attributes
        eprosima::fastrtps::rtps::RemoteServerAttributes remote_server_att;
        remote_server_att.metatrafficUnicastLocatorList.push_back(server_locator);

        // Set the GUID prefix to identify this server
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> remote_server_att.guidPrefix;

        // Add the server
        qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_att);
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
            if (ReturnCode_t::RETCODE_OK != factory->check_xml_static_discovery(file))
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
            if (ReturnCode_t::RETCODE_OK != factory->check_xml_static_discovery(fileData))
            {
                printf("Error parsing xml file %s\n", argv[1]);
                exit_code = -1;
            }
        }
        else
        {
            eprosima::fastrtps::xmlparser::XMLProfileManager parser;
            if (parser.loadXMLFile(argv[1]) != eprosima::fastrtps::xmlparser::XMLP_ret::XML_OK)
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
