
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

using namespace eprosima::fastdds::dds;

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

    virtual void on_participant_discovery(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info)
    {
        if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
        {
            std::cout << "New participant discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
                info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT)
        {
            std::cout << "New participant lost" << std::endl;
        }
    }

#if HAVE_SECURITY
    virtual void onParticipantAuthentication(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantAuthenticationInfo&& info)
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

#endif

    virtual void on_subscriber_discovery(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info)
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

    virtual void on_publisher_discovery(
            DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info)
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

    virtual void on_type_discovery(
            DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::string_255& topic,
            const eprosima::fastrtps::types::TypeIdentifier* identifier,
            const eprosima::fastrtps::types::TypeObject* object,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type)
    {
        (void)participant, (void)request_sample_id, (void)topic, (void)identifier, (void)object, (void)dyn_type;
        std::cout << "New data type discovered" << std::endl;

    }

    virtual void on_type_dependencies_reply(
            DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq& dependencies)
    {
        (void)participant, (void)request_sample_id, (void)dependencies;
        std::cout << "Answer to a request for type dependencies was received" << std::endl;
    }

    virtual void on_type_information_received(
            DomainParticipant* participant,
            const eprosima::fastrtps::string_255 topic_name,
            const eprosima::fastrtps::string_255 type_name,
            const eprosima::fastrtps::types::TypeInformation& type_information)
    {
        (void)participant, (void)topic_name, (void)type_name, (void)type_information;
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
        if (nullptr != participant_with_profile)
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
        if (nullptr != participant_with_default_attributes)
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
        if (nullptr != participant_with_custom_qos)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with default QoS and a custom Listener.
        // CustomDomainParticipantListener inherits from DomainParticipantListener.
        // The value PARTICIPANT_QOS_DEFAULT is used to denote the default QoS.
        CustomDomainParticipantListener custom_listener;
        DomainParticipant* participant_with_default_qos_and_custom_listener =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT, &custom_listener);
        if (nullptr != participant_with_default_qos_and_custom_listener)
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
        if (nullptr != participant_with_profile)
        {
            // Error
            return;
        }

        // Create a DomainParticipant using a profile and a custom Listener.
        // CustomDomainParticipantListener inherits from DomainParticipantListener.
        CustomDomainParticipantListener listener;
        DomainParticipant* participant_with_profile_and_listener =
                DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile", &listener);
        if (nullptr != participant_with_profile_and_listener)
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
        if (nullptr != participant)
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
        if (nullptr != participant)
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
        if(participant->set_qos(DomainParticipantFactory::get_instance()->get_default_participant_qos())
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Use the DomainParticipant to communicate
        // (...)

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
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new default DomainParticipantQos.
        DomainParticipant* participant_with_qos_type1 =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr != participant_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DomainParticipantQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        DomainParticipant* participant_with_qos_type2 =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr != participant_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default DomainParticipantQos to the original default constructed values
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(PARTICIPANT_QOS_DEFAULT)
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(DomainParticipantQos())
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
        if(DomainParticipantFactory::get_instance()->set_qos(qos) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new DomainParticipantFactoryQos.
        // The returned DomainParticipant is already enabled
        DomainParticipant* enabled_participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr != enabled_participant)
        {
            // Error
            return;
        }

        // Setting autoenable_created_entities to false makes the created DomainParticipants
        // to be disabled upon creation
        qos.entity_factory().autoenable_created_entities = false;
        if(DomainParticipantFactory::get_instance()->set_qos(qos) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DomainParticipant with the new DomainParticipantFactoryQos.
        // The returned DomainParticipant is disabled and will need to be enabled explicitly
        DomainParticipant* disabled_participant =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (nullptr != disabled_participant)
        {
            // Error
            return;
        }
        //!--
    }
}



