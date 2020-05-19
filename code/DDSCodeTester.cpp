
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
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/SubscriberListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/topic/TopicListener.hpp>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/types/DynamicTypePtr.h>


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
        CustomDomainParticipantListener custom_listener;
        DomainParticipant* participant_with_profile_and_custom_listener =
                DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile", &custom_listener);
        if (nullptr != participant_with_profile_and_custom_listener)
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
    {
        // DDS_SECURITY_AUTH_PLUGIN
        eprosima::fastdds::dds::DomainParticipantQos pqos;

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
        eprosima::fastdds::dds::DomainParticipantQos pqos;

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

    virtual void on_inconsistent_topic(
            Topic* topic,
            InconsistentTopicStatus status)
    {
        (void)topic, (void)status;
        std::cout << "Inconsistent topic received discovered" << std::endl;
    }
};
//!--

class CustomDataType : public TopicDataType
{
public:

    CustomDataType()
        : TopicDataType()
    {
        setName("footype");
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Topic with default TopicQos and no Listener
        // The symbol TOPIC_QOS_DEFAULT is used to denote the default QoS.
        Topic* topic_with_default_qos =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr != topic_with_default_qos)
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
        if (nullptr != topic_with_custom_qos)
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
        if (nullptr != topic_with_default_qos_and_custom_listener)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Topic using a profile and no Listener
        Topic* topic_with_profile =
                participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile");
        if (nullptr != topic_with_profile)
        {
            // Error
            return;
        }

        // Create a Topic using a profile and a custom Listener.
        // CustomTopicListener inherits from TopicListener.
        CustomTopicListener custom_listener;
        Topic* topic_with_profile_and_custom_listener =
                participant->create_topic_with_profile("TopicName", "DataTypeName", "topic_profile", &custom_listener);
        if (nullptr != topic_with_profile_and_custom_listener)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Topic with default TopicQos
        Topic* topic =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr != topic)
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
        if (nullptr != participant)
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
        if (nullptr != topic)
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
        if(topic->set_qos(participant->get_default_topic_qos())
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Topic
        Topic* topic =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr != topic)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        TopicQos qos_type1 = participant->get_default_topic_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if(participant->set_default_topic_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        Topic* topic_with_qos_type1 =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr != topic_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        TopicQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default TopicQos
        if(participant->set_default_topic_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Topic with the new default TopicQos.
        Topic* topic_with_qos_type2 =
                participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);
        if (nullptr != topic_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default TopicQos to the original default constructed values
        if(participant->set_default_topic_qos(TOPIC_QOS_DEFAULT)
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if(participant->set_default_topic_qos(TopicQos())
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
        if (nullptr != participant)
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
        if (nullptr != topic)
        {
            // Error
            return;
        }

        // Create an alias for the same data type using a different name.
        custom_type_support.register_type(participant, "data_type_name");

        // We can now use the aliased name to If no name is given, it uses the name returned by the type itself
        Topic* another_topic =
                participant->create_topic("other_topic_name", "data_type_name", TOPIC_QOS_DEFAULT);
        if (nullptr != another_topic)
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
        if (nullptr != participant)
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
        if (nullptr != topic)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Publisher with default PublisherQos and no Listener
        // The value PUBLISHER_QOS_DEFAULT is used to denote the default QoS.
        Publisher* publisher_with_default_qos =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr != publisher_with_default_qos)
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
        if (nullptr != publisher_with_custom_qos)
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
        if (nullptr != publisher_with_default_qos_and_custom_listener)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Publisher using a profile and no Listener
        Publisher* publisher_with_profile =
                participant->create_publisher_with_profile("publisher_profile");
        if (nullptr != publisher_with_profile)
        {
            // Error
            return;
        }

        // Create a Publisher using a profile and a custom Listener.
        // CustomPublisherListener inherits from PublisherListener.
        CustomPublisherListener custom_listener;
        Publisher* publisher_with_profile_and_custom_listener =
                participant->create_publisher_with_profile("publisher_profile", &custom_listener);
        if (nullptr != publisher_with_profile_and_custom_listener)
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Publisher with default PublisherQos
        Publisher* publisher =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr != publisher)
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
        if (nullptr != participant)
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
        if (nullptr != publisher)
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
        if(publisher->set_qos(participant->get_default_publisher_qos())
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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Create a Publisher
        Publisher* publisher =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr != publisher)
        {
            // Error
            return;
        }

        // Use the Publisher to communicate
        // (...)

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
        if (nullptr != participant)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        PublisherQos qos_type1 = participant->get_default_publisher_qos();

        // Modify QoS attributes
        // (...)

        // Set as the new default PublisherQos
        if(participant->set_default_publisher_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Publisher with the new default PublisherQos.
        Publisher* publisher_with_qos_type1 =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr != publisher_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        PublisherQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default PublisherQos
        if(participant->set_default_publisher_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a Publisher with the new default PublisherQos.
        Publisher* publisher_with_qos_type2 =
                participant->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (nullptr != publisher_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default PublisherQos to the original default constructed values
        if(participant->set_default_publisher_qos(PUBLISHER_QOS_DEFAULT)
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if(participant->set_default_publisher_qos(PublisherQos())
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

    virtual void on_publication_matched(
            DataWriter* writer,
            const PublicationMatchedStatus& info)
    {
        (void)writer
        ;
        if (info.current_count_change == 1)
        {
            std::cout << "Matched a remote Subscriber for one of our Topics" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Unmatched a remote Subscriber" << std::endl;
        }
    }

    virtual void on_offered_deadline_missed(
             DataWriter* writer,
             const OfferedDeadlineMissedStatus& status)
    {
         (void)writer, (void)status;
         std::cout << "Some data could not be delivered on time" << std::endl;
    }

    virtual void on_offered_incompatible_qos(
         DataWriter* writer,
         const OfferedIncompatibleQosStatus& status)
    {
        (void)writer, (void)status;
        std::cout << "Found a remote Topic with incompatible QoS" << std::endl;
    }

    virtual void on_liveliness_lost(
         DataWriter* writer,
         const LivelinessLostStatus& status)
    {
        (void)writer, (void)status;
        std::cout << "Liveliness lost. Matched Subscribers will consider us offline" << std::endl;
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
        if (nullptr != data_writer_with_default_qos)
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
        if (nullptr != data_writer_with_custom_qos)
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
        if (nullptr != data_writer_with_default_qos_and_custom_listener)
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
        if (nullptr != data_writer_with_profile)
        {
            // Error
            return;
        }

        // Create a DataWriter using a profile and a custom Listener.
        // CustomDataWriterListener inherits from DataWriterListener.
        CustomDataWriterListener custom_listener;
        DataWriter* data_writer_with_profile_and_custom_listener =
                publisher->create_datawriter_with_profile(topic, "data_writer_profile", &custom_listener);
        if (nullptr != data_writer_with_profile_and_custom_listener)
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
        if (nullptr != data_writer)
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
        if (nullptr != data_writer)
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
        if(data_writer->set_qos(publisher->get_default_datawriter_qos())
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
        if (nullptr != data_writer)
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
        if(publisher->set_default_datawriter_qos(qos_type1) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataWriter with the new default DataWriterQos.
        DataWriter* data_writer_with_qos_type1 =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr != data_writer_with_qos_type1)
        {
            // Error
            return;
        }

        // Get the current QoS or create a new one from scratch
        DataWriterQos qos_type2;

        // Modify QoS attributes
        // (...)

        // Set as the new default DataWriterQos
        if(publisher->set_default_datawriter_qos(qos_type2) != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // Create a DataWriter with the new default DataWriterQos.
        DataWriter* data_writer_with_qos_type2 =
                publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr != data_writer_with_qos_type2)
        {
            // Error
            return;
        }

        // Resetting the default DataWriterQos to the original default constructed values
        if(publisher->set_default_datawriter_qos(DATAWRITER_QOS_DEFAULT)
                != ReturnCode_t::RETCODE_OK)
        {
            // Error
            return;
        }

        // The previous instruction is equivalent to the following
        if(publisher->set_default_datawriter_qos(DataWriterQos())
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
        if (nullptr != custom_topic)
        {
            // Error
            return;
        }

        // Create a DataWriter
        DataWriter* data_writer =
                publisher->create_datawriter(custom_topic, DATAWRITER_QOS_DEFAULT);
        if (nullptr != data_writer)
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
    }
}
