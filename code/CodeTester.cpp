
#include <fastdds/rtps/history/WriterHistory.h>
#include <fastdds/rtps/history/ReaderHistory.h>
#include <fastdds/rtps/reader/RTPSReader.h>
#include <fastdds/rtps/reader/ReaderListener.h>
#include <fastdds/rtps/writer/RTPSWriter.h>
#include <fastdds/subscriber/SampleInfo.h>
#include <fastrtps/log/Log.h>
#include <fastrtps/log/FileConsumer.h>
#include <fastrtps/TopicDataType.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>

#include <fstream>

using namespace eprosima::fastrtps;
using namespace ::rtps;
using namespace ::security;
using namespace ::types;

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

class HelloWorldPubSubType : public TopicDataType
{
    bool serialize(
            void* data,
            rtps::SerializedPayload_t* payload) override
    {
        return false;
    }

    bool deserialize(
            rtps::SerializedPayload_t* payload,
            void* data) override
    {
        return false;
    }

    std::function<uint32_t()> getSerializedSizeProvider(
            void* data) override
    {
        return []
               {
                   return 0;
               };
    }

    void* createData() override
    {
        return nullptr;
    }

    void deleteData(
            void* data) override
    {
    }

    bool getKey(
            void* data,
            rtps::InstanceHandle_t* ihandle,
            bool force_md5 = false) override
    {
        return false;
    }

};

//RTPS_API_READER_LISTENER
class MyReaderListener : public ReaderListener
{
public:

    MyReaderListener()
    {
    }

    ~MyReaderListener()
    {
    }

    void onNewCacheChangeAdded(
            RTPSReader* reader,
            const CacheChange_t* const change)
    {
        // The incoming message is enclosed within the `change` in the function parameters
        printf("%s\n", change->serializedPayload.data);
        // Once done, remove the change
        reader->getHistory()->remove_change((CacheChange_t*)change);
    }

};
//!--

void rtps_setup_transports_example()
{
    //RTPS_SETUP_TRANSPORTS_EXAMPLE
    RTPSParticipantAttributes participant_attr;
    participant_attr.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);
    RTPSParticipant* participant = RTPSDomain::createParticipant(0, participant_attr);
    //!--
}

void rtps_api_example_create_entities()
{
    //RTPS_API_CREATE_PARTICIPANT
    RTPSParticipantAttributes participant_attr;
    participant_attr.setName("participant");
    RTPSParticipant* participant = RTPSDomain::createParticipant(0, participant_attr);
    //!--

    {
        //RTPS_API_CONF_PARTICIPANT
        RTPSParticipantAttributes participant_attr;
        participant_attr.setName("my_participant");
        //etc.
        //!--
    }

    //RTPS_API_WRITER_CONF_HISTORY
    HistoryAttributes history_attr;
    WriterHistory* history = new WriterHistory(history_attr);
    WriterAttributes writer_attr;
    RTPSWriter* writer = RTPSDomain::createRTPSWriter(participant, writer_attr, history);
    //!--

    {
        //RTPS_API_READER_CONF_HISTORY
        class MyReaderListener : public ReaderListener
        {
            // Callbacks override
        };
        MyReaderListener listener;
        HistoryAttributes history_attr;
        ReaderHistory* history = new ReaderHistory(history_attr);
        ReaderAttributes reader_attr;
        RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, history, &listener);
        //!--
    }

    //RTPS_API_WRITE_SAMPLE
    //Request a change from the writer
    CacheChange_t* change = writer->new_change([]() -> uint32_t
                    {
                        return 255;
                    }, ALIVE);
    //Write serialized data into the change
    change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2) + 1;
    //Insert change into the history. The Writer takes care of the rest.
    history->add_change(change);
    //!--
}

void rtps_api_example_create_entities_with_custom_pool()
{
    RTPSParticipantAttributes participant_attr;
    participant_attr.setName("participant");
    RTPSParticipant* participant = RTPSDomain::createParticipant(0, participant_attr);

    //RTPS_API_ENTITY_CREATE_WITH_PAYLOAD_POOL
    // A simple payload pool that reserves and frees memory each time
    class CustomPayloadPool : public IPayloadPool
    {
        bool get_payload(
                uint32_t size,
                CacheChange_t& cache_change) override
        {
            // Reserve new memory for the payload buffer
            octet* payload = new octet[size];

            // Assign the payload buffer to the CacheChange and update sizes
            cache_change.serializedPayload.data = payload;
            cache_change.serializedPayload.length = size;
            cache_change.serializedPayload.max_size = size;

            // Tell the CacheChange who needs to release its payload
            cache_change.payload_owner(this);

            return true;
        }

        bool get_payload(
                SerializedPayload_t& data,
                IPayloadPool*& /*data_owner*/,
                CacheChange_t& cache_change)
        {
            // Reserve new memory for the payload buffer
            octet* payload = new octet[data.length];

            // Copy the data
            memcpy(payload, data.data, data.length);

            // Tell the CacheChange who needs to release its payload
            cache_change.payload_owner(this);

            // Assign the payload buffer to the CacheChange and update sizes
            cache_change.serializedPayload.data = payload;
            cache_change.serializedPayload.length = data.length;
            cache_change.serializedPayload.max_size = data.length;

            return true;
        }

        bool release_payload(
                CacheChange_t& cache_change) override
        {
            // Ensure precondition
            if (this != cache_change.payload_owner())
            {
                std::cerr << "Trying to release a payload buffer allocated by a different PayloadPool." << std::endl;
                return false;
            }

            // Dealloc the buffer of the payload
            delete[] cache_change.serializedPayload.data;

            // Reset sizes and pointers
            cache_change.serializedPayload.data = nullptr;
            cache_change.serializedPayload.length = 0;
            cache_change.serializedPayload.max_size = 0;

            // Reset the owner of the payload
            cache_change.payload_owner(nullptr);

            return true;
        }

    };

    std::shared_ptr<CustomPayloadPool> payload_pool = std::make_shared<CustomPayloadPool>();

    // A writer using the custom payload pool
    HistoryAttributes writer_history_attr;
    WriterHistory* writer_history = new WriterHistory(writer_history_attr);
    WriterAttributes writer_attr;
    RTPSWriter* writer = RTPSDomain::createRTPSWriter(participant, writer_attr, payload_pool, writer_history);

    // A reader using the same instance of the custom payload pool
    HistoryAttributes reader_history_attr;
    ReaderHistory* reader_history = new ReaderHistory(reader_history_attr);
    ReaderAttributes reader_attr;
    RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, payload_pool, reader_history);

    // Write and Read operations work as usual, but take the Payloads from the pool.
    // Requesting a change to the Writer will provide one with an empty Payload taken from the pool
    CacheChange_t* change = writer->new_change([]() -> uint32_t
                    {
                        return 255;
                    }, ALIVE);

    // Write serialized data into the change and add it to the history
    change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2) + 1;
    writer_history->add_change(change);
    //!--
}

void rtps_api_example_conf()
{
    WriterAttributes writer_attr;
    HistoryAttributes history_attr;

    //RTPS_API_WRITER_CONF_RELIABILITY
    writer_attr.endpoint.reliabilityKind = BEST_EFFORT;
    //!--

    //RTPS_API_WRITER_CONF_DURABILITY
    writer_attr.endpoint.durabilityKind = TRANSIENT_LOCAL;
    //!--

    //RTPS_API_HISTORY_CONF_PAYLOADMAXSIZE
    history_attr.payloadMaxSize  = 250;//Defaults to 500 bytes
    //!--

    //RTPS_API_HISTORY_CONF_RESOURCES
    history_attr.initialReservedCaches = 250; //Defaults to 500
    history_attr.maximumReservedCaches = 500; //Defaults to 0 = Unlimited Changes
    //!--
}

void xml_dyn_examples_check()
{
    //XML-DYN-ENUM
    DynamicTypeBuilder_ptr enum_builder = DynamicTypeBuilderFactory::get_instance()->create_enum_builder();
    enum_builder->set_name("MyEnum");
    enum_builder->add_empty_member(0, "A");
    enum_builder->add_empty_member(1, "B");
    enum_builder->add_empty_member(2, "C");
    DynamicType_ptr enum_type = DynamicTypeBuilderFactory::get_instance()->create_type(enum_builder.get());
    //!--
    //XML-TYPEDEF
    DynamicTypeBuilder_ptr alias1_builder = DynamicTypeBuilderFactory::get_instance()->create_alias_builder(
        enum_builder.get(), "MyAlias1");
    DynamicType_ptr alias1_type = DynamicTypeBuilderFactory::get_instance()->create_type(alias1_builder.get());

    std::vector<uint32_t> sequence_lengths = { 2, 2 };
    DynamicTypeBuilder_ptr int_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
    DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::get_instance()->create_array_builder(
        int_builder.get(), sequence_lengths);
    DynamicTypeBuilder_ptr alias2_builder = DynamicTypeBuilderFactory::get_instance()->create_alias_builder(
        array_builder.get(), "MyAlias2");
    DynamicType_ptr alias2_type = DynamicTypeBuilderFactory::get_instance()->create_type(alias2_builder.get());
    //!--
    {
        //XML-STRUCT
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::get_instance()->create_int64_builder();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();

        struct_builder->set_name("MyStruct");
        struct_builder->add_member(0, "first", long_builder.get());
        struct_builder->add_member(1, "second", long_long_builder.get());
        DynamicType_ptr struct_type = DynamicTypeBuilderFactory::get_instance()->create_type(struct_builder.get());
        //!--
    }
    {
        //XML-STRUCT-INHERIT
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::get_instance()->create_int64_builder();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();

        struct_builder->set_name("ParentStruct");
        struct_builder->add_member(0, "first", long_builder.get());
        struct_builder->add_member(1, "second", long_long_builder.get());
        DynamicType_ptr struct_type = DynamicTypeBuilderFactory::get_instance()->create_type(struct_builder.get());

        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance()->create_child_struct_builder(struct_builder.get());

        child_builder->set_name("ChildStruct");
        child_builder->add_member(0, "third", long_builder.get());
        child_builder->add_member(1, "fourth", long_long_builder.get());
        DynamicType_ptr child_struct_type = DynamicTypeBuilderFactory::get_instance()->create_type(child_builder.get());
        //!--
    }
    {
        //XML-UNION
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::get_instance()->create_int64_builder();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        DynamicTypeBuilder_ptr octet_builder = DynamicTypeBuilderFactory::get_instance()->create_byte_builder();
        DynamicTypeBuilder_ptr union_builder = DynamicTypeBuilderFactory::get_instance()->create_union_builder(
            octet_builder.get());

        union_builder->set_name("MyUnion");
        union_builder->add_member(0, "first", long_builder.get(), "", { 0, 1 }, false);
        union_builder->add_member(1, "second", struct_builder.get(), "", { 2 }, false);
        union_builder->add_member(2, "third", long_long_builder.get(), "", { }, true);
        DynamicType_ptr union_type = DynamicTypeBuilderFactory::get_instance()->create_type(union_builder.get());
        //!--
    }
    {
        //XML-GENERIC
        DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::get_instance()->create_int64_builder();
        long_long_builder->set_name("my_long");
        DynamicType_ptr long_long_type =
                DynamicTypeBuilderFactory::get_instance()->create_type(long_long_builder.get());
        //!--
    }
    {
        //XML-BOUNDEDSTRINGS
        DynamicTypeBuilder_ptr string_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(41925);
        string_builder->set_name("my_large_string");
        DynamicType_ptr string_type = DynamicTypeBuilderFactory::get_instance()->create_type(string_builder.get());

        DynamicTypeBuilder_ptr wstring_builder =
                DynamicTypeBuilderFactory::get_instance()->create_wstring_builder(20925);
        wstring_builder->set_name("my_large_wstring");
        DynamicType_ptr wstring_type = DynamicTypeBuilderFactory::get_instance()->create_type(wstring_builder.get());
        //!--
    }
    {
        //XML-ARRAYS
        std::vector<uint32_t> lengths = { 2, 3, 4 };
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::get_instance()->create_array_builder(
            long_builder.get(), lengths);
        array_builder->set_name("long_array");
        DynamicType_ptr array_type = DynamicTypeBuilderFactory::get_instance()->create_type(array_builder.get());
        //!--
    }
    {
        //XML-SEQUENCES
        uint32_t child_len = 2;
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr seq_builder = DynamicTypeBuilderFactory::get_instance()->create_sequence_builder(
            long_builder.get(),
            child_len);
        uint32_t length = 3;
        DynamicTypeBuilder_ptr seq_seq_builder = DynamicTypeBuilderFactory::get_instance()->create_sequence_builder(
            seq_builder.get(), length);
        seq_seq_builder->set_name("my_sequence_sequence");
        DynamicType_ptr seq_type = DynamicTypeBuilderFactory::get_instance()->create_type(seq_seq_builder.get());
        //!--
    }
    {
        //XML-MAPS
        uint32_t length = 2;
        DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicTypeBuilder_ptr map_builder = DynamicTypeBuilderFactory::get_instance()->create_map_builder(
            long_builder.get(),
            long_builder.get(), length);

        DynamicTypeBuilder_ptr map_map_builder = DynamicTypeBuilderFactory::get_instance()->create_map_builder(
            long_builder.get(),
            map_builder.get(), length);
        map_map_builder->set_name("my_map_map");
        DynamicType_ptr map_type = DynamicTypeBuilderFactory::get_instance()->create_type(map_map_builder.get());
        //!--
    }
    {
        //XML-BITSET
        DynamicTypeBuilderFactory* m_factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = m_factory->create_bitset_builder();
        builder_ptr->add_member(0, "a", m_factory->create_byte_builder()->build());
        builder_ptr->add_member(1, "b", m_factory->create_bool_builder()->build());
        builder_ptr->add_member(3, "c", m_factory->create_uint16_builder()->build());
        builder_ptr->add_member(4, "d", m_factory->create_int16_builder()->build());
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_BIT_BOUND_ID, "value", "3");
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_POSITION_ID, "value", "0");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_BIT_BOUND_ID, "value", "1");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_POSITION_ID, "value", "3");
        builder_ptr->apply_annotation_to_member(3, ANNOTATION_BIT_BOUND_ID, "value", "10");
        builder_ptr->apply_annotation_to_member(3, ANNOTATION_POSITION_ID, "value", "8"); // 4 empty
        builder_ptr->apply_annotation_to_member(4, ANNOTATION_BIT_BOUND_ID, "value", "12");
        builder_ptr->apply_annotation_to_member(4, ANNOTATION_POSITION_ID, "value", "18");
        builder_ptr->set_name("MyBitSet");
        //!--
    }
    {
        //XML-BITMASK
        DynamicTypeBuilderFactory* m_factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = m_factory->create_bitmask_builder(8);
        builder_ptr->add_empty_member(0, "flag0");
        builder_ptr->add_empty_member(1, "flag1");
        builder_ptr->add_empty_member(2, "flag2");
        builder_ptr->add_empty_member(5, "flag5");
        builder_ptr->set_name("MyBitMask");
        //!--
    }
    {
        //XML-BITSET-INHERIT
        DynamicTypeBuilderFactory* m_factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = m_factory->create_bitset_builder();
        builder_ptr->add_member(0, "a", m_factory->create_byte_builder()->build());
        builder_ptr->add_member(1, "b", m_factory->create_bool_builder()->build());
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_BIT_BOUND_ID, "value", "3");
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_POSITION_ID, "value", "0");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_BIT_BOUND_ID, "value", "1");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_POSITION_ID, "value", "3");
        builder_ptr->set_name("ParentBitSet");

        DynamicTypeBuilder_ptr child_ptr = m_factory->create_child_struct_builder(builder_ptr.get());
        child_ptr->add_member(3, "c", m_factory->create_uint16_builder()->build());
        child_ptr->add_member(4, "d", m_factory->create_int16_builder()->build());
        child_ptr->apply_annotation_to_member(3, ANNOTATION_BIT_BOUND_ID, "value", "10");
        child_ptr->apply_annotation_to_member(3, ANNOTATION_POSITION_ID, "value", "8"); // 4 empty
        child_ptr->apply_annotation_to_member(4, ANNOTATION_BIT_BOUND_ID, "value", "12");
        child_ptr->apply_annotation_to_member(4, ANNOTATION_POSITION_ID, "value", "18");
        child_ptr->set_name("ChildBitSet");
        //!--
    }
    {
        //XML-USAGE
        // Load the XML File
        if (eprosima::fastrtps::xmlparser::XMLP_ret::XML_OK !=
                eprosima::fastrtps::xmlparser::XMLProfileManager::loadXMLFile("types.xml"))
        {
            std::cout << "Cannot open XML file \"types.xml\". "
                      << "Please, set the correct path to the XML file"
                      << std::endl;
        }
        // Create the "MyStructPubSubType"
        eprosima::fastrtps::types::DynamicPubSubType* pbType =
                eprosima::fastrtps::xmlparser::XMLProfileManager::CreateDynamicPubSubType("MyStruct");
        // Create a "MyStruct" instance
        eprosima::fastrtps::types::DynamicData* data =
                eprosima::fastrtps::types::DynamicDataFactory::get_instance()->create_data(
            pbType->GetDynamicType());
        //!--
    }
}

void security_configuration()
{
    //SECURITY_CONF_ALL_PLUGINS
    eprosima::fastrtps::ParticipantAttributes part_attr;

    // Activate Auth:PKI-DH plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");

    // Configure Auth:PKI-DH plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
            "file://maincacert.pem");
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
            "file://appcert.pem");
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key", "file://appkey.pem");

    // Activate Access:Permissions plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.access.plugin", "builtin.Access-Permissions");

    // Configure Access:Permissions plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions_ca",
            "file://maincacet.pem");
    part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.governance",
            "file://governance.smime");
    part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions",
            "file://permissions.smime");

    // Activate Crypto:AES-GCM-GMAC plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");
    //!--

    {
        //SECURITY_CONF_AUTH_AND_CRYPT_PLUGINS
        eprosima::fastrtps::ParticipantAttributes part_attr;

        // Activate Auth:PKI-DH plugin
        part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");

        // Configure Auth:PKI-DH plugin
        part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
                "file://maincacert.pem");
        part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
                "file://appcert.pem");
        part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key",
                "file://appkey.pem");

        // Activate Crypto:AES-GCM-GMAC plugin
        part_attr.rtps.properties.properties().emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");

        // Encrypt all RTPS submessages
        part_attr.rtps.properties.properties().emplace_back("rtps.participant.rtps_protection_kind", "ENCRYPT");
        //!--

        //SECURITY_CONF_PUBLISHER_AUTH_AND_CRYPT_PLUGINS
        eprosima::fastrtps::PublisherAttributes pub_attr;

        // Encrypt RTPS submessages
        pub_attr.properties.properties().emplace_back("rtps.endpoint.submessage_protection_kind", "ENCRYPT");

        // Encrypt payload
        pub_attr.properties.properties().emplace_back("rtps.endpoint.payload_protection_kind", "ENCRYPT");
        //!--

        //SECURITY_CONF_SUBSCRIBER_AUTH_AND_CRYPT_PLUGINS
        eprosima::fastrtps::SubscriberAttributes sub_attr;

        // Encrypt RTPS submessages
        sub_attr.properties.properties().emplace_back("rtps.endpoint.submessage_protection_kind", "ENCRYPT");
        //!--
    }

}

void dynamictypes_configuration()
{
    {
        //DYNAMIC_TYPES_QUICK_EXAMPLE
        // Create a builder for a specific type
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_enum_builder();

        // Use the builder to configure the type
        builder->add_empty_member(0, "DEFAULT");
        builder->add_empty_member(1, "FIRST");
        builder->add_empty_member(2, "SECOND");

        // Create the data type using the builder
        // The builder will internally use the DynamicTypeBuilderFactory to create the type
        DynamicType_ptr type = builder->build();

        // Create a new data instance of the create data type
        DynamicData_ptr data (DynamicDataFactory::get_instance()->create_data(type));

        // Now we can set or read data values
        data->set_int32_value(1);

        // No need of deleting the objects, since we used the
        // automanaged smart pointers
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_PRIMITIVES
        // Using Builders
        DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::get_instance()->create_int32_builder();
        DynamicType_ptr created_type = DynamicTypeBuilderFactory::get_instance()->create_type(created_builder.get());
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(created_type);
        data->set_int32_value(1);

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pType);
        data2->set_int32_value(1);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_STRINGS
        // Using Builders
        DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(100);
        DynamicType_ptr created_type = DynamicTypeBuilderFactory::get_instance()->create_type(created_builder.get());
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(created_type);
        data->set_string_value("Dynamic String");

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_string_type(100);
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pType);
        data2->set_string_value("Dynamic String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ALIAS
        // Create the base type
        DynamicTypeBuilder_ptr base_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(100);
        DynamicType_ptr base_type = DynamicTypeBuilderFactory::get_instance()->create_type(base_builder.get());

        // Create alias using Builders
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_alias_builder(base_type,
                        "alias");
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(builder.get());
        data->set_string_value("Dynamic Alias String");

        // Create alias type directly
        DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::get_instance()->create_alias_type(base_type, "alias");
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pAliasType);
        data2->set_string_value("Dynamic Alias String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ENUMERATIONS
        // Add enumeration values using the DynamicTypeBuilder
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_enum_builder();
        builder->add_empty_member(0, "DEFAULT");
        builder->add_empty_member(1, "FIRST");
        builder->add_empty_member(2, "SECOND");

        // Create the data instance
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(builder.get());

        // Access value using the name
        std::string sValue = "SECOND";
        data->set_enum_value(sValue);
        std::string sStoredValue;
        data->get_enum_value(sStoredValue, MEMBER_ID_INVALID);

        // Access value using the index
        uint32_t uValue = 2;
        data->set_enum_value(uValue);
        uint32_t uStoredValue;
        data->get_enum_value(uStoredValue, MEMBER_ID_INVALID);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_BITSETS
        // Create bitfields with the appropriate type for their size
        DynamicTypeBuilder_ptr base_type_byte_builder =
                DynamicTypeBuilderFactory::get_instance()->create_byte_builder();
        auto base_type_byte = base_type_byte_builder->build();

        DynamicTypeBuilder_ptr base_type_uint32_builder =
                DynamicTypeBuilderFactory::get_instance()->create_uint32_builder();
        auto base_type_uint32 = base_type_uint32_builder->build();

        // Create the bitset with two bitfields
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_bitset_builder();
        builder->add_member(0, "byte", base_type_byte);
        builder->add_member(1, "uint32", base_type_uint32);

        // Apply members' annotations
        builder->apply_annotation_to_member(0, ANNOTATION_POSITION_ID, "value", "0");   // "byte" starts at position 0
        builder->apply_annotation_to_member(0, ANNOTATION_BIT_BOUND_ID, "value", "2");  // "byte" is 2 bit length
        builder->apply_annotation_to_member(1, ANNOTATION_POSITION_ID, "value", "10");  // "uint32" starts at position 10 (8 bits empty)
        builder->apply_annotation_to_member(1, ANNOTATION_BIT_BOUND_ID, "value", "20"); // "uint32" is 20 bits length

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(builder.get()));

        // Access values
        data->set_byte_value(234, 0);
        data->set_uint32_value(2340, 1);
        octet bValue;
        uint32_t uValue;
        data->get_byte_value(bValue, 0);
        data->get_uint32_value(uValue, 1);
        //!--
        //DYNAMIC_TYPES_CREATE_BITSETS-INHERIT
        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance()->create_child_struct_builder(builder.get());
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_BITMASKS
        uint32_t limit = 5; // Stores as "octet"

        // Add bitmask flags using the DynamicTypeBuilder
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_bitmask_builder(limit);
        builder->add_empty_member(0, "FIRST");
        builder->add_empty_member(1, "SECOND");

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(builder.get()));

        // Access the mask values using the name
        data->set_bool_value(true, "FIRST");                // Set the "FIRST" bit
        bool bSecondValue = data->get_bool_value("SECOND"); // Get the "SECOND" bit

        // Access the mask values using the index
        data->set_bool_value(true, 1);                      // Set the "SECOND" bit
        bool bFirstValue = data->get_bool_value(0);         // Get the "FIRST" bit

        // Get the complete bitmask as integer
        uint64_t fullValue;
        data->get_bitmask_value(fullValue);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_STRUCTS
        // Build a structure with two fields ("first" as int32, "other" as uint64) using DynamicTypeBuilder
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
        DynamicType_ptr struct_type(builder->build());

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(struct_type));

        // Access struct members
        data->set_int32_value(5, 0);
        data->set_uint64_value(13, 1);
        //!--
        //DYNAMIC_TYPES_CREATE_STRUCTS-INHERIT
        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance()->create_child_struct_builder(builder.get());
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_UNIONS
        // Create the union DynamicTypeBuilder with an int32 discriminator
        DynamicType_ptr discriminator = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_union_builder(discriminator);

        // Add the union members. "firts" will be the default value
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type(), "", { 0 },
                true);
        builder->add_member(0, "second", DynamicTypeBuilderFactory::get_instance()->create_int64_type(), "", { 1 },
                false);

        // Create the data instance
        DynamicType_ptr union_type = builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(union_type));

        // Access the values using the member index
        data->set_int32_value(9, 0);
        data->set_int64_value(13, 1);

        // Get the label of the currently selected member
        uint64_t unionLabel;
        data->get_union_label(unionLabel);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_SEQUENCES
        // Create a DynamicTypeBuilder for a sequence of two elements of type inte32
        uint32_t length = 2;
        DynamicType_ptr base_type = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance()->create_sequence_builder(base_type, length);

        // Create the data instance
        DynamicType_ptr sequence_type = builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(sequence_type));

        // Insert and remove elements
        MemberId newId, newId2;
        data->insert_int32_value(10, newId);
        data->insert_int32_value(12, newId2);
        data->remove_sequence_data(newId);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ARRAYS
        // Create an array DynamicTypeBuilder for a 2x2 elements of type int32
        std::vector<uint32_t> lengths = { 2, 2 };
        DynamicType_ptr base_type = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance()->create_array_builder(base_type, lengths);

        // Create the data instance
        DynamicType_ptr array_type = builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(array_type));

        // Access elements in the multidimensional array
        MemberId pos = data->get_array_index({1, 0});
        data->set_int32_value(11, pos);
        data->set_int32_value(27, pos + 1);
        data->clear_array_data(pos);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_MAPS
        // Create DynamicTypeBuilder for a map of two pairs of {key:int32, value:int32}
        uint32_t length = 2;
        DynamicType_ptr base = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance()->create_map_builder(base, base, length);

        // Create the data instance
        DynamicType_ptr map_type = builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(map_type));

        // Add a new element to the map with key 1
        DynamicData_ptr key(DynamicDataFactory::get_instance()->create_data(base));
        MemberId keyId;
        MemberId valueId;
        key->set_int32_value(1);
        data->insert_map_data(key.get(), keyId, valueId);

        // Add a new element to the map with key 2
        // insert_map_data creates a copy of the key, so the same instance can be reused
        MemberId keyId2;
        MemberId valueId2;
        key->set_int32_value(2);
        data->insert_map_data(key.get(), keyId2, valueId2);

        // Set the value to the element with key 2, using the returned value Id
        data->set_int32_value(53, valueId2);

        // Remove elements from the map
        data->remove_map_data(keyId);
        data->remove_map_data(keyId2);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_NESTED_STRUCTS
        // Create a struct type
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
        DynamicType_ptr struct_type = builder->build();

        // Create a struct type with the previous struct as member
        DynamicTypeBuilder_ptr parent_builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        parent_builder->add_member(0, "child_struct", struct_type);
        parent_builder->add_member(1, "second", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(parent_builder.get()));

        // Access the child struct with the loan operations
        DynamicData* child_data = data->loan_value(0);
        child_data->set_int32_value(5, 0);
        child_data->set_uint64_value(13, 1);
        data->return_loaned_value(child_data);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_INHERITANCE_STRUCTS
        // Create a base struct type
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());

        // Create a struct type derived from the previous struct
        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance()->create_child_struct_builder(builder.get());

        // Add new members to the derived type
        builder->add_member(2, "third", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());

        // Create the data instance
        DynamicType_ptr struct_type = child_builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(struct_type));

        // The derived type includes the members defined on the base type
        data->set_int32_value(5, 0);
        data->set_uint64_value(13, 1);
        data->set_uint64_value(47, 2);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_NESTED_ALIAS
        // Using Builders
        DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(100);
        DynamicType_ptr created_type = DynamicTypeBuilderFactory::get_instance()->create_type(created_builder.get());
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_alias_builder(
            created_builder.get(), "alias");
        DynamicTypeBuilder_ptr builder2 = DynamicTypeBuilderFactory::get_instance()->create_alias_builder(
            builder.get(), "alias2");
        DynamicData* data(DynamicDataFactory::get_instance()->create_data(builder2->build()));
        data->set_string_value("Dynamic Alias 2 String");

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_string_type(100);
        DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::get_instance()->create_alias_type(pType, "alias");
        DynamicType_ptr pAliasType2 =
                DynamicTypeBuilderFactory::get_instance()->create_alias_type(pAliasType, "alias2");
        DynamicData* data2(DynamicDataFactory::get_instance()->create_data(pAliasType));
        data2->set_string_value("Dynamic Alias 2 String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_NESTED_UNIONS
        // Create a union DynamicTypeBuilder
        DynamicType_ptr discriminator = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_union_builder(discriminator);

        // Add a int32 to the union
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type(), "", { 0 },
                true);

        // Create a struct type and add it to the union
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        struct_builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
        struct_builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
        builder->add_member(1, "first", struct_builder.get(), "", { 1 }, false);

        // Create the union data instance
        DynamicType_ptr union_type = builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(union_type));

        // Access the struct member using the loan operations
        DynamicData* child_data = data->loan_value(1);
        child_data->set_int32_value(9, 0);
        child_data->set_int64_value(13, 1);
        data->return_loaned_value(child_data);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ANNOTATION
        // Apply the annotation
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
        //...
        builder->apply_annotation("MyAnnotation", "value", "5");
        builder->apply_annotation("MyAnnotation", "name", "length");
        //!--
    }

    {
        //DYNAMIC_TYPES_SERIALIZATION
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicPubSubType pubsubType(pType);

        // SERIALIZATION EXAMPLE
        DynamicData* pData = DynamicDataFactory::get_instance()->create_data(pType);
        uint32_t payloadSize = static_cast<uint32_t>(pubsubType.getSerializedSizeProvider(pData)());
        SerializedPayload_t payload(payloadSize);
        pubsubType.serialize(pData, &payload);

        // DESERIALIZATION EXAMPLE
        types::DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pType);
        pubsubType.deserialize(&payload, data2);
        //!--
    }

    {
        //DYNAMIC_TYPES_NOTES_1
        DynamicTypeBuilder* pBuilder = DynamicTypeBuilderFactory::get_instance()->create_uint32_builder();
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicData* pData = DynamicDataFactory::get_instance()->create_data(pType);

        DynamicTypeBuilderFactory::get_instance()->delete_builder(pBuilder);
        DynamicDataFactory::get_instance()->delete_data(pData);
        //!--
    }

    {
        //DYNAMIC_TYPES_NOTES_2
        DynamicTypeBuilder_ptr pBuilder = DynamicTypeBuilderFactory::get_instance()->create_uint32_builder();
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance()->create_int32_type();
        DynamicData_ptr pData(DynamicDataFactory::get_instance()->create_data(pType));
        //!--
    }

    {
        //DYNAMIC_HELLO_WORLD_API
        // In HelloWorldPublisher.h
        // Dynamic Types
        eprosima::fastrtps::types::DynamicData* m_DynHello;
        eprosima::fastrtps::types::DynamicPubSubType m_DynType;

        // In HelloWorldPublisher.cpp
        // Create basic builders
        DynamicTypeBuilder_ptr struct_type_builder(DynamicTypeBuilderFactory::get_instance()->create_struct_builder());

        // Add members to the struct.
        struct_type_builder->add_member(0, "index", DynamicTypeBuilderFactory::get_instance()->create_uint32_type());
        struct_type_builder->add_member(1, "message", DynamicTypeBuilderFactory::get_instance()->create_string_type());
        struct_type_builder->set_name("HelloWorld");

        DynamicType_ptr dynType = struct_type_builder->build();
        m_DynType.SetDynamicType(dynType);
        m_DynHello = DynamicDataFactory::get_instance()->create_data(dynType);
        m_DynHello->set_uint32_value(0, 0);
        m_DynHello->set_string_value("HelloWorld", 1);
        //!--
    }

    {
        //DYNAMIC_HELLO_WORLD_API
        // In HelloWorldPublisher.h
        // Dynamic Types
        eprosima::fastrtps::types::DynamicData* m_DynHello;
        eprosima::fastrtps::types::DynamicPubSubType m_DynType;

        // In HelloWorldPublisher.cpp
        // Create basic builders
        DynamicTypeBuilder_ptr struct_type_builder(DynamicTypeBuilderFactory::get_instance()->create_struct_builder());

        // Add members to the struct.
        struct_type_builder->add_member(0, "index", DynamicTypeBuilderFactory::get_instance()->create_uint32_type());
        struct_type_builder->add_member(1, "message", DynamicTypeBuilderFactory::get_instance()->create_string_type());
        struct_type_builder->set_name("HelloWorld");

        DynamicType_ptr dynType = struct_type_builder->build();
        m_DynType.SetDynamicType(dynType);
        m_DynHello = DynamicDataFactory::get_instance()->create_data(dynType);
        m_DynHello->set_uint32_value(0, 0);
        m_DynHello->set_string_value("HelloWorld", 1);
        //!--
    }

}

bool permissions_test(
        std::string main_ca_file,
        std::string appcert_file,
        std::string appkey_file,
        std::string governance_file,
        std::string permissions_file)
{
    eprosima::fastrtps::ParticipantAttributes part_attr;

    // Activate Auth:PKI-DH plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.plugin",
        "builtin.PKI-DH");

    // Configure Auth:PKI-DH plugin
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
        main_ca_file);
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
        appcert_file);
    part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key",
        appkey_file);

    part_attr.rtps.properties.properties().emplace_back("dds.sec.access.plugin",
        "builtin.Access-Permissions");

    // Configure DDS:Access:Permissions plugin
    part_attr.rtps.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions_ca",
        main_ca_file);
    part_attr.rtps.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.governance",
        governance_file);
    part_attr.rtps.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions",
        permissions_file);

    if (Domain::createParticipant(part_attr))
    {
        return true;
    }
    return false;
}

int main(
        int argc,
        const char** argv)
{
    if (argc != 2 && argc != 6)
    {
        printf("Bad number of parameters\n");
        exit(-1);
    }

    // Also show log warnings to spot potential mistakes
    Log::SetVerbosity(Log::Kind::Warning);
    // Report filename and line number for debugging
    Log::ReportFilenames(true);

    int exit_code = 0;
    if (argc == 6)
    {
        if (!permissions_test(argv[1], argv[2], argv[3], argv[4], argv[5]))
        {
            printf("Error parsing persimission xml file\n");
            exit_code = -1;
        }
    }

    // Make sure all logs are displayed before exiting
    Log::Flush();

    exit(exit_code);
}
