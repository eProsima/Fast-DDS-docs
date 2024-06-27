#include <fstream>

#include <fastdds/dds/log/FileConsumer.hpp>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/rtps/history/ReaderHistory.hpp>
#include <fastdds/rtps/history/WriterHistory.hpp>
#include <fastdds/rtps/participant/RTPSParticipant.hpp>
#include <fastdds/rtps/reader/ReaderListener.hpp>
#include <fastdds/rtps/reader/RTPSReader.hpp>
#include <fastdds/rtps/RTPSDomain.hpp>
#include <fastdds/rtps/writer/RTPSWriter.hpp>
#include <fastdds/utils/IPLocator.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds;
using namespace ::rtps;

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
            const void* const data,
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
            const void* const data) override
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
            const void* const data,
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
        reader->get_history()->remove_change((CacheChange_t*)change);
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
        // etc.
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
    // Request a change from the history
    CacheChange_t* change = history->create_change(255, ALIVE);
    // Write serialized data into the change
    change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2) + 1;
    // Insert change into the history. The Writer takes care of the rest.
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
                SerializedPayload_t& payload) override
        {
            // Reserve new memory for the payload buffer
            octet* payload_buff = new octet[size];

            // Assign the payload buffer to the CacheChange and update sizes
            payload.data = payload_buff;
            payload.length = size;
            payload.max_size = size;

            // Tell the CacheChange who needs to release its payload
            payload.payload_owner = this;

            return true;
        }

        bool get_payload(
                const SerializedPayload_t& data,
                SerializedPayload_t& payload)
        {
            // Reserve new memory for the payload buffer
            octet* payload_buff = new octet[data.length];

            // Copy the data
            memcpy(payload_buff, data.data, data.length);

            // Tell the CacheChange who needs to release its payload
            payload.payload_owner = this;

            // Assign the payload buffer to the CacheChange and update sizes
            payload.data = payload_buff;
            payload.length = data.length;
            payload.max_size = data.length;

            return true;
        }

        bool release_payload(
                SerializedPayload_t& payload) override
        {
            // Ensure precondition
            if (this != payload.payload_owner)
            {
                std::cerr << "Trying to release a payload buffer allocated by a different PayloadPool." << std::endl;
                return false;
            }

            // Dealloc the buffer of the payload
            delete[] payload.data;

            // Reset sizes and pointers
            payload.data = nullptr;
            payload.length = 0;
            payload.max_size = 0;

            // Reset the owner of the payload
            payload.payload_owner = nullptr;

            return true;
        }

    };

    std::shared_ptr<CustomPayloadPool> payload_pool = std::make_shared<CustomPayloadPool>();

    // A writer using the custom payload pool
    HistoryAttributes writer_history_attr;
    WriterHistory* writer_history = new WriterHistory(writer_history_attr, payload_pool);
    WriterAttributes writer_attr;
    RTPSWriter* writer = RTPSDomain::createRTPSWriter(participant, writer_attr, writer_history);

    // A reader using the same instance of the custom payload pool
    HistoryAttributes reader_history_attr;
    ReaderHistory* reader_history = new ReaderHistory(reader_history_attr);
    ReaderAttributes reader_attr;
    RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, payload_pool, reader_history);

    // Write and Read operations work as usual, but take the Payloads from the pool.
    // Requesting a change to the Writer will provide one with an empty Payload taken from the pool
    CacheChange_t* change = writer_history->create_change(255, ALIVE);

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
    history_attr.payloadMaxSize  = 250; // Defaults to 500 bytes
    //!--

    //RTPS_API_HISTORY_CONF_RESOURCES
    history_attr.initialReservedCaches = 250; // Defaults to 500
    history_attr.maximumReservedCaches = 500; // Defaults to 0 = Unlimited Changes
    //!--
}

bool permissions_test(
        std::string main_ca_file,
        std::string appcert_file,
        std::string appkey_file,
        std::string governance_file,
        std::string permissions_file)
{
    eprosima::fastdds::rtps::RTPSParticipantAttributes part_attr;

    // Activate Auth:PKI-DH plugin
    part_attr.properties.properties().emplace_back("dds.sec.auth.plugin",
            "builtin.PKI-DH");

    // Configure Auth:PKI-DH plugin
    part_attr.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
            main_ca_file);
    part_attr.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
            appcert_file);
    part_attr.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key",
            appkey_file);

    part_attr.properties.properties().emplace_back("dds.sec.access.plugin",
            "builtin.Access-Permissions");

    // Configure DDS:Access:Permissions plugin
    part_attr.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions_ca",
        main_ca_file);
    part_attr.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.governance",
        governance_file);
    part_attr.properties.properties().emplace_back(
        "dds.sec.access.builtin.Access-Permissions.permissions",
        permissions_file);
    RTPSParticipant* participant = RTPSDomain::createParticipant(0, part_attr);
    if (participant != nullptr)
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
