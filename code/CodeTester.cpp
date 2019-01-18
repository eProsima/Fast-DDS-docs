#include <fastrtps/Domain.h>
#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/xmlparser/XMLEndpointParser.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/log/Log.h>
#include <fastrtps/log/FileConsumer.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <cpp/security/accesscontrol/GovernanceParser.h>
#include <cpp/security/accesscontrol/PermissionsParser.h>

#include <fstream>

using namespace eprosima::fastrtps;
using namespace ::rtps;
using namespace ::xmlparser;
using namespace ::security;

class HelloWorld
{
    public:

        void msg(const std::string&) {}

        std::string msg() { return ""; }
};

class HelloWorldPubSubType : public TopicDataType
{
    bool serialize(void* data, rtps::SerializedPayload_t* payload) override { return false; }

    bool deserialize(rtps::SerializedPayload_t* payload, void* data) override { return false; }

    std::function<uint32_t()> getSerializedSizeProvider(void* data) override { return []{ return 0; }; }

    void * createData() override { return nullptr; }

    void deleteData(void * data) override {}

    bool getKey(void* data, rtps::InstanceHandle_t* ihandle, bool force_md5 = false) override { return false; }
};

//PUBSUB_API_PUBLISHER_LISTENER
class PubListener : public PublisherListener
{
    public:

        PubListener() {}
        ~PubListener() {}

        void onPublicationmatched(Publisher* pub, MatchingInfo& info)
        {
            //Callback implementation. This is called each time the Publisher finds a Subscriber on the network that listens to the same topic.
        }
};
//!--

//PUBSUB_API_SUBSCRIBER_LISTENER
class SubListener: public SubscriberListener
{
    public:

        SubListener() {}

        ~SubListener() {}

        void onNewDataMessage(Subscriber * sub)
        {
            if(sub->takeNextData((void*)&sample, &sample_info))
            {
                if(sample_info.sampleKind == ALIVE)
                {
                    std::cout << "New message: " << sample.msg() << std::endl;
                }
            }
        }

        HelloWorld sample; //Storage for incoming messages

        SampleInfo_t sample_info; //Auxiliary structure with meta-data on the message
};
//!--

void configuration_compilation_check()
{
ParticipantAttributes participant_attr;
PublisherAttributes publisher_attr;
SubscriberAttributes subscriber_attr;

//CONF-QOS-KEY
// Publisher-Subscriber Layer configuration.
publisher_attr.topic.topicKind = WITH_KEY;
//!--

//CONF-QOS-RESOURCELIMIT-INSTANCES
// Set the subscriber to remember and store up to 3 different keys.
subscriber_attr.topic.resourceLimitsQos.max_instances = 3;
// Hold a maximum of 20 samples per key.
subscriber_attr.topic.resourceLimitsQos.max_samples_per_instance = 20;
//!--

//CONF-COMMON-TRANSPORT-SETTING
//Create a descriptor for the new transport.
auto custom_transport = std::make_shared<UDPv4TransportDescriptor>();
    custom_transport->sendBufferSize = 9216;
    custom_transport->receiveBufferSize = 9216;

//Disable the built-in Transport Layer.
participant_attr.rtps.useBuiltinTransports = false;

//Link the Transport Layer to the Participant.
participant_attr.rtps.userTransports.push_back(custom_transport);
//!--

//CONF-TCP-TRANSPORT-SETTING
//Create a descriptor for the new transport.
auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();
tcp_transport->add_listener_port(5100);

//Disable the built-in Transport Layer.
participant_attr.rtps.useBuiltinTransports = false;

//Link the Transport Layer to the Participant.
participant_attr.rtps.userTransports.push_back(tcp_transport);
//!--

//CONF-TCP2-TRANSPORT-SETTING
auto tcp2_transport = std::make_shared<TCPv4TransportDescriptor>();

//Disable the built-in Transport Layer.
participant_attr.rtps.useBuiltinTransports = false;

//Set initial peers.
Locator_t initial_peer_locator;
initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
IPLocator::setIPv4(initial_peer_locator, "192.168.1.55");
initial_peer_locator.port = 5100;
participant_attr.rtps.builtin.initialPeersList.push_back(initial_peer_locator);

//Link the Transport Layer to the Participant.
participant_attr.rtps.userTransports.push_back(tcp2_transport);
//!--

{
//CONF-IPLOCATOR-USAGE
Locator_t locator;
// Get & Set Physical Port
uint16_t physical_port = IPLocator::getPhysicalPort(locator);
IPLocator::setPhysicalPort(locator, 5555);

// Get & Set Logical Port
uint16_t logical_port = IPLocator::getLogicalPort(locator);
IPLocator::setLogicalPort(locator, 7400);

// Set WAN Address
IPLocator::setWan(locator, "80.88.75.55");
//!--
}

{
//CONF-METAMULTICASTLOCATOR
// This locator will open a socket to listen network messages on UDPv4 port 22222 over multicast address 239.255.0.1
eprosima::fastrtps::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 239, 255, 0 , 1);
locator.port = 22222;

participant_attr.rtps.builtin.metatrafficMulticastLocatorList.push_back(locator);
//!--
}

{
//CONF-METAUNICASTLOCATOR
// This locator will open a socket to listen network messages on UDPv4 port 22223 over network interface 192.168.0.1
eprosima::fastrtps::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 192, 168, 0 , 1);
locator.port = 22223;

participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(locator);
//!--
}

{
//CONF-USERMULTICASTLOCATOR
// This locator will open a socket to listen network messages on UDPv4 port 22224 over multicast address 239.255.0.1
eprosima::fastrtps::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 239, 255, 0 , 1);
locator.port = 22224;

participant_attr.rtps.defaultMulticastLocatorList.push_back(locator);
//!--
}

{
//CONF-USERUNICASTLOCATOR
// This locator will open a socket to listen network messages on UDPv4 port 22225 over network interface 192.168.0.1
eprosima::fastrtps::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 192, 168, 0 , 1);
locator.port = 22225;

participant_attr.rtps.defaultUnicastLocatorList.push_back(locator);
//!--
}

{
//CONF-INITIALPEERS
// This locator configures as initial peer the UDPv4 address 192.168.0.2:7600.
// Initial discovery network messages will send to this UDPv4 address.
eprosima::fastrtps::rtps::Locator_t locator;
IPLocator::setIPv4(locator, "192.168.0.2");
locator.port = 7600;

participant_attr.rtps.builtin.initialPeersList.push_back(locator);
//!--
}

//CONF-DISABLE-MULTICAST
// Metatraffic Multicast Locator List will be empty.
// Metatraffic Unicast Locator List will contain one locator, with null address and null port.
// Then eProsima Fast RTPS will use all network interfaces to receive network messages using a well-known port.
Locator_t default_unicast_locator;
participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(default_unicast_locator);

// Initial peer will be UDPv4 addresss 192.168.0.1. The port will be a well-known port.
// Initial discovery network messages will be sent to this UDPv4 address.
Locator_t initial_peer;
IPLocator::setIPv4(initial_peer, 192, 168, 0, 1);
participant_attr.rtps.builtin.initialPeersList.push_back(initial_peer);
//!--

//CONF-QOS-FLOWCONTROLLER
// Limit to 300kb per second.
ThroughputControllerDescriptor slowPublisherThroughputController{300000, 1000};
publisher_attr.throughputController = slowPublisherThroughputController;
//!--

//CONF_QOS_RTPS_FLOWCONTROLLER
WriterAttributes writer_attr;
writer_attr.throughputController.bytesPerPeriod = 300000; //300kb
writer_attr.throughputController.periodMillisecs = 1000; //1000ms

//CONF-QOS-PUBLISHMODE
// Allows fragmentation.
publisher_attr.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
//!--

{
//CONF_QOS_RTPS_PUBLISHMODE
WriterAttributes write_attr;
write_attr.mode = ASYNCHRONOUS_WRITER;    // Allows fragmentation
//!--
}

//CONF-QOS-DISABLE-DISCOVERY
participant_attr.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = false;
//!--

//CONF-QOS-INCREASE-SOCKETBUFFERS
participant_attr.rtps.sendSocketBufferSize = 1048576;
participant_attr.rtps.listenSocketBufferSize = 4194304;
//!--

//CONF-TRANSPORT-DESCRIPTORS
UDPv4TransportDescriptor descriptor;
descriptor.interfaceWhiteList.emplace_back("127.0.0.1");
//!--

//LOG_USAGE_PRINT
logInfo(INFO_MSG, "This is an info message");
logWarning(WARN_MSG, "This is a warning message");
logError(ERROR_MSG, "This is an error message");
//!--

//LOG_USAGE_INFO
logInfo(NEW_CATEGORY, "This log message belong to NEW_CATEGORY category.");
//!--

//LOG_USAGE_VERBOSITY
Log::SetVerbosity(Log::Kind::Warning);
std::regex my_regex("NEW_CATEGORY");
Log::SetCategoryFilter(my_regex);
//!--

/*
//LOG_USAGE_API
//! Enables the reporting of filenames in log entries. Disabled by default.
RTPS_DllAPI static void ReportFilenames(bool);
//! Enables the reporting of function names in log entries. Enabled by default when supported.
RTPS_DllAPI static void ReportFunctions(bool);
//! Sets the verbosity level, allowing for messages equal or under that priority to be logged.
RTPS_DllAPI static void SetVerbosity(Log::Kind);
//! Returns the current verbosity level.
RTPS_DllAPI static Log::Kind GetVerbosity();
//! Sets a filter that will pattern-match against log categories, dropping any unmatched categories.
RTPS_DllAPI static void SetCategoryFilter    (const std::regex&);
//! Sets a filter that will pattern-match against filenames, dropping any unmatched categories.
RTPS_DllAPI static void SetFilenameFilter    (const std::regex&);
//! Sets a filter that will pattern-match against the provided error string, dropping any unmatched categories.
RTPS_DllAPI static void SetErrorStringFilter (const std::regex&);
//!--
*/

//LOG-CONFIG
Log::ClearConsumers(); // Deactivate StdoutConsumer

// Add FileConsumer consumer
std::unique_ptr<FileConsumer> fileConsumer(new FileConsumer("append.log", true));
Log::RegisterConsumer(std::move(fileConsumer));

// Back to its defaults: StdoutConsumer will be enable and FileConsumer removed.
Log::Reset();
//!--

//CONF_QOS_STATIC_DISCOVERY_CODE
participant_attr.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = false;
participant_attr.rtps.builtin.use_STATIC_EndpointDiscoveryProtocol = true;
//!--

//CONF_QOS_STATIC_DISCOVERY_XML
participant_attr.rtps.builtin.setStaticEndpointXMLFilename("ParticipantWithASubscriber.xml");
//!--

//CONF_QOS_TUNING_RELIABLE_PUBLISHER
publisher_attr.times.heartbeatPeriod.seconds = 0;
publisher_attr.times.heartbeatPeriod.fraction = 4294967 * 500; //500 ms
//!--

//CONF_QOS_TUNING_RELIABLE_WRITER
writer_attr.times.heartbeatPeriod.seconds = 0;
writer_attr.times.heartbeatPeriod.fraction = 4294967 * 500; //500 ms
//!--

}

//API-DISCOVERY-TOPICS-LISTENER
class CustomParticipantListener : public eprosima::fastrtps::ParticipantListener
{
    /* Custom Listener onSubscriberDiscovery */
    void onSubscriberDiscovery(
            eprosima::fastrtps::Participant * participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo && info) override
    {
        (void)participant;
        switch(info.status) {
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER:
                /* Process the case when a new subscriber was found in the domain */
                cout << "New subscriber for topic '" << info.info.topicName() << "' of type '" << info.info.typeName() << "' discovered";
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER:
                /* Process the case when a subscriber changed its QOS */
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER:
                /* Process the case when a subscriber was removed from the domain */
                cout << "Subscriber for topic '" << info.info.topicName() << "' of type '" << info.info.typeName() << "' left the domain.";
                break;
        }
    }

    /* Custom Listener onPublisherDiscovery */
    void onPublisherDiscovery(
            eprosima::fastrtps::Participant * participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo  && info) override
    {
        (void)participant;
        switch(info.status) {
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::DISCOVERED_WRITER:
                /* Process the case when a new publisher was found in the domain */
                cout << "New publisher for topic '" << info.info.topicName() << "' of type '" << info.info.typeName() << "' discovered";
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::CHANGED_QOS_WRITER:
                /* Process the case when a publisher changed its QOS */
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::REMOVED_WRITER:
                /* Process the case when a publisher was removed from the domain */
                cout << "publisher for topic '" << info.info.topicName() << "' of type '" << info.info.typeName() << "' left the domain.";
                break;
        }
    }
};
//!--

//RTPS_API_READER_LISTENER
class MyReaderListener: public ReaderListener
{
    public:

        MyReaderListener(){}

        ~MyReaderListener(){}

        void onNewCacheChangeAdded(RTPSReader* reader,const CacheChange_t* const change)
        {
            // The incoming message is enclosed within the `change` in the function parameters
            printf("%s\n",change->serializedPayload.data);
            // Once done, remove the change
            reader->getHistory()->remove_change((CacheChange_t*)change);
        }
};
//!--

void rtps_api_example_create_entities()
{
//RTPS_API_CREATE_PARTICIPANT
RTPSParticipantAttributes participant_attr;
participant_attr.setName("participant");
RTPSParticipant* participant = RTPSDomain::createParticipant(participant_attr);
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
class MyReaderListener : public ReaderListener{};
MyReaderListener listener;
HistoryAttributes history_attr;
ReaderHistory* history = new ReaderHistory(history_attr);
ReaderAttributes reader_attr;
RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, history, &listener);
//!--
}

//RTPS_API_WRITE_SAMPLE
//Request a change from the history
CacheChange_t* change = writer->new_change([]() -> uint32_t { return 255;}, ALIVE);
//Write serialized data into the change
change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2)+1;
//Insert change back into the history. The Writer takes care of the rest.
history->add_change(change);
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
history_attr.payloadMaxSize  = 250; //Defaults to 500 bytes
//!--

//RTPS_API_HISTORY_CONF_RESOURCES
history_attr.initialReservedCaches = 250; //Defaults to 500
history_attr.maximumReservedCaches = 500; //Defaults to 0 = Unlimited Changes
//!--
}

void pubsub_api_example_create_entities()
{
//PUBSUB_API_CREATE_PARTICIPANT
ParticipantAttributes participant_attr; //Configuration structure
Participant *participant = Domain::createParticipant(participant_attr);
//!--

//PUBSUB_API_REGISTER_TYPE
HelloWorldPubSubType m_type; //Auto-generated type from FastRTPSGen
Domain::registerType(participant, &m_type);
//!--

//PUBSUB_API_CREATE_PUBLISHER
PublisherAttributes publisher_attr; //Configuration structure
PubListener publisher_listener; //Class that implements callbacks from the publisher
Publisher *publisher = Domain::createPublisher(participant, publisher_attr, &publisher_listener);
//!--

//PUBSUB_API_WRITE_SAMPLE
HelloWorld sample; //Auto-generated container class for topic data from FastRTPSGen
sample.msg("Hello there!"); // Add contents to the message
publisher->write(&sample); //Publish
//!--

//PUBSUB_API_CREATE_SUBSCRIBER
SubscriberAttributes subscriber_attr; //Configuration structure
SubListener subscriber_listener; //Class that implements callbacks from the Subscriber
Subscriber *subscriber = Domain::createSubscriber(participant, subscriber_attr, &subscriber_listener);
//!--
}

void pubsub_api_example_participant_configuration()
{
//PUBSUB_API_CONF_PARTICIPANT
ParticipantAttributes participant_attr;

participant_attr.rtps.setName("my_participant");
participant_attr.rtps.builtin.domainId = 80;

Participant *participant = Domain::createParticipant(participant_attr);
//!--

{
//PUBSUB_API_CONF_PARTICIPANT_XML
Participant *participant = Domain::createParticipant("participant_xml_profile");
//!--
}

//PUBSUB_API_CONF_PARTICIPANT_NAME
participant_attr.rtps.setName("my_participant");
//!--

//PUBSUB_API_CONF_PARTICIPANT_DOMAIN
participant_attr.rtps.builtin.domainId = 80;
//!--

//PUBSUB_API_CONF_CREATE_PUBSUB
PublisherAttributes publisher_attr;
Publisher *publisher = Domain::createPublisher(participant, publisher_attr);

SubscriberAttributes subscriber_attr;
Subscriber *subscriber = Domain::createSubscriber(participant, subscriber_attr);
//!--

{
//PUBSUB_API_CONF_CREATE_PUBSUB_XML
Publisher *publisher = Domain::createPublisher(participant, "publisher_xml_profile");
Subscriber *subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");
//!--
}

//PUBSUB_API_CONF_PUBSUB_TOPIC
publisher_attr.topic.topicDataType = "HelloWorldType";
publisher_attr.topic.topicName = "HelloWorldTopic";

subscriber_attr.topic.topicDataType = "HelloWorldType";
subscriber_attr.topic.topicName = "HelloWorldTopic";
//!--

//PUBSUB_API_CONF_PUBSUB_RELIABILITY
publisher_attr.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;

subscriber_attr.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
//!--

//PUBSUB_API_CONF_PUBSUB_HISTORY
publisher_attr.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;

subscriber_attr.topic.historyQos.kind =   KEEP_LAST_HISTORY_QOS;
subscriber_attr.topic.historyQos.depth = 5;
//!--

//PUBSUB_API_CONF_PUBSUB_DURABILITY
publisher_attr.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;

subscriber_attr.qos.m_durability.kind = VOLATILE_DURABILITY_QOS;
//!--

//PUBSUB_API_CONF_PUBSUB_RESOURCE_LIMITS
publisher_attr.topic.resourceLimitsQos.max_samples = 200;

subscriber_attr.topic.resourceLimitsQos.max_samples = 200;
//!--

//PUBSUB_API_CONF_PUBSUB_UNICAST_LOCATORS
Locator_t new_locator;
new_locator.port = 7800;

subscriber_attr.unicastLocatorList.push_back(new_locator);

publisher_attr.unicastLocatorList.push_back(new_locator); 
//!--

{
//PUBSUB_API_CONF_PUBSUB_MULTICAST_LOCATORS
Locator_t new_locator;

IPLocator::setIPv4(new_locator, "239.255.0.4");
new_locator.port = 7900;

subscriber_attr.multicastLocatorList.push_back(new_locator);

publisher_attr.multicastLocatorList.push_back(new_locator);
//!--
}

//PUBSUB_API_CONF_PUBSUB_STATIC_SAMPLEINFO
HelloWorld sample;
SampleInfo_t sample_info;
subscriber->takeNextData((void*)&sample, &sample_info);
//!--

{
DynamicPubSubType* input_type = nullptr;
//PUBSUB_API_CONF_PUBSUB_DYNAMIC_SAMPLEINFO
// input_type is an instance of DynamicPubSubType of out current dynamic type
DynamicPubSubType *pst = dynamic_cast<DynamicPubSubType*>(input_type);
DynamicData *sample = DynamicDataFactory::GetInstance()->CreateData(pst->GetDynamicType());
subscriber->takeNextData(sample, &sample_info);
//!--
}

//PUBSUB_API_CONF_PUBSUB_SAMPLEINFO_USAGE
if( (sample_info.sampleKind == ALIVE) & (sample_info.ownershipStrength > 25) )
{
    //Process data
}
//!--

}

void discovery_topic_api_compilation_check()
{
ParticipantAttributes participant_attr;
//API-DISCOVERY-TOPICS-SET
// Create Custom user ParticipantListener (should inherit from eprosima::fastrtps::ParticipantListener.
CustomParticipantListener *listener = new CustomParticipantListener();
// Pass the listener on participant creation.
Participant* participant = Domain::createParticipant(participant_attr, listener);
//!--
}

void xml_load_and_apply_profiles_check()
{
//XML-LOAD-APPLY-PROFILES
eprosima::fastrtps::Domain::loadXMLProfilesFile("my_profiles.xml");

Participant *participant = Domain::createParticipant("participant_xml_profile");
Publisher *publisher = Domain::createPublisher(participant, "publisher_xml_profile");
Subscriber *subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");
//!--
}

void xml_dyn_examples_check()
{
//XML-DYN-ENUM
DynamicTypeBuilder_ptr enum_builder = DynamicTypeBuilderFactory::GetInstance()->CreateEnumBuilder();
enum_builder->SetName("MyEnum");
enum_builder->AddEmptyMember(0, "A");
enum_builder->AddEmptyMember(1, "B");
enum_builder->AddEmptyMember(2, "C");
DynamicType_ptr enum_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(enum_builder.get());
//!--
//XML-TYPEDEF
DynamicTypeBuilder_ptr alias1_builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(enum_builder.get(), "MyAlias1");
DynamicType_ptr alias1_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(alias1_builder.get());

std::vector<uint32_t> sequence_lengths = { 2, 2 };
DynamicTypeBuilder_ptr int_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(int_builder.get(), sequence_lengths);
DynamicTypeBuilder_ptr alias2_builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(array_builder.get(), "MyAlias2");
DynamicType_ptr alias2_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(alias2_builder.get());
//!--
//XML-STRUCT
DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();
DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();

struct_builder->SetName("MyStruct");
struct_builder->AddMember(0, "first", long_builder.get());
struct_builder->AddMember(1, "second", long_long_builder.get());
DynamicType_ptr struct_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(struct_builder.get());
//!--
{
//XML-UNION
DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();
DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
DynamicTypeBuilder_ptr octet_builder = DynamicTypeBuilderFactory::GetInstance()->CreateByteBuilder();
DynamicTypeBuilder_ptr union_builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(octet_builder.get());

union_builder->SetName("MyUnion");
union_builder->AddMember(0, "first", long_builder.get(), "", { 0, 1 }, false);
union_builder->AddMember(1, "second", struct_builder.get(), "", { 2 }, false);
union_builder->AddMember(2, "third", long_long_builder.get(), "", { }, true);
DynamicType_ptr union_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(union_builder.get());
//!--
}
{
//XML-GENERIC
DynamicTypeBuilder_ptr long_long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt64Builder();
long_long_builder->SetName("my_long");
DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(long_long_builder.get());
//!--
}
{
//XML-BOUNDEDSTRINGS
DynamicTypeBuilder_ptr string_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(41925);
string_builder->SetName("my_large_string");
DynamicType_ptr string_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(string_builder.get());

DynamicTypeBuilder_ptr wstring_builder = DynamicTypeBuilderFactory::GetInstance()->CreateWstringBuilder(20925);
wstring_builder->SetName("my_large_wstring");
DynamicType_ptr wstring_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(wstring_builder.get());
//!--
}
{
//XML-ARRAYS
std::vector<uint32_t> lengths = { 2, 3, 4 };
DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(long_builder.get(), lengths);
array_builder->SetName("long_array");
DynamicType_ptr array_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(array_builder.get());
//!--
}
{
//XML-SEQUENCES
uint32_t child_len = 2;
DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr seq_builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(long_builder.get(),
    child_len);
uint32_t length = 3;
DynamicTypeBuilder_ptr seq_seq_builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(
    seq_builder.get(), length);
seq_seq_builder->SetName("my_sequence_sequence");
DynamicType_ptr seq_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(seq_seq_builder.get());
//!--
}
{
//XML-MAPS
uint32_t length = 2;
DynamicTypeBuilder_ptr long_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicTypeBuilder_ptr map_builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(long_builder.get(),
    long_builder.get(), length);

DynamicTypeBuilder_ptr map_map_builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(long_builder.get(),
    map_builder.get(), length);
map_map_builder->SetName("my_map_map");
DynamicType_ptr map_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(map_map_builder.get());
//!--
}
{
//XML-USAGE
// Load the XML File
XMLP_ret ret = XMLProfileManager::loadXMLFile("types.xml");
// Create the "MyStructPubSubType"
DynamicPubSubType *pbType = XMLProfileManager::CreateDynamicPubSubType("MyStruct");
// Create a "MyStruct" instance
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(pbType->GetDynamicType());
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
part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca", "file://maincacert.pem");
part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate", "file://appcert.pem");
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
part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca", "file://maincacert.pem");
part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate", "file://appcert.pem");
part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key", "file://appkey.pem");

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

void persistence_configuration()
{
//PERSISTENCE_CONF_PARTICIPANT
eprosima::fastrtps::rtps::RTPSParticipantAttributes part_attr;

// Activate Persistence:SQLITE3 plugin
part_attr.properties.properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");

// Configure Persistence:SQLITE3 plugin
part_attr.properties.properties().emplace_back("dds.persistence.sqlite3.filename", "example.db");
//!--

//PERSISTENCE_CONF_WRITER
eprosima::fastrtps::rtps::WriterAttributes writer_attr;

// Set durability to TRANSIENT
writer_attr.endpoint.durabilityKind = TRANSIENT;

// Set persistence_guid
writer_attr.endpoint.persistence_guid.guidPrefix.value[11] = 1;
writer_attr.endpoint.persistence_guid.entityId = 0x12345678;
//!--

//PERSISTENCE_CONF_READER
eprosima::fastrtps::rtps::ReaderAttributes reader_attr;

// Set durability to TRANSIENT
reader_attr.endpoint.durabilityKind = TRANSIENT;

// Set persistence_guid
reader_attr.endpoint.persistence_guid.guidPrefix.value[11] = 1;
reader_attr.endpoint.persistence_guid.entityId = 0x3456789A;
//!--
}

void dynamictypes_configuration()
{
//DYNAMIC_TYPES_CREATE_PRIMITIVES
// Using Builders
DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Builder();
DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(created_type);
data->SetInt32Value(1);

// Creating directly the Dynamic Type
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
data2->SetInt32Value(1);
//!--

{
//DYNAMIC_TYPES_CREATE_STRINGS
// Using Builders
DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(created_type);
data->SetStringValue("Dynamic String");

// Creating directly the Dynamic Type
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
data2->SetStringValue("Dynamic String");
//!--
}

{
//DYNAMIC_TYPES_CREATE_ALIAS
// Using Builders
DynamicTypeBuilder_ptr base_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(base_builder.get());
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(created_type.get(), "alias");
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(builder.get());
data->SetStringValue("Dynamic Alias String");

// Creating directly the Dynamic Type
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pType, "alias");
DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pAliasType);
data2->SetStringValue("Dynamic Alias String");
//!--
}

{
//DYNAMIC_TYPES_CREATE_ENUMERATIONS
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateEnumBuilder();
builder->AddEmptyMember(0, "DEFAULT");
builder->AddEmptyMember(1, "FIRST");
builder->AddEmptyMember(2, "SECOND");
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(pType);

std::string sValue = "SECOND";
data->SetEnumValue(sValue);
uint32_t uValue = 2;
data->SetEnumValue(uValue);
//!--
}

{
//DYNAMIC_TYPES_CREATE_BITSETS
uint32_t limit = 5;

// Using Builders
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateBitsetBuilder(limit);
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(pType);
data->SetBoolValue(true, 2);
bool bValue;
data->GetBoolValue(bValue, 0);

// Creating directly the Dynamic Type
DynamicType_ptr pType2 = DynamicTypeBuilderFactory::GetInstance()->CreateBitsetType(limit);
DynamicData_ptr data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
data2->SetBoolValue(true, 2);
bool bValue2;
data2->GetBoolValue(bValue2, 0);
//!--
}

{
//DYNAMIC_TYPES_CREATE_BITMASKS
uint32_t limit = 5;

// Using Builders
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateBitmaskBuilder(limit);
builder->AddEmptyMember(0, "FIRST");
builder->AddEmptyMember(1, "SECOND");
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateType(builder.get());
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(pType);
data->SetBoolValue(true, 2);
bool bValue;
data->GetBoolValue(bValue, 0);
bValue = data->GetBitmaskValue("FIRST");
//!--
}

{
//DYNAMIC_TYPES_CREATE_STRUCTS
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

DynamicType_ptr struct_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(struct_type);

data->SetInt32Value(5, 0);
data->SetUint64Value(13, 1);
//!--
}

{
//DYNAMIC_TYPES_CREATE_UNIONS
DynamicType_ptr discriminator = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(discriminator.get());

builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type(), "", { 0 }, true);
builder->AddMember(0, "second", DynamicTypeBuilderFactory::GetInstance()->CreateInt64Type(), "", { 1 }, false);
DynamicType_ptr union_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(union_type);

data->SetInt32Value(9, 0);
data->SetInt64Value(13, 1);
uint64_t unionLabel;
data->GetUnionLabel(unionLabel);
//!--
}

{
//DYNAMIC_TYPES_CREATE_SEQUENCES
uint32_t length = 2;

DynamicType_ptr base_type = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateSequenceBuilder(base_type.get(), length);
DynamicType_ptr sequence_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(sequence_type);

MemberId newId, newId2;
data->InsertInt32Value(10, newId);
data->InsertInt32Value(12, newId2);
data->RemoveSequenceData(newId);
//!--
}

{
//DYNAMIC_TYPES_CREATE_ARRAYS
std::vector<uint32_t> lengths = { 2, 2 };

DynamicType_ptr base_type = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateArrayBuilder(base_type.get(), lengths);
DynamicType_ptr array_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(array_type);

MemberId pos = data->GetArrayIndex({1, 0});
data->SetInt32Value(11, pos);
data->SetInt32Value(27, pos + 1);
data->ClearArrayData(pos);
//!--
}

{
//DYNAMIC_TYPES_CREATE_MAPS
uint32_t length = 2;

DynamicType_ptr base = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateMapBuilder(base.get(), base.get(), length);
DynamicType_ptr map_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(map_type);

DynamicData_ptr key = DynamicDataFactory::GetInstance()->CreateData(base);
MemberId keyId;
MemberId valueId;
data->InsertMapData(key.get(), keyId, valueId);
MemberId keyId2;
MemberId valueId2;
key->SetInt32Value(2);
data->InsertMapData(key.get(), keyId2, valueId2);

data->SetInt32Value(53, valueId2);

data->RemoveMapData(keyId);
data->RemoveMapData(keyId2);
//!--
}

{
//DYNAMIC_TYPES_CREATE_NESTED_STRUCTS
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());
DynamicType_ptr struct_type = builder->Build();

DynamicTypeBuilder_ptr parent_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
parent_builder->AddMember(0, "child_struct", struct_type);
parent_builder->AddMember(1, "second", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(parent_builder.get());

DynamicData* child_data = data->LoanValue(0);
child_data->SetInt32Value(5, 0);
child_data->SetUint64Value(13, 1);
data->ReturnLoanedValue(child_data);
//!--
}

{
//DYNAMIC_TYPES_CREATE_INHERITANCE_STRUCTS
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

DynamicTypeBuilder_ptr child_builder = DynamicTypeBuilderFactory::GetInstance()->CreateChildStructBuilder(builder.get());
builder->AddMember(2, "third", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());

DynamicType_ptr struct_type = child_builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(struct_type);

data->SetInt32Value(5, 0);
data->SetUint64Value(13, 1);
data->SetUint64Value(47, 2);
//!--
}

{
//DYNAMIC_TYPES_CREATE_NESTED_ALIAS
// Using Builders
DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder(100);
DynamicType_ptr created_type = DynamicTypeBuilderFactory::GetInstance()->CreateType(created_builder.get());
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(created_builder.get(), "alias");
DynamicTypeBuilder_ptr builder2 = DynamicTypeBuilderFactory::GetInstance()->CreateAliasBuilder(builder.get(), "alias2");
DynamicData* data = DynamicDataFactory::GetInstance()->CreateData(builder2.get());
data->SetStringValue("Dynamic Alias 2 String");

// Creating directly the Dynamic Type
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateStringType(100);
DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pType, "alias");
DynamicType_ptr pAliasType2 = DynamicTypeBuilderFactory::GetInstance()->CreateAliasType(pAliasType, "alias2");
DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pAliasType);
data2->SetStringValue("Dynamic Alias 2 String");
//!--
}

{
//DYNAMIC_TYPES_CREATE_NESTED_UNIONS
DynamicType_ptr discriminator = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::GetInstance()->CreateUnionBuilder(discriminator.get());
builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type(), "", { 0 }, true);

DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();
struct_builder->AddMember(0, "first", DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type());
struct_builder->AddMember(1, "other", DynamicTypeBuilderFactory::GetInstance()->CreateUint64Type());
builder->AddMember(1, "first", struct_builder.get(), "", { 1 }, false);

DynamicType_ptr union_type = builder->Build();
DynamicData_ptr data = DynamicDataFactory::GetInstance()->CreateData(union_type);

DynamicData* child_data = data->LoanValue(1);
child_data->SetInt32Value(9, 0);
child_data->SetInt64Value(13, 1);
data->ReturnLoanedValue(child_data);
//!--
}

{
//DYNAMIC_TYPES_SERIALIZATION
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicPubSubType pubsubType(pType);

// SERIALIZATION EXAMPLE
DynamicData* pData = DynamicDataFactory::GetInstance()->CreateData(pType);
uint32_t payloadSize = static_cast<uint32_t>(pubsubType.getSerializedSizeProvider(data)());
SerializedPayload_t payload(payloadSize);
pubsubType.serialize(data, &payload);

// DESERIALIZATION EXAMPLE
types::DynamicData* data2 = DynamicDataFactory::GetInstance()->CreateData(pType);
pubsubType.deserialize(&payload, data2);
//!--
}

{
//DYNAMIC_TYPES_NOTES_1
DynamicTypeBuilder* pBuilder = DynamicTypeBuilderFactory::GetInstance()->CreateUint32Builder();
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicData* pData = DynamicDataFactory::GetInstance()->CreateData(pType);

DynamicTypeBuilderFactory::GetInstance()->DeleteBuilder(pBuilder);
DynamicDataFactory::GetInstance()->DeleteData(pData);
//!--
}

{
//DYNAMIC_TYPES_NOTES_2
DynamicTypeBuilder_ptr pBuilder = DynamicTypeBuilderFactory::GetInstance()->CreateUint32Builder();
DynamicType_ptr pType = DynamicTypeBuilderFactory::GetInstance()->CreateInt32Type();
DynamicData_ptr pData = DynamicDataFactory::GetInstance()->CreateData(pType);
//!--
}

}

int main(int argc, const char** argv)
{
    if(argc != 2)
    {
        printf("Bad number of parameters\n");
        exit(-1);
    }

    if(strncmp(argv[1], "Governance", 10) == 0)
    {
        std::ifstream file;
        file.open(argv[1]);
        std::string content((std::istreambuf_iterator<char>(file)),
                (std::istreambuf_iterator<char>()));

        GovernanceParser parser;

        if(!parser.parse_stream(content.c_str(), content.length()))
        {
            printf("Error parsing xml file %s\n", argv[1]);
            exit(-1);
        }
    }
    else if(strncmp(argv[1], "Permissions", 11) == 0)
    {
        std::ifstream file;
        file.open(argv[1]);
        std::string content((std::istreambuf_iterator<char>(file)),
                (std::istreambuf_iterator<char>()));

        PermissionsParser parser;

        if(!parser.parse_stream(content.c_str(), content.length()))
        {
            printf("Error parsing xml file %s\n", argv[1]);
            exit(-1);
        }
    }
    else if(strncmp(argv[1], "Static", 6) == 0)
    {
        XMLEndpointParser parser;
        std::string file = argv[1];

        if(parser.loadXMLFile(file) != XMLP_ret::XML_OK)
        {
            printf("Error parsing xml file %s\n", argv[1]);
            exit(-1);
        }
    }
    else
    {
        XMLProfileManager parser;

        if(parser.loadXMLFile(argv[1]) != XMLP_ret::XML_OK)
        {
            printf("Error parsing xml file %s\n", argv[1]);
            exit(-1);
        }
    }

    exit(0);
}
