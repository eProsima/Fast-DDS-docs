#include <fastrtps/Domain.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/utils/IPLocator.h>

using namespace eprosima::fastrtps;
using namespace ::rtps;
using namespace ::xmlparser;

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

//CONF-QOS-PUBLISHMODE
// Allows fragmentation.
publisher_attr.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
//!--

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

int main(int argc, const char** argv)
{
    if(argc != 2)
    {
        printf("Bad number of parameters\n");
        exit(-1);
    }

    XMLProfileManager parser;

    if(parser.loadXMLFile(argv[1]) != XMLP_ret::XML_OK)
    {
        printf("Error parsing xml file %s\n", argv[1]);
        exit(-1);
    }

    exit(0);
}
