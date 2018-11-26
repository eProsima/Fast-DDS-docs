#include <fastrtps/Domain.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>

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
