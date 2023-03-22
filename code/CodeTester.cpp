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
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/log/Log.h>
#include <fastrtps/log/FileConsumer.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/TopicDataType.h>

#include <fstream>

using namespace eprosima::fastrtps;
using namespace ::rtps;
using namespace ::security;
using namespace ::types;
using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;

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

//PUBSUB_API_PUBLISHER_LISTENER
class PubListener : public PublisherListener
{
public:

    PubListener()
    {
    }

    ~PubListener()
    {
    }

    void onPublicationmatched(
            Publisher* pub,
            MatchingInfo& info)
    {
        //Callback implementation. This is called each time the Publisher finds a Subscriber on the network that listens to the same topic.
    }

};
//!--

//PUBSUB_API_SUBSCRIBER_LISTENER
class SubListener : public SubscriberListener
{
public:

    SubListener()
    {
    }

    ~SubListener()
    {
    }

    void onNewDataMessage(
            Subscriber* sub)
    {
        if (sub->takeNextData((void*)&sample, &sample_info))
        {
            if (sample_info.sampleKind == ALIVE)
            {
                std::cout << "New message: " << sample.msg() << std::endl;
            }
        }
    }

    HelloWorld sample;     //Storage for incoming messages

    SampleInfo_t sample_info;     //Auxiliary structure with meta-data on the message
};
//!--

void configuration_compilation_check()
{
    ParticipantAttributes participant_attr;
    PublisherAttributes publisher_attr;
    SubscriberAttributes subscriber_attr;

    {
        //CONF-QOS-PARTITIONS
        PublisherAttributes pub_11_attr;
        pub_11_attr.qos.m_partition.push_back("Partition_1");
        pub_11_attr.qos.m_partition.push_back("Partition_2");

        PublisherAttributes pub_12_attr;
        pub_12_attr.qos.m_partition.push_back("*");

        PublisherAttributes pub_21_attr;
        //No partitions defined for pub_21

        PublisherAttributes pub_22_attr;
        pub_22_attr.qos.m_partition.push_back("Partition*");

        SubscriberAttributes subs_31_attr;
        subs_31_attr.qos.m_partition.push_back("Partition_1");

        SubscriberAttributes subs_32_attr;
        subs_32_attr.qos.m_partition.push_back("Partition_2");

        SubscriberAttributes subs_33_attr;
        subs_33_attr.qos.m_partition.push_back("Partition_3");

        SubscriberAttributes subs_34_attr;
        //No partitions defined for subs_34
        //!--
    }

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

    //CONF-UDP-TRANSPORT-SETTING
    //Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 9216;
    udp_transport->receiveBufferSize = 9216;
    udp_transport->non_blocking_send = true;

    //Link the Transport Layer to the Participant.
    participant_attr.rtps.userTransports.push_back(udp_transport);
    //!--

    //CONF-TCP-TRANSPORT-SETTING
    //Create a descriptor for the new transport.
    auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();
    tcp_transport->sendBufferSize = 9216;
    tcp_transport->receiveBufferSize = 9216;
    tcp_transport->add_listener_port(5100);
    tcp_transport->set_WAN_address("80.80.99.45");

    //Link the Transport Layer to the Participant.
    participant_attr.rtps.userTransports.push_back(tcp_transport);
    //!--

    //CONF-SHM-TRANSPORT-SETTING
    // Create a descriptor for the new transport.
    std::shared_ptr<SharedMemTransportDescriptor> shm_transport = std::make_shared<SharedMemTransportDescriptor>();

    // Link the Transport Layer to the Participant.
    participant_attr.rtps.userTransports.push_back(shm_transport);
    //!--

    //CONF-TCP2-TRANSPORT-SETTING
    auto tcp2_transport = std::make_shared<TCPv4TransportDescriptor>();

    //Disable the built-in Transport Layer.
    participant_attr.rtps.useBuiltinTransports = false;

    //Set initial peers.
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
    IPLocator::setIPv4(initial_peer_locator, "80.80.99.45");
    initial_peer_locator.port = 5100;
    participant_attr.rtps.builtin.initialPeersList.push_back(initial_peer_locator);

    //Link the Transport Layer to the Participant.
    participant_attr.rtps.userTransports.push_back(tcp2_transport);
    //!--


    //CONF-ALLOCATION-QOS-EXAMPLE
    // Before creating a participant:
    // We know we have 3 participants on the domain
    participant_attr.rtps.allocation.participants =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
    // We know we have at most 2 readers on each participant
    participant_attr.rtps.allocation.readers =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
    // We know we have at most 1 writer on each participant
    participant_attr.rtps.allocation.writers =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
    // We know the maximum size of partition data
    participant_attr.rtps.allocation.data_limits.max_partitions = 256u;
    // We know the maximum size of user data
    participant_attr.rtps.allocation.data_limits.max_user_data = 256u;
    // We know the maximum size of properties data
    participant_attr.rtps.allocation.data_limits.max_properties = 512u;

    // Before creating the publisher for topic 1:
    // we know we will only have three matching subscribers
    publisher_attr.matched_subscriber_allocation =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(3u);

    // Before creating the publisher for topic 2:
    // we know we will only have two matching subscribers
    publisher_attr.matched_subscriber_allocation =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(2u);

    // Before creating a subscriber:
    // we know we will only have one matching publisher
    subscriber_attr.matched_publisher_allocation =
            eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
    //!--

    //TYPELOOKUP_SERVICE_ENABLING
    participant_attr.rtps.builtin.typelookup_config.use_client = true;
    participant_attr.rtps.builtin.typelookup_config.use_server = true;
    //!--

    {
        //CONF-TCP-TLS-SERVER
        auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();

        using TLSOptions = TCPTransportDescriptor::TLSConfig::TLSOptions;
        tls_transport->apply_security = true;
        tls_transport->tls_config.password = "test";
        tls_transport->tls_config.cert_chain_file = "server.pem";
        tls_transport->tls_config.private_key_file = "serverkey.pem";
        tls_transport->tls_config.tmp_dh_file = "dh2048.pem";
        tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
        tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
        tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);
        //!--
    }

    {
        //CONF-TCP-TLS-CLIENT
        auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();

        using TLSOptions = TCPTransportDescriptor::TLSConfig::TLSOptions;
        using TLSVerifyMode = TCPTransportDescriptor::TLSConfig::TLSVerifyMode;
        tls_transport->apply_security = true;
        tls_transport->tls_config.verify_file = "ca.pem";
        tls_transport->tls_config.verify_mode = TLSVerifyMode::VERIFY_PEER;
        tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
        tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
        tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);
        //!--
    }

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
        IPLocator::setIPv4(locator, 239, 255, 0, 1);
        locator.port = 22222;

        participant_attr.rtps.builtin.metatrafficMulticastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF-METAUNICASTLOCATOR
        // This locator will open a socket to listen network messages on UDPv4 port 22223 over network interface 192.168.0.1
        eprosima::fastrtps::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 0, 1);
        locator.port = 22223;

        participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF-USERMULTICASTLOCATOR
        // This locator will open a socket to listen network messages on UDPv4 port 22224 over multicast address 239.255.0.1
        eprosima::fastrtps::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 239, 255, 0, 1);
        locator.port = 22224;

        participant_attr.rtps.defaultMulticastLocatorList.push_back(locator);
        //!--
    }

    {
        //CONF-USERUNICASTLOCATOR
        // This locator will open a socket to listen network messages on UDPv4 port 22225 over network interface 192.168.0.1
        eprosima::fastrtps::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 0, 1);
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

    //CONF-NON-BLOCKING-WRITE
    //Create a descriptor for the new transport.
    auto non_blocking_UDP_transport = std::make_shared<UDPv4TransportDescriptor>();
    non_blocking_UDP_transport->non_blocking_send = false;

    //Disable the built-in Transport Layer.
    participant_attr.rtps.useBuiltinTransports = false;

    //Link the Transport Layer to the Participant.
    participant_attr.rtps.userTransports.push_back(non_blocking_UDP_transport);
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
        write_attr.mode = ASYNCHRONOUS_WRITER; // Allows fragmentation
        //!--
    }

    //CONF-QOS-DISABLE-DISCOVERY
    participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::NONE;
    //!--

    //CONF-QOS-DISCOVERY-EDP-ATTRIBUTES
    participant_attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
    participant_attr.rtps.builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
    participant_attr.rtps.builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = false;
    //!--

    //CONF-QOS-DISCOVERY-SERVERLIST
    Locator_t server_address(LOCATOR_KIND_UDPv4, 5574);
    IPLocator::setIPv4(server_address, 192, 168, 2, 65);

    RemoteServerAttributes ratt;
    ratt.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");
    ratt.metatrafficUnicastLocatorList.push_back(server_address);

    participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::CLIENT;
    participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(ratt);
    //!--

    //CONF-QOS-DISCOVERY-LEASEDURATION
    participant_attr.rtps.builtin.discovery_config.leaseDuration = Duration_t(5, 0);
    participant_attr.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(2, 0);
    //!--

    //CONF-QOS-INCREASE-SOCKETBUFFERS
    // Increase the sending buffer size
    participant_attr.rtps.sendSocketBufferSize = 1048576;

    // Increase the receiving buffer size
    participant_attr.rtps.listenSocketBufferSize = 4194304;
    //!--

    //CONF-TRANSPORT-DESCRIPTORS
    UDPv4TransportDescriptor descriptor;
    descriptor.interfaceWhiteList.emplace_back("127.0.0.1");
    //!--

    //CONF_QOS_STATIC_DISCOVERY_CODE
    participant_attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
    participant_attr.rtps.builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
    //!--

    //CONF_QOS_STATIC_DISCOVERY_XML
    participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://RemotePublisher.xml");
    participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://RemoteSubscriber.xml");
    //!--

    {
        //CONF_QOS_STATIC_DISCOVERY_USERID
        SubscriberAttributes sub_attr;
        sub_attr.setUserDefinedID(3);

        PublisherAttributes pub_attr;
        pub_attr.setUserDefinedID(5);
        //!--
    }

    //CONF_QOS_TUNING_RELIABLE_PUBLISHER
    publisher_attr.times.heartbeatPeriod.seconds = 0;
    publisher_attr.times.heartbeatPeriod.nanosec = 500000000; //500 ms
    //!--

    //CONF_QOS_TUNING_RELIABLE_WRITER
    writer_attr.times.heartbeatPeriod.seconds = 0;
    writer_attr.times.heartbeatPeriod.nanosec = 500000000; //500 ms
    //!--

    //CONF_INITIAL_PEERS_BASIC
    Locator_t initial_peers_locator;
    IPLocator::setIPv4(initial_peers_locator, "192.168.10.13");
    initial_peers_locator.port = 7412;
    participant_attr.rtps.builtin.initialPeersList.push_back(initial_peers_locator);
    //!--

    //CONF_INITIAL_PEERS_METAUNICAST
    Locator_t meta_unicast_locator;
    IPLocator::setIPv4(meta_unicast_locator, "192.168.10.13");
    meta_unicast_locator.port = 7412;
    participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(meta_unicast_locator);
    //!--

    //CONF_DS_MAIN_SCENARIO_SERVER
    Locator_t server_locator;
    IPLocator::setIPv4(server_locator, "192.168.10.57");
    server_locator.port = 56542;

    participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
    participant_attr.rtps.ReadguidPrefix("72.61.73.70.66.61.72.6d.74.65.73.74");
    participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(server_locator);
    participant_attr.rtps.builtin.discovery_config.discoveryServer_client_syncperiod = Duration_t(0, 250000000);
    //!--

    //CONF_DS_MAIN_SCENARIO_CLIENT
    Locator_t remote_server_locator;
    IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
    remote_server_locator.port = 56542;

    RemoteServerAttributes remote_server_attr;
    remote_server_attr.ReadguidPrefix("72.61.73.70.66.61.72.6d.74.65.73.74");
    remote_server_attr.metatrafficUnicastLocatorList.push_back(remote_server_locator);

    participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::CLIENT;
    participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
    //!--

    {
        //CONF_DS_REDUNDANCY_SCENARIO_SERVER
        Locator_t server_locator_1, server_locator_2;

        IPLocator::setIPv4(server_locator_1, "192.168.10.57");
        server_locator_1.port = 56542;
        IPLocator::setIPv4(server_locator_2, "192.168.10.60");
        server_locator_2.port = 56543;

        ParticipantAttributes participant_attr_1, participant_attr_2;

        participant_attr_1.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        participant_attr_1.rtps.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        participant_attr_1.rtps.builtin.metatrafficUnicastLocatorList.push_back(server_locator_1);

        participant_attr_2.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        participant_attr_2.rtps.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        participant_attr_2.rtps.builtin.metatrafficUnicastLocatorList.push_back(server_locator_2);
        //!--
    }

    {
        //CONF_DS_REDUNDANCY_SCENARIO_CLIENT
        Locator_t remote_server_locator_1, remote_server_locator_2;

        IPLocator::setIPv4(remote_server_locator_1, "192.168.10.57");
        remote_server_locator.port = 56542;
        IPLocator::setIPv4(remote_server_locator_2, "192.168.10.60");
        server_locator.port = 56543;

        RemoteServerAttributes remote_server_attr_1, remote_server_attr_2;

        remote_server_attr_1.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        remote_server_attr_1.metatrafficUnicastLocatorList.push_back(remote_server_locator_1);
        remote_server_attr_2.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr_2.metatrafficUnicastLocatorList.push_back(remote_server_locator_2);

        participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::CLIENT;
        participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_1);
        participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_2);
        //!--
    }

    {
        //CONF_DS_PARTITION_2
        Locator_t server_locator, remote_server_locator;

        IPLocator::setIPv4(server_locator, "192.168.10.60");
        server_locator.port = 56543;
        IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
        remote_server_locator.port = 56542;

        RemoteServerAttributes remote_server_attr;
        remote_server_attr.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr.metatrafficUnicastLocatorList.push_back(remote_server_locator);

        participant_attr.rtps.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
        //!--
    }

    {
        //CONF_DS_PARTITION_3
        Locator_t server_locator, remote_server_locator_A, remote_server_locator_B;

        IPLocator::setIPv4(server_locator, "192.168.10.54");
        server_locator.port = 56541;
        IPLocator::setIPv4(remote_server_locator_A, "192.168.10.60");
        remote_server_locator_A.port = 56543;
        IPLocator::setIPv4(remote_server_locator_B, "192.168.10.57");
        remote_server_locator_B.port = 56542;

        RemoteServerAttributes remote_server_attr_A, remote_server_attr_B;
        remote_server_attr_A.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.31");
        remote_server_attr_A.metatrafficUnicastLocatorList.push_back(remote_server_locator_A);
        remote_server_attr_B.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.32");
        remote_server_attr_B.metatrafficUnicastLocatorList.push_back(remote_server_locator_B);

        participant_attr.rtps.ReadguidPrefix("75.63.2D.73.76.72.63.6C.6E.74.2D.33");
        participant_attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(server_locator);

        participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SERVER;
        participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_A);
        participant_attr.rtps.builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr_B);
        //!--
    }

    {
        //STATIC_DISCOVERY_USE_CASE_PUB
        // Participant attributes
        participant_attr.rtps.setName("HelloWorldPublisher");
        participant_attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
        participant_attr.rtps.builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
        participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://HelloWorldSubscriber.xml");

        // Publisher attributes
        publisher_attr.topic.topicName = "HelloWorldTopic";
        publisher_attr.topic.topicDataType = "HelloWorld";
        publisher_attr.setUserDefinedID(1);
        publisher_attr.setEntityID(2);
        //!--
    }

    {
        //STATIC_DISCOVERY_USE_CASE_SUB
        // Participant attributes
        participant_attr.rtps.setName("HelloWorldSubscriber");
        participant_attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
        participant_attr.rtps.builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
        participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://HelloWorldPublisher.xml");

        // Subscriber attributes
        subscriber_attr.topic.topicName = "HelloWorldTopic";
        subscriber_attr.topic.topicDataType = "HelloWorld";
        subscriber_attr.setUserDefinedID(3);
        subscriber_attr.setEntityID(4);
        //!--
    }

    //CONF-DISCOVERY-PROTOCOL
    participant_attr.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SIMPLE;
    //!--

    //CONF-DISCOVERY-IGNORE-FLAGS
    participant_attr.rtps.builtin.discovery_config.ignoreParticipantFlags =
            static_cast<ParticipantFilteringFlags_t>(
        ParticipantFilteringFlags_t::FILTER_DIFFERENT_PROCESS |
        ParticipantFilteringFlags_t::FILTER_SAME_PROCESS);
    //!--

    //CONF-DISCOVERY-LEASE-DURATION
    participant_attr.rtps.builtin.discovery_config.leaseDuration = Duration_t(10, 20);
    //!--

    //CONF-DISCOVERY-LEASE-ANNOUNCEMENT
    participant_attr.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(1, 2);
    //!--

    //DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT
    participant_attr.rtps.builtin.discovery_config.initial_announcements.count = 5;
    participant_attr.rtps.builtin.discovery_config.initial_announcements.period = Duration_t(0, 100000000u);
    //!--

    //CONF_STATIC_DISCOVERY_CODE
    participant_attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
    participant_attr.rtps.builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
    //!--

    //CONF_STATIC_DISCOVERY_XML_FILE
    participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://RemotePublisher.xml");
    participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("file://RemoteSubscriber.xml");
    //!--

    //CONF_STATIC_DISCOVERY_XML_DATA
    participant_attr.rtps.builtin.discovery_config.static_edp_xml_config("data://<?xml version=\"1.0\" encoding=\"utf-8\"?>" \
            "<staticdiscovery><participant><name>RTPSParticipant</name></participant></staticdiscovery>");
    //!--

    {
        //CONF_QOS_STATIC_DISCOVERY_USERID
        SubscriberAttributes sub_attr;
        sub_attr.setUserDefinedID(3);

        PublisherAttributes pub_attr;
        pub_attr.setUserDefinedID(5);
        //!--
    }

    //CONF_SERVER_PREFIX_EXAMPLE
    participant_attr.rtps.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");
    //!--

    {
        //CONF_SERVER_DISCOVERY_PROTOCOL
        DiscoverySettings& ds = participant_attr.rtps.builtin.discovery_config;
        ds.discoveryProtocol =  DiscoveryProtocol_t::CLIENT;
        ds.discoveryProtocol =  DiscoveryProtocol_t::SUPER_CLIENT;
        ds.discoveryProtocol =  DiscoveryProtocol_t::SERVER;
        ds.discoveryProtocol =  DiscoveryProtocol_t::BACKUP;
        //!--
    }

    {
        //CONF_SERVER_CLIENT_GUIDPREFIX
        RemoteServerAttributes server;
        server.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");

        DiscoverySettings& ds = participant_attr.rtps.builtin.discovery_config;
        ds.m_DiscoveryServers.push_back(server);
        //!--
    }

    {
        //CONF_SERVER_SERVER_GUIDPREFIX
        participant_attr.rtps.ReadguidPrefix("44.53.00.5f.45.50.52.4f.53.49.4d.41");
        //!--
    }

    {
        //CONF_SERVER_CLIENT_LOCATORS
        eprosima::fastrtps::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 1, 133);
        locator.port = 64863;
        RemoteServerAttributes server;
        server.metatrafficUnicastLocatorList.push_back(locator);

        DiscoverySettings& ds = participant_attr.rtps.builtin.discovery_config;
        ds.m_DiscoveryServers.push_back(server);
        //!--
    }

    {
        //CONF_SERVER_SERVER_LOCATORS
        eprosima::fastrtps::rtps::Locator_t locator;
        IPLocator::setIPv4(locator, 192, 168, 1, 133);
        locator.port = 64863;

        LocatorList_t& ull = participant_attr.rtps.builtin.metatrafficUnicastLocatorList;
        ull.push_back(locator);
        //!--
    }

    {
        //CONF_SERVER_CLIENT_PING
        DiscoverySettings& ds = participant_attr.rtps.builtin.discovery_config;
        ds.discoveryServer_client_syncperiod = Duration_t(0, 250000000);
        //!--
    }
}

//API-DISCOVERY-TOPICS-LISTENER
class CustomParticipantListener : public eprosima::fastrtps::ParticipantListener
{
    /* Custom Listener onSubscriberDiscovery */
    void onSubscriberDiscovery(
            eprosima::fastrtps::Participant* participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info) override
    {
        (void)participant;
        switch (info.status){
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER:
                /* Process the case when a new subscriber was found in the domain */
                std::cout << "New subscriber for topic '" << info.info.topicName() << "' of type '" <<
                    info.info.typeName() << "' discovered";
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER:
                /* Process the case when a subscriber changed its QOS */
                break;
            case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER:
                /* Process the case when a subscriber was removed from the domain */
                std::cout << "Subscriber for topic '" << info.info.topicName() << "' of type '" <<
                    info.info.typeName() <<
                    "' left the domain.";
                break;
        }
    }

    /* Custom Listener onPublisherDiscovery */
    void onPublisherDiscovery(
            eprosima::fastrtps::Participant* participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info) override
    {
        (void)participant;
        switch (info.status){
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::DISCOVERED_WRITER:
                /* Process the case when a new publisher was found in the domain */
                std::cout << "New publisher for topic '" << info.info.topicName() << "' of type '" <<
                    info.info.typeName() <<
                    "' discovered";
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::CHANGED_QOS_WRITER:
                /* Process the case when a publisher changed its QOS */
                break;
            case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::REMOVED_WRITER:
                /* Process the case when a publisher was removed from the domain */
                std::cout << "publisher for topic '" << info.info.topicName() << "' of type '" <<
                    info.info.typeName() <<
                    "' left the domain.";
                break;
        }
    }

};
//!--

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
                IPayloadPool*& /* data_owner */,
                CacheChange_t& cache_change) override
        {
            // Reserve new memory for the payload buffer
            octet* payload = new octet[data.length];

            // Copy the data
            memcpy(payload, data.data, data.length);

            // Assign the payload buffer to the CacheChange and update sizes
            cache_change.serializedPayload.data = payload;
            cache_change.serializedPayload.length = data.length;
            cache_change.serializedPayload.max_size = data.length;

            // Tell the CacheChange who needs to release its payload
            cache_change.payload_owner(this);

            return true;
        }

        bool release_payload(
                CacheChange_t& cache_change) override
        {
            // Ensure precondition
            assert(this == cache_change.payload_owner());

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

void pubsub_api_example_create_entities()
{
    //PUBSUB_API_CREATE_PARTICIPANT
    ParticipantAttributes participant_attr; //Configuration structure
    Participant* participant = Domain::createParticipant(participant_attr);
    //!--

    //PUBSUB_API_REGISTER_TYPE
    HelloWorldPubSubType m_type; //Auto-generated type from Fast DDS-Gen
    Domain::registerType(participant, &m_type);
    //!--

    //PUBSUB_API_CREATE_PUBLISHER
    PublisherAttributes publisher_attr; //Configuration structure
    PubListener publisher_listener; //Class that implements callbacks from the publisher
    Publisher* publisher = Domain::createPublisher(participant, publisher_attr, &publisher_listener);
    //!--

    //PUBSUB_API_WRITE_SAMPLE
    HelloWorld sample; //Auto-generated container class for topic data from Fast DDS-Gen
    sample.msg("Hello there!"); // Add contents to the message
    publisher->write(&sample); //Publish
    //!--

    //PUBSUB_API_CREATE_SUBSCRIBER
    SubscriberAttributes subscriber_attr; //Configuration structure
    SubListener subscriber_listener; //Class that implements callbacks from the Subscriber
    Subscriber* subscriber = Domain::createSubscriber(participant, subscriber_attr, &subscriber_listener);
    //!--
}

void pubsub_api_example_participant_configuration()
{
    //PUBSUB_API_CONF_PARTICIPANT
    ParticipantAttributes participant_attr;

    participant_attr.rtps.setName("my_participant");
    participant_attr.domainId = 80;

    Participant* participant = Domain::createParticipant(participant_attr);
    //!--

    {
        //PUBSUB_API_CONF_PARTICIPANT_XML
        Participant* participant = Domain::createParticipant("participant_xml_profile");
        //!--
    }

    //PUBSUB_API_CONF_PARTICIPANT_NAME
    participant_attr.rtps.setName("my_participant");
    //!--

    //PUBSUB_API_CONF_PARTICIPANT_DOMAIN
    participant_attr.domainId = 80;
    //!--

    //PUBSUB_API_CONF_PARTICIPANT_MUTATION_TRIES
    participant_attr.rtps.builtin.mutation_tries = 55;
    //!--

    //PUBSUB_API_CONF_CREATE_PUBSUB
    PublisherAttributes publisher_attr;
    Publisher* publisher = Domain::createPublisher(participant, publisher_attr);

    SubscriberAttributes subscriber_attr;
    Subscriber* subscriber = Domain::createSubscriber(participant, subscriber_attr);
    //!--

    {
        //PUBSUB_API_CONF_CREATE_PUBSUB_XML
        Publisher* publisher = Domain::createPublisher(participant, "publisher_xml_profile");
        Subscriber* subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");
        //!--
    }

    //PUBSUB_API_CONF_PUBSUB_RELIABILITY
    publisher_attr.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;

    subscriber_attr.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    //!--

    //PUBSUB_API_CONF_PUBSUB_DURABILITY
    publisher_attr.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;

    subscriber_attr.qos.m_durability.kind = VOLATILE_DURABILITY_QOS;
    //!--

    //PUBSUB_API_CONF_PUBSUB_DEADLINE
    publisher_attr.qos.m_deadline.period = 1;

    subscriber_attr.qos.m_deadline.period = 1;
    //!--

    //PUBSUB_API_CONF_PUBSUB_LIFESPAN
    publisher_attr.qos.m_lifespan.duration = 1;

    subscriber_attr.qos.m_lifespan.duration = 1;
    //!--

    //PUBSUB_API_CONF_PUBSUB_DISABLE_POSITIVE_ACKS
    publisher_attr.qos.m_disablePositiveACKs.enabled = true;
    publisher_attr.qos.m_disablePositiveACKs.duration = 1;

    subscriber_attr.qos.m_disablePositiveACKs.enabled = true;
    //!--

    //PUBSUB_API_CONF_PUBSUB_LIVELINESS
    publisher_attr.qos.m_liveliness.announcement_period = 0.5;
    publisher_attr.qos.m_liveliness.lease_duration = 1;
    publisher_attr.qos.m_liveliness.kind = AUTOMATIC_LIVELINESS_QOS;

    subscriber_attr.qos.m_liveliness.lease_duration = 1;
    subscriber_attr.qos.m_liveliness.kind = AUTOMATIC_LIVELINESS_QOS;
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
        DynamicPubSubType* pst = dynamic_cast<DynamicPubSubType*>(input_type);
        DynamicData* sample = DynamicDataFactory::get_instance()->create_data(pst->GetDynamicType());
        subscriber->takeNextData(sample, &sample_info);
        //!--
    }

    //PUBSUB_API_CONF_PUBSUB_SAMPLEINFO_USAGE
    if ((sample_info.sampleKind == ALIVE) & (sample_info.ownershipStrength > 25))
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
    CustomParticipantListener* listener = new CustomParticipantListener();
    // Pass the listener on participant creation.
    Participant* participant = Domain::createParticipant(participant_attr, listener);
    //!--
}

void xml_load_and_apply_profiles_check()
{
    //XML-LOAD-APPLY-PROFILES
    eprosima::fastrtps::Domain::loadXMLProfilesFile("my_profiles.xml");

    Participant* participant = Domain::createParticipant("participant_xml_profile");
    Publisher* publisher = Domain::createPublisher(participant, "publisher_xml_profile");
    Subscriber* subscriber = Domain::createSubscriber(participant, "subscriber_xml_profile");
    //!--
}

void xml_dyn_examples_check()
{
    //XML-DYN-ENUM
    DynamicTypeBuilder_ptr enum_builder = DynamicTypeBuilderFactory::get_instance().create_enum_builder();
    enum_builder->set_name("MyEnum");
    enum_builder->add_member(0, "A");
    enum_builder->add_member(1, "B");
    enum_builder->add_member(2, "C");
    DynamicType_ptr enum_type = enum_builder->build();
    //!--
    //XML-TYPEDEF
    DynamicTypeBuilder_ptr alias1_builder = DynamicTypeBuilderFactory::get_instance().create_alias_builder(
        *enum_type, "MyAlias1");
    DynamicType_ptr alias1_type = alias1_builder->build();

    std::vector<uint32_t> sequence_lengths = { 2, 2 };
    DynamicTypeBuilder_cptr int_builder = DynamicTypeBuilderFactory::get_instance().create_int32_builder();
    DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::get_instance().create_array_builder(
        *int_builder->build(), sequence_lengths);
    DynamicTypeBuilder_ptr alias2_builder = DynamicTypeBuilderFactory::get_instance().create_alias_builder(
        *array_builder->build(), "MyAlias2");
    DynamicType_ptr alias2_type = alias2_builder->build();
    //!--
    {
        //XML-STRUCT
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::get_instance().create_int64_type();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();

        struct_builder->set_name("MyStruct");
        struct_builder->add_member(0, "first", long_type);
        struct_builder->add_member(1, "second", long_long_type);
        DynamicType_ptr struct_type = struct_builder->build();
        //!--
    }
    {
        //XML-STRUCT-INHERIT
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::get_instance().create_int64_type();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();

        struct_builder->set_name("ParentStruct");
        struct_builder->add_member(0, "first", long_type);
        struct_builder->add_member(1, "second", long_long_type);
        DynamicType_ptr struct_type = struct_builder->build();

        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance().create_child_struct_builder(*struct_type);

        child_builder->set_name("ChildStruct");
        child_builder->add_member(0, "third", long_type);
        child_builder->add_member(1, "fourth", long_long_type);
        DynamicType_ptr child_struct_type = child_builder->build();
        //!--
    }
    {
        //XML-UNION
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicType_ptr long_long_type = DynamicTypeBuilderFactory::get_instance().create_int64_type();
        DynamicType_ptr octet_type = DynamicTypeBuilderFactory::get_instance().create_byte_type();
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        DynamicTypeBuilder_ptr union_builder = DynamicTypeBuilderFactory::get_instance().create_union_builder(
            *octet_type);

        union_builder->set_name("MyUnion");
        union_builder->add_member(0, "first", long_type, "", std::vector<uint64_t>{ 0, 1 }, false);
        union_builder->add_member(1, "second", struct_builder->build(), "", std::vector<uint64_t>{ 2 }, false);
        union_builder->add_member(2, "third", long_long_type, "", std::vector<uint64_t>{ }, true);
        DynamicType_ptr union_type = union_builder->build();
        //!--
    }
    {
        //XML-GENERIC
        DynamicTypeBuilder_cptr long_long_builder = DynamicTypeBuilderFactory::get_instance().create_int64_builder();
        DynamicTypeBuilder_ptr my_builder = DynamicTypeBuilderFactory::get_instance().create_builder_copy(*long_long_builder);
        my_builder->set_name("my_long");
        DynamicType_ptr long_long_type = my_builder->build();
        //!--
    }
    {
        //XML-BOUNDEDSTRINGS
        DynamicTypeBuilder_cptr string_builder = DynamicTypeBuilderFactory::get_instance().create_string_builder(41925);
        DynamicTypeBuilder_ptr my_builder = DynamicTypeBuilderFactory::get_instance().create_builder_copy(*string_builder);
        my_builder->set_name("my_large_string");
        DynamicType_ptr string_type = my_builder->build();

        DynamicTypeBuilder_cptr wstring_builder = DynamicTypeBuilderFactory::get_instance().create_wstring_builder(20925);
        DynamicTypeBuilder_ptr my_wbuilder = DynamicTypeBuilderFactory::get_instance().create_builder_copy(*wstring_builder);
        my_wbuilder->set_name("my_large_wstring");
        DynamicType_ptr wstring_type = my_wbuilder->build();
        //!--
    }
    {
        //XML-ARRAYS
        std::vector<uint32_t> lengths = { 2, 3, 4 };
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr array_builder = DynamicTypeBuilderFactory::get_instance().create_array_builder(
            *long_type, lengths);
        array_builder->set_name("long_array");
        DynamicType_ptr array_type = array_builder->build();
        //!--
    }
    {
        //XML-SEQUENCES
        uint32_t child_len = 2;
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_cptr seq_builder = DynamicTypeBuilderFactory::get_instance().create_sequence_builder(
            *long_type,
            child_len);
        uint32_t length = 3;
        DynamicTypeBuilder_ptr seq_seq_builder = DynamicTypeBuilderFactory::get_instance().create_sequence_builder(
            *seq_builder->build(), length);
        seq_seq_builder->set_name("my_sequence_sequence");
        DynamicType_ptr seq_type = seq_seq_builder->build();
        //!--
    }
    {
        //XML-MAPS
        uint32_t length = 2;
        DynamicType_ptr long_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr map_builder = DynamicTypeBuilderFactory::get_instance().create_map_builder(
            *long_type,
            *long_type, length);

        DynamicTypeBuilder_ptr map_map_builder = DynamicTypeBuilderFactory::get_instance().create_map_builder(
            *long_type,
            *map_builder->build(), length);
        map_map_builder->set_name("my_map_map");
        DynamicType_ptr map_type = map_map_builder->build();
        //!--
    }
    {
        //XML-BITSET
        DynamicTypeBuilderFactory& factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = factory.create_bitset_builder();
        builder_ptr->add_member(0, "a", factory.create_byte_type());
        builder_ptr->add_member(1, "b", factory.create_bool_type());
        builder_ptr->add_member(3, "c", factory.create_uint16_type());
        builder_ptr->add_member(4, "d", factory.create_int16_type());
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
        DynamicTypeBuilderFactory& factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = factory.create_bitmask_builder(8);
        builder_ptr->add_member(0, "flag0");
        builder_ptr->add_member(1, "flag1");
        builder_ptr->add_member(2, "flag2");
        builder_ptr->add_member(5, "flag5");
        builder_ptr->set_name("MyBitMask");
        //!--
    }
    {
        //XML-BITSET-INHERIT
        DynamicTypeBuilderFactory& factory = DynamicTypeBuilderFactory::get_instance();
        DynamicTypeBuilder_ptr builder_ptr = factory.create_bitset_builder();
        builder_ptr->add_member(0, "a", factory.create_byte_type());
        builder_ptr->add_member(1, "b", factory.create_bool_type());
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_BIT_BOUND_ID, "value", "3");
        builder_ptr->apply_annotation_to_member(0, ANNOTATION_POSITION_ID, "value", "0");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_BIT_BOUND_ID, "value", "1");
        builder_ptr->apply_annotation_to_member(1, ANNOTATION_POSITION_ID, "value", "3");
        builder_ptr->set_name("ParentBitSet");

        DynamicTypeBuilder_ptr child_ptr = factory.create_child_struct_builder(*builder_ptr->build());
        child_ptr->add_member(3, "c", factory.create_uint16_type());
        child_ptr->add_member(4, "d", factory.create_int16_type());
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
    {
        //DYNAMIC_TYPES_QUICK_EXAMPLE
        // Create a builder for a specific type
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_enum_builder();

        // Use the builder to configure the type
        builder->add_member(0, "DEFAULT");
        builder->add_member(1, "FIRST");
        builder->add_member(2, "SECOND");

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
        DynamicTypeBuilder_cptr created_builder = DynamicTypeBuilderFactory::get_instance().create_int32_builder();
        DynamicType_ptr created_type = created_builder->build();
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(created_type);
        data->set_int32_value(1);

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pType);
        data2->set_int32_value(1);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_STRINGS
        // Using Builders
        DynamicTypeBuilder_cptr created_builder = DynamicTypeBuilderFactory::get_instance().create_string_builder(100);
        DynamicType_ptr created_type = created_builder->build();
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(created_type);
        data->set_string_value("Dynamic String");

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance().create_string_type(100);
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pType);
        data2->set_string_value("Dynamic String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ALIAS
        // Create the base type
        DynamicTypeBuilder_cptr base_builder = DynamicTypeBuilderFactory::get_instance().create_string_builder(100);
        DynamicType_ptr base_type = base_builder->build();

        // Create alias using Builders
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_alias_builder(*base_type,
                        "alias");
        DynamicData* data = DynamicDataFactory::get_instance()->create_data(builder->build());
        data->set_string_value("Dynamic Alias String");

        // Create alias type directly
        DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::get_instance().create_alias_type(*base_type, "alias");
        DynamicData* data2 = DynamicDataFactory::get_instance()->create_data(pAliasType);
        data2->set_string_value("Dynamic Alias String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_ENUMERATIONS
        // Add enumeration values using the DynamicTypeBuilder
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_enum_builder();
        builder->add_member(0, "DEFAULT");
        builder->add_member(1, "FIRST");
        builder->add_member(2, "SECOND");

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
        DynamicType_ptr base_type_byte = DynamicTypeBuilderFactory::get_instance().create_byte_type();
        DynamicType_ptr base_type_uint32 = DynamicTypeBuilderFactory::get_instance().create_uint32_type();

        // Create the bitset with two bitfields
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_bitset_builder();
        builder->add_member(0, "byte", base_type_byte);
        builder->add_member(1, "uint32", base_type_uint32);

        // Apply members' annotations
        builder->apply_annotation_to_member(0, ANNOTATION_POSITION_ID, "value", "0");   // "byte" starts at position 0
        builder->apply_annotation_to_member(0, ANNOTATION_BIT_BOUND_ID, "value", "2");  // "byte" is 2 bit length
        builder->apply_annotation_to_member(1, ANNOTATION_POSITION_ID, "value", "10");  // "uint32" starts at position 10 (8 bits empty)
        builder->apply_annotation_to_member(1, ANNOTATION_BIT_BOUND_ID, "value", "20"); // "uint32" is 20 bits length
        DynamicType_ptr bitset_type = builder->build();

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(bitset_type));

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
                DynamicTypeBuilderFactory::get_instance().create_child_struct_builder(*bitset_type);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_BITMASKS
        uint32_t limit = 5; // Stores as "octet"

        // Add bitmask flags using the DynamicTypeBuilder
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_bitmask_builder(limit);
        builder->add_member(0, "FIRST");
        builder->add_member(1, "SECOND");

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(builder->build()));

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
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance().create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance().create_uint64_type());
        DynamicType_ptr struct_type = builder->build();

        // Create the data instance
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(struct_type));

        // Access struct members
        data->set_int32_value(5, 0);
        data->set_uint64_value(13, 1);
        //!--
        //DYNAMIC_TYPES_CREATE_STRUCTS-INHERIT
        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance().create_child_struct_builder(*struct_type);
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_UNIONS
        // Create the union DynamicTypeBuilder with an int32 discriminator
        DynamicType_ptr discriminator = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_union_builder(*discriminator);

        // Add the union members. "firts" will be the default value
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance().create_int32_type(), "",
                std::vector<uint64_t>{ 0 },
                true);
        builder->add_member(0, "second", DynamicTypeBuilderFactory::get_instance().create_int64_type(), "",
                std::vector<uint64_t>{ 1 },
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
        DynamicType_ptr base_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance().create_sequence_builder(*base_type, length);

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
        DynamicType_ptr base_type = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance().create_array_builder(*base_type, lengths);

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
        DynamicType_ptr base = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr builder =
                DynamicTypeBuilderFactory::get_instance().create_map_builder(*base, *base, length);

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
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance().create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance().create_uint64_type());
        DynamicType_ptr struct_type = builder->build();

        // Create a struct type with the previous struct as member
        DynamicTypeBuilder_ptr parent_builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        parent_builder->add_member(0, "child_struct", struct_type);
        parent_builder->add_member(1, "second", DynamicTypeBuilderFactory::get_instance().create_int32_type());
        DynamicType_ptr parent_type = parent_builder->build();
        DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(parent_type));

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
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance().create_int32_type());
        builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance().create_uint64_type());
        DynamicType_ptr base_type = builder->build();

        // Create a struct type derived from the previous struct
        DynamicTypeBuilder_ptr child_builder =
                DynamicTypeBuilderFactory::get_instance().create_child_struct_builder(*base_type);

        // Add new members to the derived type
        builder->add_member(2, "third", DynamicTypeBuilderFactory::get_instance().create_uint64_type());

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
        DynamicTypeBuilder_cptr created_builder = DynamicTypeBuilderFactory::get_instance().create_string_builder(100);
        DynamicType_ptr created_type = created_builder->build();
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_alias_builder(
            *created_type, "alias");
        DynamicType_ptr alias_type = builder->build();
        DynamicTypeBuilder_ptr builder2 = DynamicTypeBuilderFactory::get_instance().create_alias_builder(
            *alias_type, "alias2");
        DynamicData* data(DynamicDataFactory::get_instance()->create_data(builder2->build()));
        data->set_string_value("Dynamic Alias 2 String");

        // Creating directly the Dynamic Type
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance().create_string_type(100);
        DynamicType_ptr pAliasType = DynamicTypeBuilderFactory::get_instance().create_alias_type(*pType, "alias");
        DynamicType_ptr pAliasType2 =
                DynamicTypeBuilderFactory::get_instance().create_alias_type(*pAliasType, "alias2");
        DynamicData* data2(DynamicDataFactory::get_instance()->create_data(pAliasType2));
        data2->set_string_value("Dynamic Alias 2 String");
        //!--
    }

    {
        //DYNAMIC_TYPES_CREATE_NESTED_UNIONS
        // Create a union DynamicTypeBuilder
        DynamicType_ptr discriminator = DynamicTypeBuilderFactory::get_instance().create_int32_type();
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_union_builder(*discriminator);

        // Add a int32 to the union
        builder->add_member(
                0, // id
                "first", // name
                DynamicTypeBuilderFactory::get_instance().create_int32_type(), // type
                "", // default value
                std::vector<uint64_t>{ 0 }, // labels
                true); // default value

        // Create a struct type and add it to the union
        DynamicTypeBuilder_ptr struct_builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        struct_builder->add_member(0, "first", DynamicTypeBuilderFactory::get_instance().create_int32_type());
        struct_builder->add_member(1, "other", DynamicTypeBuilderFactory::get_instance().create_uint64_type());
        builder->add_member(1, "first", struct_builder->build(), "", std::vector<uint64_t>{ 1 }, false);

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
        DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance().create_struct_builder();
        //...
        builder->apply_annotation("MyAnnotation", "value", "5");
        builder->apply_annotation("MyAnnotation", "name", "length");
        //!--
    }

    {
        //DYNAMIC_TYPES_SERIALIZATION
        DynamicType_ptr pType = DynamicTypeBuilderFactory::get_instance().create_int32_type();
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
        DynamicTypeBuilder_ptr pBuilder = DynamicTypeBuilderFactory::get_instance().create_string_builder(16);
        DynamicType_ptr pType = pBuilder->build();
        DynamicData* pData = DynamicDataFactory::get_instance()->create_data(pType);

        DynamicTypeBuilderFactory::get_instance().delete_builder(pBuilder.get());
        DynamicDataFactory::get_instance()->delete_data(pData);
        //!--
    }

    {
        //DYNAMIC_TYPES_NOTES_2
        DynamicTypeBuilder_ptr pBuilder = DynamicTypeBuilderFactory::get_instance().create_string_builder(16);
        DynamicType_ptr pType = pBuilder->build();
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
        DynamicTypeBuilder_ptr struct_type_builder(DynamicTypeBuilderFactory::get_instance().create_struct_builder());

        // Add members to the struct.
        struct_type_builder->add_member(0, "index", DynamicTypeBuilderFactory::get_instance().create_uint32_type());
        struct_type_builder->add_member(1, "message", DynamicTypeBuilderFactory::get_instance().create_string_type());
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
        DynamicTypeBuilder_ptr struct_type_builder(DynamicTypeBuilderFactory::get_instance().create_struct_builder());

        // Add members to the struct.
        struct_type_builder->add_member(0, "index", DynamicTypeBuilderFactory::get_instance().create_uint32_type());
        struct_type_builder->add_member(1, "message", DynamicTypeBuilderFactory::get_instance().create_string_type());
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
