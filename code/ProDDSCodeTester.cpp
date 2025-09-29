#include <cstdint>

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/ethernet/EthernetTransportDescriptor.hpp>
#include <fastdds/rtps/transport/low-bandwidth/PayloadCompressionTransportDescriptor.hpp>
#include <fastdds/rtps/transport/low-bandwidth/HeaderReductionTransportDescriptor.hpp>
#include <fastdds/rtps/transport/udp_tsn/TSN_UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/udp_tsn/UDPPriorityMappings.hpp>

using namespace eprosima::fastdds::dds;

void dds_transport_examples ()
{
    using TSN_UDPv4TransportDescriptor = eprosima::fastdds::rtps::TSN_UDPv4TransportDescriptor;
    using EthernetTransportDescriptor = eprosima::fastdds::rtps::EthernetTransportDescriptor;
    using UDPPriorityMapping = eprosima::fastdds::rtps::UDPPriorityMapping;
    using PayloadCompressionTransportDescriptor = eprosima::fastdds::rtps::PayloadCompressionTransportDescriptor;
    using HeaderReductionTransportDescriptor = eprosima::fastdds::rtps::HeaderReductionTransportDescriptor;

    {
        //TSN_SET_UDP_TUPLE
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<TSN_UDPv4TransportDescriptor>();

        // Set the TSN-UDP tuples
        UDPPriorityMapping udp_priority_mapping_1;
        int32_t udp_priority_1 = -1;
        udp_priority_mapping_1.source_port = 5000;
        udp_transport->udp_priority_mappings[udp_priority_1] = udp_priority_mapping_1;

        UDPPriorityMapping udp_priority_mapping_2;
        int32_t udp_priority_2 = 1;
        udp_priority_mapping_2.source_port = 5000;
        udp_priority_mapping_2.dscp = 1;
        udp_transport->udp_priority_mappings[udp_priority_2] = udp_priority_mapping_2;

        UDPPriorityMapping udp_priority_mapping_3;
        int32_t udp_priority_3 = 2;
        udp_priority_mapping_3.source_port = 5000;
        udp_priority_mapping_3.dscp = 2;
        udp_priority_mapping_3.interface = "eth0";
        udp_transport->udp_priority_mappings[udp_priority_3] = udp_priority_mapping_3;

        UDPPriorityMapping udp_priority_mapping_4;
        int32_t udp_priority_4 = 3;
        udp_priority_mapping_4.source_port = 5000;
        udp_priority_mapping_4.dscp = 3;
        udp_priority_mapping_4.interface = "10.10.1.2";
        udp_transport->udp_priority_mappings[udp_priority_4] = udp_priority_mapping_4;

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(udp_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //TSN_SET_ETHERNET_TUPLE
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto ethernet_transport = std::make_shared<EthernetTransportDescriptor>();

        // Set the ethernet interface name
        ethernet_transport->interface_name = "lo";

        // Set default source port
        ethernet_transport->default_source_port = 6000;

        // Set the TSN-Ethernet tuples
        EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_1;
        int32_t ethernet_priority_1 = -1;
        ethernet_priority_mapping_1.source_port = 5000;
        ethernet_transport->ethernet_priority_mappings[ethernet_priority_1] = ethernet_priority_mapping_1;

        EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_2;
        int32_t ethernet_priority_2 = 1;
        ethernet_priority_mapping_2.source_port = 5000;
        ethernet_priority_mapping_2.pcp = 1;
        ethernet_transport->ethernet_priority_mappings[ethernet_priority_2] = ethernet_priority_mapping_2;

        EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_3;
        int32_t ethernet_priority_3 = 2;
        ethernet_priority_mapping_3.source_port = 5000;
        ethernet_priority_mapping_3.pcp = 2;
        ethernet_priority_mapping_3.vlan_id = 100;
        ethernet_transport->ethernet_priority_mappings[ethernet_priority_3] = ethernet_priority_mapping_3;

        // Link the Transport Layer to the Participant.
        qos.transport().user_transports.push_back(ethernet_transport);

        // Avoid using the default transport
        qos.transport().use_builtin_transports = false;
        //!--
    }

    {
        //CONF-PAYLOAD-COMPRESSION-TRANSPORT
        DomainParticipantQos participant_qos;

        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Create a descriptor for the new transport.
        auto compression_transport =
                std::make_shared<PayloadCompressionTransportDescriptor>(udp_transport);

        // [OPTIONAL] Transport configuration
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.payload_compression.compression_library",
                    "AUTOMATIC"));

        // Link the Transport Layer to the Participant.
        participant_qos.transport().use_builtin_transports = false;
        participant_qos.transport().user_transports.push_back(compression_transport);
        //!--
    }

    {
        //CONF-HEADER-REDUCTION-TRANSPORT
        DomainParticipantQos participant_qos;

        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        // Create a descriptor for the new transport.
        auto header_reduction_transport = std::make_shared<HeaderReductionTransportDescriptor>(
            udp_transport);

        // [OPTIONAL] Transport configuration
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.remove_version", "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.remove_vendor_id", "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.submessage.combine_id_and_flags",
                    "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.submessage.compress_entitiy_ids",
                    "16,16"));

        // Link the Transport Layer to the Participant.
        participant_qos.transport().use_builtin_transports = false;
        participant_qos.transport().user_transports.push_back(header_reduction_transport);
        //!--
    }

    {
        //CONF-LOW-BANDWIDTH-TRANSPORT
        DomainParticipantQos participant_qos;

        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

        auto compress_transport =
                std::make_shared<PayloadCompressionTransportDescriptor>(udp_transport);
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.payload_compression.compression_library",
                    "AUTOMATIC"));

        auto header_reduction_transport = std::make_shared<HeaderReductionTransportDescriptor>(
            compress_transport);
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.remove_version", "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.remove_vendor_id", "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.submessage.combine_id_and_flags",
                    "true"));
        participant_qos.properties().properties().emplace_back(Property(
                    "rtps.header_reduction.submessage.compress_entitiy_ids",
                    "16,16"));

        participant_qos.transport().use_builtin_transports = false;
        participant_qos.transport().user_transports.push_back(header_reduction_transport);
        //!--
    }
}
