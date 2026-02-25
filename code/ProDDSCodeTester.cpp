#include <cstdint>

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/ethernet/EthernetTransportDescriptor.hpp>
#include <fastdds/rtps/transport/low-bandwidth/PayloadCompressionTransportDescriptor.hpp>
#include <fastdds/rtps/transport/low-bandwidth/HeaderReductionTransportDescriptor.hpp>
#include <fastdds/rtps/transport/udp_tsn/TSN_UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/udp_tsn/UDPPriorityMappings.hpp>

using namespace eprosima::fastdds::dds;

void dds_transport_pro_examples ()
{
    using TSN_UDPv4TransportDescriptor = eprosima::fastdds::rtps::TSN_UDPv4TransportDescriptor;
    using EthernetTransportDescriptor = eprosima::fastdds::rtps::EthernetTransportDescriptor;
    using UDPPriorityMapping = eprosima::fastdds::rtps::UDPPriorityMapping;
    using UDPv4TransportDescriptor = eprosima::fastdds::rtps::UDPv4TransportDescriptor;
    using PayloadCompressionTransportDescriptor = eprosima::fastdds::rtps::PayloadCompressionTransportDescriptor;
    using HeaderReductionTransportDescriptor = eprosima::fastdds::rtps::HeaderReductionTransportDescriptor;
    using Property = eprosima::fastdds::rtps::Property;

    {
        //TSN_SET_UDP_TUPLE
        DomainParticipantQos qos;

        // Create a descriptor for the new transport.
        auto udp_transport = std::make_shared<TSN_UDPv4TransportDescriptor>();

        // Set the TSN-UDP tuples
        UDPPriorityMapping udp_priority_mapping_1;
        int32_t udp_priority_1 = -1;
        udp_priority_mapping_1.source_port = 5000;
        udp_transport->priority_mapping[udp_priority_1] = udp_priority_mapping_1;

        UDPPriorityMapping udp_priority_mapping_2;
        int32_t udp_priority_2 = 1;
        udp_priority_mapping_2.source_port = 5000;
        udp_priority_mapping_2.dscp = 1;
        udp_transport->priority_mapping[udp_priority_2] = udp_priority_mapping_2;

        UDPPriorityMapping udp_priority_mapping_3;
        int32_t udp_priority_3 = 2;
        udp_priority_mapping_3.source_port = 5000;
        udp_priority_mapping_3.dscp = 2;
        udp_priority_mapping_3.interface = "eth0";
        udp_transport->priority_mapping[udp_priority_3] = udp_priority_mapping_3;

        UDPPriorityMapping udp_priority_mapping_4;
        int32_t udp_priority_4 = 3;
        udp_priority_mapping_4.source_port = 5000;
        udp_priority_mapping_4.dscp = 3;
        udp_priority_mapping_4.interface = "10.10.1.2";
        udp_transport->priority_mapping[udp_priority_4] = udp_priority_mapping_4;

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
        ethernet_transport->priority_mapping[ethernet_priority_1] = ethernet_priority_mapping_1;

        EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_2;
        int32_t ethernet_priority_2 = 1;
        ethernet_priority_mapping_2.source_port = 5000;
        ethernet_priority_mapping_2.pcp = 1;
        ethernet_transport->priority_mapping[ethernet_priority_2] = ethernet_priority_mapping_2;

        EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_3;
        int32_t ethernet_priority_3 = 2;
        ethernet_priority_mapping_3.source_port = 5000;
        ethernet_priority_mapping_3.pcp = 2;
        ethernet_priority_mapping_3.vlan_id = 100;
        ethernet_transport->priority_mapping[ethernet_priority_3] = ethernet_priority_mapping_3;

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

void rpcdds_internal_api_examples()
{
    {
        //!--CREATE_DELETE_SERVICE_EXAMPLE
        TypeSupport request_type_support = TypeSupport(new CustomDataType());
        TypeSupport reply_type_support = TypeSupport(new CustomDataType());
        DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);

        // Create a new ServiceTypeSupport instance
        ServiceTypeSupport service_type_support(request_type_support, reply_type_support);

        // Register the ServiceTypeSupport instance in a DomainParticipant
        ReturnCode_t ret;
        ret = participant->register_service_type(service_type_support, "ServiceType");
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        // Create a new Service instance
        Service* service = participant->create_service("Service", "ServiceType");
        if (!service)
        {
            // Error
            return;
        }

        // ... Create Requesters and Repliers here

        // Delete the created Service
        ret = participant->delete_service(service);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }
        //!--
    }

    {
        DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        TypeSupport request_type_support = TypeSupport(new CustomDataType());
        TypeSupport reply_type_support = TypeSupport(new CustomDataType());
        ServiceTypeSupport service_type_support(request_type_support, reply_type_support);
        participant->register_service_type(service_type_support, "ServiceType");
        participant->create_service("Service", "ServiceType");

        //!--RPC_REQUESTER_EXAMPLE
        ReturnCode_t ret;

        // Get a valid service
        Service* service = participant->find_service("Service");
        if (!service)
        {
            // Error
            return;
        }

        /* Create a new Requester instance */
        RequesterQos requester_qos;

        Requester* requester = participant->create_service_requester(service, requester_qos);
        if (!requester)
        {
            // Error
            return;
        }

        /* Send a new Request sample and check if a received Reply sample is a match */
        // Make sure that all RPC Entities are enabled
        if (!service->is_enabled())
        {
            ret = service->enable();
            if (RETCODE_OK != ret)
            {
                // Error
                return;
            }
        }

        //  Enabling the service enables the registered Requester unless
        //  the creation of the internal DataWriter and DataReader failed.
        //  We can make sure that they have been created correctly by checking if the Requester is enabled.
        if (!requester->is_enabled())
        {
            // Error
            return;
        }

        RequestInfo expected_request_info;
        RequestInfo received_request_info;
        // Create a new Request sample
        void* request_data = request_type_support->create_data();
        ret = requester->send_request(request_data, expected_request_info);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        // Wait for some time until a Reply sample is received
        requester->get_requester_reader()->wait_for_unread_message(Duration_t{3,0});

        void* data = nullptr;
        ret = requester->take_reply(data, received_request_info);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        if (expected_request_info.related_sample_identity == received_request_info.related_sample_identity)
        {
          // Received Reply sample is associated to the sent Request sample
          if (received_request_info.has_more_replies)
          {
              // More replies for the same request will be received
          }
        }

        // Delete created Requester
        ret = participant->delete_service_requester(requester->get_service_name(), requester);
        if (RETCODE_OK != ret)
        {
          // Error
          return;
        }
        //!--
    }

    {
        DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        TypeSupport request_type_support = TypeSupport(new CustomDataType());
        TypeSupport reply_type_support = TypeSupport(new CustomDataType());
        ServiceTypeSupport service_type_support(request_type_support, reply_type_support);
        participant->register_service_type(service_type_support, "ServiceType");
        participant->create_service("Service", "ServiceType");

        //!--RPC_REPLIER_EXAMPLE
        ReturnCode_t ret;

        // Get a valid service
        Service* service = participant->find_service("Service");
        if (!service)
        {
            // Error
            return;
        }

        /* Create a new Replier instance */
        ReplierQos replier_qos;

        Replier* replier = participant->create_service_replier(service, replier_qos);
        if (!replier)
        {
            // Error
            return;
        }

        /* Take a received Request sample and send a new Reply sample */

        // Make sure that all RPC Entities are enabled
        if (!service->is_enabled())
        {
            ret = service->enable();
            if (RETCODE_OK != ret)
            {
                // Error
                return;
            }
        }

        //  Enabling the service enables the registered Replier unless
        //  the creation of the internal DataWriter and DataReader failed.
        //  We can make sure that they have been created correctly by checking if the Replier is enabled.
        if (!replier->is_enabled())
        {
            // Error
            return;
        }

        RequestInfo received_request_info;

        // Wait for some time until a Request sample is received
        replier->get_replier_reader()->wait_for_unread_message(Duration_t{3,0});

        void* received_data = nullptr;

        ret = replier->take_request(received_data, received_request_info);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        // ... Process received data

        // Send a Reply with the received related_sample_identity, indicating there will be more replies.
        void* reply_data = reply_type_support->create_data();
        received_request_info.has_more_replies = true;
        ret = replier->send_reply(reply_data, received_request_info);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        // Send a Reply with the received related_sample_identity, indicating it is the last one.
        void* reply_data_2 = reply_type_support->create_data();
        received_request_info.has_more_replies = false;
        ret = replier->send_reply(reply_data_2, received_request_info);
        if (RETCODE_OK != ret)
        {
            // Error
            return;
        }

        /* Delete created Replier */
        ret = participant->delete_service_replier(replier->get_service_name(), replier);
        if (RETCODE_OK != ret)
        {
          // Error
          return;
        }
        //!--
    }
}

void rpcdds_custom_scheduling_examples()
{
    //!--RPC_CUSTOM_SCHEDULING_EXAMPLES
    // A scheduling strategy where requests are processed in the same thread where they are received.
    // Should not be used in servers with feed operations.
    struct DirectRequestScheduling : public eprosima::fastdds::dds::rpc::RpcServerSchedulingStrategy
    {
        void schedule_request(
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcRequest>& request,
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
        {
            server->execute_request(request);
        }

        void server_stopped(
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
        {
            static_cast<void>(server);
        }
    };

    // A scheduling strategy where each request is processed in a detached thread.
    struct DetachedThreadRequestScheduling : public eprosima::fastdds::dds::rpc::RpcServerSchedulingStrategy
    {
        void schedule_request(
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcRequest>& request,
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
        {
            std::thread([server, request](){server->execute_request(request);}).detach();
        }

        void server_stopped(
                const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
        {
            static_cast<void>(server);
        }
    };
    //!--
}
