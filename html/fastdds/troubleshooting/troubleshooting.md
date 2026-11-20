<a id="troubleshooting"></a>

<a id="id1"></a>

# Troubleshooting

This section offers hints and pointers to help users with navigating through the documentation while troubleshooting
issues.

* Although UDP/SHM default transports of Fast DDS are designed to work in most network environments, they may encounter
  certain limitations when operating over WiFi or within lossy network conditions. In these cases, it is advisable to
  set up the `LARGE_DATA` configuration, which has been specifically optimized for these scenarios. The
  `LARGE_DATA` profile limits the use of UDP solely to the [PDP discovery](../discovery/discovery.md#disc-phases) phase, employing the more
  reliable TCP/SHM for the remainder of the communication process. Its implementation can be accomplished by simply
  configuring the `FASTDDS_BUILTIN_TRANSPORTS` environment variable, or alternatively through XML profiles
  or via code. For more information, please refer to [Large Data Mode](../use_cases/tcp/tcp_with_multicast_discovery.md#use-case-tcp-multicast).

  ### Environment Variable

  ```bash
  export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
  ```

  ### XML

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds xmlns="http://www.eprosima.com">
      <profiles>
          <!--
              UDP transport for PDP and SHM/TCPv4 transport for both EDP and application data
          -->
          <participant profile_name="large_data_builtin_transports" is_default_profile="true">
              <rtps>
                  <builtinTransports>LARGE_DATA</builtinTransports>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

  ### C++

  ```c++
  eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

  /* Transports configuration */
  // UDPv4 transport for PDP over multicast and SHM / TCPv4 transport for EDP and application data
  pqos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);

  /* Create participant as usual */
  eprosima::fastdds::dds::DomainParticipant* participant =
          eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
  ```
* If having problems with transmitting **large samples** when using the `LARGE_DATA` mode, try to use the builtin
  transports configuration options to adjust `LARGE_DATA` to your specific use case.
  Please refer to [Large Data with configuration options](../use_cases/tcp/tcp_large_data_with_options.md#use-case-large-data-options) for more information.
* If having problems with transmitting **large samples such as video or point clouds**, please refer to
  [Large Data Rates](../use_cases/large_data/large_data.md#use-case-largedata).
* Fast DDS v3 introduced the new feature [XTypes](../xtypes/xtypes.md#dynamic-types), which allows to discover remote types.
  In consequence, discovery traffic can be increased during start up.
  If you are experiencing high load during discovery, try disabling the new feature.
  Please refer to [disable type propagation](../property_policies/non_consolidated_qos.md#property-type-propagation) to learn how to do it.
