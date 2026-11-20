<a id="use-case-tcp"></a>

<a id="large-data-mode-and-fast-dds-over-tcp"></a>

# Large Data mode and Fast DDS over TCP

As explained in [TCP Transport](../../transport/tcp/tcp.md#transport-tcp-tcp), Fast DDS offers the possibility to communicate nodes within distributed
applications with DDS over a TCP transport layer.
This has the advantage of leveraging the builtin flow control and reliability of the TCP protocol, which in the
context of DDS applications is best suited for the transmission of large payloads, i.e. large data samples, over
lossy networks.
Examples of such cases would be transmitting video or large point clouds resulting from laser scanning over WiFi
links.

The configuration of the TCP transport typically involves an *a priori* knowledge of the deployment in order
to set [Initial peers](../../discovery/simple.md#simple-initial-peers) for [Discovery](../../discovery/discovery.md#discovery), which may not always be possible and creates difficulties when
reallocating nodes of the distributed applications, as the entire discovery configuration needs to be changed.
To overcome this problem, Fast DDS presents the `LARGE_DATA` builtin transports configuration as an approach for
leveraging the Fast DDS’ TCP transport capabilities while at the same time not requiring configuration modifications
when the deployment changes over time.

`LARGE_DATA` has been specifically designed to improve communication performance of large data samples over lossy
networks. When configured, UDP transport will exclusively be used during the [PDP discovery](../../discovery/discovery.md#disc-phases) phase,
taking advantage of the more reliable TCP/SHM for the remainder of the communication process. Fast DDS offers
an extremely straightforward implementation for this mode through an environment variable, XML profiles or via code.
Additionally, it is possible to use the builtin transports configuration options to customize the `LARGE_DATA` mode
to better suit the specific requirements of each user.

For a video demonstration showcasing a practical example of this configuration, please refer to:
[Large Data communication with ROS 2](https://docs.vulcanexus.org/en/latest/rst/use_cases/large_data/large_data.html).

Also, it is possible to enable TCP communication while using
[Discovery Server](../wifi/discovery_server_use_case.md#discovery-server-use-case) to manage [Discovery](../../discovery/discovery.md#discovery).

* [Large Data Mode](tcp_with_multicast_discovery.md)
* [Large Data with configuration options](tcp_large_data_with_options.md)
* [TCP Communication with Discovery Server](tcp_with_discovery_server.md)
