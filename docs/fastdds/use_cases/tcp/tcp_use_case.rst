.. include:: ../../../03-exports/aliases.include

.. _use-case-tcp:

Fast DDS over TCP
=================

As explained in :ref:`transport_tcp_tcp`, Fast DDS offers the possibility to communicate nodes within distributed
applications with DDS over a TCP transport layer.
This has the advantage of leveraging the builtin flow control and reliability of the TCP protocol, which in the
context of DDS applications is best suited for the transmission of large payloads, i.e. large data samples, over
lossy networks.
Examples of such cases would be transmitting video or large point clouds resulting from laser scanning over WiFi
links.

The configuration of the TCP transport typically involves an *a priori* knowledge of the deployment in order
to set :ref:`Simple Initial Peers` for :ref:`discovery`, which may not always be possible and creates difficulties when
reallocating nodes of the distributed applications, as the entire discovery configuration needs to be changed.
To overcome this problem, these use cases present an approach for leveraging the Fast DDS' TCP transport capabilities
while at the same time not requiring configuration modifications when the deployment changes over time.
One option is to configure the participant discovery phase (see :ref:`disc_phases`) to occur over UDP multicast, while
the application data delivery occurs over TCP. Also, it is possible to enable TCP communication while using
:ref:`discovery-server-use-case` to manage :ref:`discovery`.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/tcp/tcp_with_multicast_discovery.rst
    /fastdds/use_cases/tcp/tcp_with_discovery_server.rst
