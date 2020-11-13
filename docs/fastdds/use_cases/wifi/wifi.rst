.. _use-case-fast-rtps-over-wifi:

Fast DDS over WIFI
==================

.. _RTPS v2.2 standard: https://www.omg.org/spec/DDSI-RTPS/2.2/

The `RTPS v2.2 standard`_ defines the SIMPLE :ref:`discovery` as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP).
This can be a problem in cases where WiFi communication is used, since multicast is not as reliable over WiFi
as it is over ethernet.

The recommended solution to this challenge is to configure an initial list of remote peers on the
:ref:`dds_layer_domainParticipant`, so that it can set unicast communication with them.
This way, the use of multicast is not needed to discover these initial peers.
Furthermore, if all the peers are known and configured beforehand, all multicast communication can be
removed.

Alternatively, **Discovery Server** can be used to avoid multicast discovery.
A DomainParticipant with a well-know address acts as a discovery server,
providing the rest of the participants the information required to connect among them.
If all the peers are known and configured beforehand, STATIC discovery can be used instead,
completely avoiding the discovery phase.
Use-case :ref:`well_known_deployments` provides a detailed explanation on how to configure *Fast DDS* for
STATIC discovery.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/wifi/initial_peers.rst
    /fastdds/use_cases/wifi/disable_multicast.rst
    /fastdds/use_cases/wifi/discovery_server_use_case.rst

