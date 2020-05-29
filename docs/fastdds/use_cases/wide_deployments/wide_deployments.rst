.. _wide_deployments:

Wide Deployments
================

Systems with large amounts of communication nodes might pose a challenge to
`Data Distribution Service (DDS) <https://www.omg.org/spec/DDS/1.4/PDF>`_ based middleware implementations in terms of
setup times, memory consumption, and network load.
During :ref:`discovery`, the Participant Discovery Phase (PDP) relies on meta traffic
announcements sent to multicast addresses so that all the :ref:`DomainParticipants<dds_layer_domainParticipant>`
in the network can acknowledge each other.
This phase is followed by a Endpoint Discovery Phase (EDP) where all the
:ref:`DomainParticipants<dds_layer_domainParticipant>` use discovered unicast addresses to exchange information about
their :ref:`dds_layer_publisher` and :ref:`dds_layer_subscriber` entities with the rest of the
:ref:`DomainParticipants<dds_layer_domainParticipant>`, so that matching between entities of the same topic can occur.
As the number of :ref:`DomainParticipants<dds_layer_domainParticipant>`, :ref:`Publishers<dds_layer_publisher>`,
and :ref:`Subscribers<dds_layer_subscriber>` increases, the meta-traffic and the number of
connections increase exponentially, severely affecting the setup time and memory consumption.
Fast DDS provides extra features that expand the DDS standard to adapt it to wide deployment scenarios.

* **Server-Client Discovery**:
  This feature is intended to substitute the standard SIMPLE PDP and SIMPLE EDP protocols with a discovery based on a
  server-client architecture, where all the meta-traffic goes through a hub (the server) to be distributed throughout
  the network communication nodes.

* **Static Discovery on well known topologies**:
  With this feature, the user can manually specify which :ref:`dds_layer_domainParticipant` should communicate with
  which one and through which address and port.
  Furthermore, the the user can specify the matching among :ref:`Publishers<dds_layer_publisher>` and
  :ref:`Subscribers<dds_layer_subscriber>`, thus removing all EDP meta traffic.


.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/wide_deployments/server_discovery.rst
    /fastdds/use_cases/wide_deployments/static_discovery.rst

