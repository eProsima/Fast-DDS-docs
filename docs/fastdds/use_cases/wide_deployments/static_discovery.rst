.. _wide_deployments_static:

Well Known Network Topologies
=============================

It is often the case in industrial deployments, such as productions lines, that the entire network topology (hosts, IP
addresses, etc.) is known beforehand.
Such scenarios are perfect candidates for Fast DDS STATIC :ref:`discovery` mechanism, which drastically reduces
the middleware setup time (time until all the entities are ready for information exchange),
while at the same time limits the connections to those strictly necessary.

Knowing the complete network topology allows to:

* Minimize the PDP meta-traffic and avoid multicast communication with :ref:`wide_deployments_static_pdp`.
* Completely avoid the EDP with :ref:`wide_deployments_static_edp`.


.. _wide_deployments_static_pdp:

Peer-to-Peer Participant Discovery Phase
----------------------------------------

.. _RTPS v2.2 standard: https://www.omg.org/spec/DDSI-RTPS/2.2/

The SIMPLE PDP discovery phase entails the :ref:`DomainParticipants<dds_layer_domainParticipant>` sending periodic PDP
announcements over multicast, and answering to the announcements received from remote
:ref:`DomainParticipants<dds_layer_domainParticipant>`.
As a result, the number of PDP connections grows quadratically with the number of
:ref:`DomainParticipants<dds_layer_domainParticipant>`, resulting in a large amount of meta traffic on the network.

However, it is often the case that not all the :ref:`DomainParticipants<dds_layer_domainParticipant>` need to be aware
of all the rest of the remote participants present in the network.
Instead of sending PDP announcements on multicast, Fast DDS :ref:`DomainParticipants<dds_layer_domainParticipant>`
can be configured to send their announcements only to specific unicast addresses, which correspond to the
addresses of their peer :ref:`DomainParticipants<dds_layer_domainParticipant>`.

This is done by specifying a list of peers as a set of IP address-port pairs, and by disabling the participant multicast
announcements.
Use-case :ref:`use-case-fast-rtps-over-wifi` provides a detailed explanation on how to configure Fast-RTPS for such
case.

.. _wide_deployments_static_edp:

STATIC Endpoint Discovery Phase
-------------------------------

Users can manually configure which :ref:`dds_layer_publisher` and :ref:`dds_layer_subscriber` match with
each other, so they can start sharing user data right away, avoiding the EDP phase.

A complete description of the feature can be found at :ref:`discovery_static`.
There is also a fully functional helloworld example implementing STATIC EDP in the
``examples/C++/DDS/StaticHelloWorldExample`` folder.

The following subsections present an example configuration where a :ref:`dds_layer_publisher` in
:ref:`dds_layer_topic` ``HelloWorldTopic`` from :ref:`dds_layer_domainParticipant` ``HelloWorldPublisher``
is matched with a :ref:`dds_layer_subscriber` from :ref:`dds_layer_domainParticipant` ``HelloWorldSubscriber``.


Create STATIC discovery XML files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   +-----------------------------------------------------+
   | **HelloWorldPublisher.xml**                         |
   +=====================================================+
   | .. literalinclude:: /../code/StaticTester.xml       |
   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_PUB |
   |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+

   +-----------------------------------------------------+
   | **HelloWorldSubscriber.xml**                        |
   +=====================================================+
   | .. literalinclude:: /../code/StaticTester.xml       |
   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_SUB |
   |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+

Create entities and load STATIC discovery XML files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When creating the entities, the local writer/reader attributes must match those defined in the STATIC discovery
XML file loaded by the remote entity.

   +-----------------------------------------------------+
   | **PUBLISHER**                                       |
   +=====================================================+
   | **C++**                                             |
   +-----------------------------------------------------+
   | .. literalinclude:: /../code/DDSCodeTester.cpp      |
   |    :language: c++                                   |
   |    :start-after: //STATIC_DISCOVERY_USE_CASE_PUB    |
   |    :end-before: //!--                               |
   |    :dedent: 8                                       |
   +-----------------------------------------------------+
   | **XML**                                             |
   +-----------------------------------------------------+
   | .. literalinclude:: /../code/XMLTester.xml          |
   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_PUB |
   |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+

   +-----------------------------------------------------+
   | **SUBSCRIBER**                                      |
   +=====================================================+
   | **C++**                                             |
   +-----------------------------------------------------+
   | .. literalinclude:: /../code/DDSCodeTester.cpp      |
   |    :language: c++                                   |
   |    :start-after: //STATIC_DISCOVERY_USE_CASE_SUB    |
   |    :end-before: //!--                               |
   |    :dedent: 8                                       |
   +-----------------------------------------------------+
   | **XML**                                             |
   +-----------------------------------------------------+
   | .. literalinclude:: /../code/XMLTester.xml          |
   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_SUB |
   |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+

