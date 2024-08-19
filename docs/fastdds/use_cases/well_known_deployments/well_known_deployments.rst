.. _well_known_deployments:

Well Known Network Deployments
==============================

It is often the case in industrial deployments, such as productions lines, that the entire network topology (hosts, IP
addresses, etc.) is known beforehand.
Such scenarios are perfect candidates for *Fast DDS* STATIC :ref:`discovery` mechanism, which drastically reduces
the middleware setup time (time until all the entities are ready for information exchange),
while at the same time limits the connections to those strictly necessary.

Knowing the complete network topology allows to:

* Minimize the PDP meta-traffic and avoid multicast communication with :ref:`wide_deployments_static_pdp`.
* Completely avoid the EDP with :ref:`wide_deployments_static_edp`.


.. _wide_deployments_static_pdp:

Peer-to-Peer Participant Discovery Phase
----------------------------------------

The SIMPLE PDP discovery phase entails the :ref:`DomainParticipants<dds_layer_domainParticipant>` sending periodic PDP
announcements over multicast, and answering to the announcements received from remote
DomainParticipants.
As a result, the number of PDP connections grows quadratically with the number of
DomainParticipants, resulting in a large amount of meta traffic on the network.

However, if all DomainParticipants are known beforehand,
they can be configured to send their announcements only to the unicast addresses of their peers.
This is done by specifying a list of peer addresses, and by disabling the participant multicast
announcements.
As an additional advantage, with this method only the peers configured on the list are known to the
DomainParticipant, allowing to arrange which participant will communicate with which.
This reduces the amount of meta traffic if not all the DomainParticipants
need to be aware of all the rest of the remote participants present in the network.

Use-case :ref:`use-case-fast-rtps-over-wifi` provides a detailed explanation on how to configure *Fast DDS* for such
cases.

.. _wide_deployments_static_edp:

STATIC Endpoint Discovery Phase
-------------------------------

Users can manually configure which :ref:`dds_layer_publisher` and :ref:`dds_layer_subscriber` match with
each other, so they can start sharing user data right away, avoiding the EDP phase.

A complete description of the feature can be found at :ref:`discovery_static`.
There is also a fully functional hello world example implementing STATIC EDP in the
`examples/cpp/static_edp_discovery <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/static_edp_discovery>`_
folder.

The following subsections present an example configuration where a Publisher in
Topic ``HelloWorldTopic`` from DomainParticipant ``HelloWorldPublisher``
is matched with a Subscriber from DomainParticipant ``HelloWorldSubscriber``.


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
|    :lines: 2-3,5-                                   |
|    :append: </profiles>                             |
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
|    :lines: 2-3,5-                                   |
|    :append: </profiles>                             |
+-----------------------------------------------------+

