.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _discovery_server:

Discovery Server
----------------

This mechanism is based on a client-server discovery paradigm, i.e. the metatraffic (message exchange among
|DomainParticipants| to identify each other) is managed by one or several server DomainParticipants (left figure), as
opposed to simple discovery (right figure), where metatraffic is exchanged using a message broadcast mechanism like an
IP multicast protocol.

.. figure:: /01-figures/fast_dds/discovery/discovery-server.svg
    :align: center
    :width: 70%

    Comparison of Discovery Server and Simple discovery mechanisms

.. _DS_pro_comparison:

Discovery Server and Discovery Server Pro
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The client-server mechanism described in this page is available in the open-source version of *Fast DDS*.
*Fast DDS Pro* uses the same architecture and adds a filtering stage in the *server*, which cuts down the discovery
traffic that each *client* receives.
The following table summarizes the differences:

.. list-table::
    :header-rows: 1
    :align: left
    :widths: 26 37 37

    *   -
        - Discovery Server
        - Discovery Server |Pro|
    *   - Client-server discovery, without multicast
        - Yes
        - Yes
    *   - Deployment topologies (single, redundant, backup, partitioned, TCP/WAN)
        - Yes
        - Yes
    *   - Discovery information delivered to a |CLIENT|
        - All the information known by the *server*
        - Only the information the *client* needs to match its own endpoints
    *   - Topic-based filtering ("matching" algorithm)
        - Not available
        - The *server* uses the *topics* of each *client* to decide which discovery data it must forward, and leaves
          unmatched the DomainParticipants that do not share a topic
    *   - Discovery traffic received by each *client*
        - Grows with the total number of endpoints in the network
        - Grows with the endpoints that the *client* actually matches
    *   - Difference between a |CLIENT| and a |SUPER_CLIENT|
        - None in practice, as every *client* already receives everything the *server* knows
        - A |CLIENT| receives filtered discovery information, while a |SUPER_CLIENT| receives all of it
    *   - :ref:`Server send rate limiter <DS_send_period>`
        - Not available
        - Batches the accumulated discovery changes, avoiding redundant retransmissions when many DomainParticipants
          join at the same time
    *   - :ref:`Secure discovery <DS_security>`
        - Not available
        - Yes

The rest of this page describes the behavior that is common to both versions.
The sections and items that are exclusive to *Fast DDS Pro* are marked with the |Pro| badge.

.. _DS_why:

Why use a Discovery Server
^^^^^^^^^^^^^^^^^^^^^^^^^^

- **It does not depend on multicast.**
  Each *client* only needs to reach the address of its *server*, which makes the mechanism suitable for networks where
  multicast is unreliable or disabled, such as WiFi, cloud deployments, or container networks.

- **It relies on a central known point for discovery.**
  With :ref:`Simple Discovery <simple_disc_settings>` every DomainParticipant announces itself to, and waits for a
  response from, every other DomainParticipant, so the metatraffic is spread over every link of the network.
  With a Discovery Server, each DomainParticipant exchanges discovery information only with its *server*.
  This is convenient in distributed deployments running over unstable or bandwidth-limited links, such as WiFi
  networks, where the repeated announcements to every peer are expensive and prone to loss.
  Centralizing discovery does not mean introducing a single point of failure: several *servers* can be deployed and
  *clients* can connect to all of them at the same time, so discovery keeps working if one of them goes down
  (see :ref:`Redundancy example <discovery_server_redundancy_scenario_setup>`).

- **It works over WAN and TCP.**
  As the *server* is reached through explicit locators, the discovery phase can be routed over TCP or across
  networks (see :ref:`use-case-tcp-discovery-server`).

- **It gives control over who discovers whom.**
  Only the DomainParticipants connected to the same *server* network discover each other, which can be used to
  isolate parts of the system (see :ref:`discovery_server_partitioning_setup`).

.. _DS_user_data:

Does the user data go through the server?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**No.**
The *server* only mediates the discovery phase.
Once two endpoints have discovered each other, they exchange user data directly between them, exactly as they would
with any other discovery mechanism.

.. figure:: /01-figures/fast_dds/discovery/ds_discovery_vs_user_data.svg
    :align: center
    :width: 80%

    The *server* participates in the discovery phase only; user data always flows peer-to-peer

Therefore the *server* is not a data broker: it does not add latency to the user data, and if it becomes unavailable,
the DomainParticipants that already discovered each other keep communicating.
Only new discovery is affected, which can be mitigated with
:ref:`redundant <discovery_server_redundancy_scenario_setup>` or
:ref:`backup <discovery_server_persistency_scenario_setup>` *servers*.

.. _DS_quick_start:

Quick start
^^^^^^^^^^^

The fastest way to try the mechanism out is the :ref:`fastdds discovery <cli_discovery>` command of the
:ref:`Fast DDS CLI <ffastddscli_cli>`, which ships with Fast DDS and launches a *server* without writing any code:

.. code-block:: bash

    fastdds discovery -l 127.0.0.1 -p 11811

Then run the applications that must discover each other, setting the
:ref:`ROS_DISCOVERY_SERVER <env_vars_ros_discovery_server>` environment variable to the *server* locator.
Any DomainParticipant created with the default |SIMPLE| discovery protocol is automatically turned into a |CLIENT| of
that *server*:

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811

That is all that is needed for a local setup.
The rest of this page explains how to configure the same behavior permanently, either programmatically or through
:ref:`XML profiles <xml_profiles>`, and how to tune it.

.. note::

  :ref:`DDS Domain <dds_layer_domain>` concept does not apply when enabling the default Discovery Server mechanism,
  but it applies when using :ref:`ROS2_EASY_MODE<env_vars_easy_mode>`.

.. _DS_deployments:

Choosing a deployment
^^^^^^^^^^^^^^^^^^^^^

The Discovery Server supports several topologies.
The following table summarizes the most common ones and where to find a complete configuration example of each:

.. list-table::
    :header-rows: 1
    :align: left
    :widths: 22 48 30

    *   - Deployment
        - When to use it
        - Where to look
    *   - Single server
        - Default choice. One *server* handles the discovery of every DomainParticipant in the network.
        - :ref:`Basic example <discovery_server_major_scenario_setup>`
    *   - Redundant servers
        - The system cannot afford to stop discovering new entities if a *server* goes down.
          *Clients* connect to several *servers* at the same time.
        - :ref:`Redundancy example <discovery_server_redundancy_scenario_setup>`
    *   - Backup server
        - The *server* must recover the network graph after an unexpected shutdown, without waiting for the *clients*
          to announce themselves again.
        - :ref:`Persistency example <discovery_server_persistency_scenario_setup>`
    *   - Partitioned network
        - Only certain groups of DomainParticipants should discover each other.
        - :ref:`Partitioning example <discovery_server_partitioning_setup>`
    *   - TCP / WAN
        - Discovery must cross network boundaries, or the deployment cannot use UDP.
        - :ref:`TCP with Discovery Server <use-case-tcp-discovery-server>`
    *   - ROS 2
        - The nodes are ROS 2 nodes, either configured manually or through ``ROS2_EASY_MODE``.
        - :ref:`ROS 2 tutorial <ros2-discovery-server>`,
          :ref:`ROS2_EASY_MODE <env_vars_easy_mode>`

.. _DS_key_concepts:

Key concepts
^^^^^^^^^^^^

In this architecture there are several key concepts to understand:

- The Discovery Server mechanism reuses the RTPS discovery messages structure, as well as the standard DDS
  |DataWriters| and |DataReaders|.

- Discovery Server DomainParticipants may be *clients* or *servers*.
  The only difference between them is how they handle discovery traffic.
  The user traffic, that is, the traffic among the DataWriters and DataReaders they create, is role-independent.

- A |SERVER| is a participant to which the *clients* (and maybe other *servers*) send their discovery information.

  * The role of the *server* is to redistribute its *clients* discovery information to its known
    *clients* and *servers*.
  * A *server* also announces the existence of a new *server* to its known *servers*, and vice versa.
    In this way, a new server can connect to every other existing *server* in the network by just knowing the
    existence of one of them.
    In this way, a mesh topology between servers is created with minimal configuration.
  * The discovery information that is redistributed might come from a **direct** *client* connected to the |SERVER|,
    or from another *server* that is redirecting the discovery data from **its** *clients*.
  * Known *servers* will receive all the information from the **direct** *clients* known by the *server* and the
    participant information of other *servers* (to announce a new server).
  * |Pro| Known *clients* will only receive the information they need to establish communication, i.e. the information
    about the DomainParticipants, DataWriters, and DataReaders to which they match.
    This means that the *server* runs a "matching" algorithm to sort out which information is required by which
    *client*.
    This optimized behavior is exclusive to Fast DDS Pro: the open-source version does not implement the "matching"
    algorithm, so *clients* receive all the discovery information known by the *server*.

- A |BACKUP| *server* is a *server* that persists its discovery database into a file.

  * This type of *server* can load the network graph from a file on start-up without the need of receiving any
    *client's* information.
  * It can be used to persist the *server* knowledge about the network between runs, thus securing the *server's*
    information in case of unexpected shutdowns.
  * It is important to note that the discovery times will be negatively affected when using this type of *server*,
    since periodically writing to a file is an expensive operation.

- A |CLIENT| is a participant that connects to one or more *servers* from which it receives the discovery information
  it needs to establish communication with matching endpoints.

  * |Pro| *Clients* receive only the discovery information they require to establish communication with matching
    endpoints.
    Since the open-source version does not implement the *server* "matching" algorithm, every *client* receives all the
    discovery information known by the *server*, thus behaving as a |SUPER_CLIENT|.
  * *Clients* require prior knowledge of the *servers* to which they want to link.
    Basically, it consists of a list of locators where the *servers* are listening, namely, an IP address and a port.
    These locators also define the transport protocol (UDP or TCP) the client will use to contact the *server*.

- A |SUPER_CLIENT| is a *client* that receives all the discovery information known by the *server*, in opposition to
  *clients*, which only receive the information they need.
  This distinction only applies to Fast DDS Pro, the version implementing the *server* "matching" algorithm.
  In the open-source version every *client* already receives all the discovery information known by the *server*, and
  therefore behaves as a |SUPER_CLIENT|.

  .. note::

      A |SUPER_CLIENT| does not behave as a *Server* as it only receives the discovery information through the *Server* to
      which it is connected.
      It will not connect to other servers, and it will not redistribute the information it receives.
      Any DomainParticipant discovered by the *Server* with no endpoints will not be known by the |SUPER_CLIENT|.

- *Servers* do not require any prior knowledge of their *clients*, but they must listen in the address specified
  by the locator provided to the *clients*.
  *Clients* send discovery messages to the *servers* at regular intervals (ping period) until they receive message
  reception acknowledgement.
  From then on, the *server* knows about the *client* and will inform it of the relevant discovery information.
  The same principle applies to a *server* connecting to another *server*.

.. _DS_discovery_protocol:

Choosing between Client and Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is set by the :ref:`Discovery Protocol <discovery_protocol>` general setting.
A participant can only play one role (despite the fact that a *server* may connect to other *servers*).
It is mandatory to fill this value because it defaults to |SIMPLE|.
The examples below shows how to set this parameter both programmatically and using XML.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_DISCOVERY_PROTOCOL
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-SERVER-DISCOVERY-PROTOCOL<-->
        :end-before: <!--><-->
        :lines: 2-3,5-18
        :append: </profiles>

.. _DS_locators:

The server locator list
^^^^^^^^^^^^^^^^^^^^^^^

Each *server* must specify valid locators where it can be reached.
Any *client* must be given proper locators to reach each of its *servers*.
Below are two examples of a *server* and a *client* side setup.

Server side setup
"""""""""""""""""

The examples below show how to setup the server locator list and XML tag.
Each locator must contain:

- IP address.
- Port.
- Transport protocol (UDPv4/6 or TCPv4/6).

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_SERVER_LOCATORS
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-SERVER-SERVER-LOCATORS<-->
        :end-before: <!--><-->
        :lines: 2-3,5-19
        :append: </profiles>

Note that a *server* can connect to other *servers*, thus, the following section may also apply.

Client side setup
"""""""""""""""""

Each *client* must keep a list of locators associated to the *servers* to which it wants to link.

Note that providing an unreachable locator will result in the *client* sending ping messages to that direction at
regular intervals until it is connected to the same amount of servers that has been configured in the locator list.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_CLIENT_LOCATORS
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-SERVER-CLIENT-LOCATORS<-->
        :end-before: <!--><-->
        :lines: 2-3,5-21
        :append: </profiles>

.. note::

    Additionally, a logical port can be specified in the locator.
    If this parameters is left empty, Fast DDS will automatically assign a logical port equal to the physical
    port whenever it is needed.
    This behavior is coherent with the logic implemented in the :ref:`env_vars_ros_discovery_server` environment
    variable and the :ref:`Fast DDS CLI<cli_discovery>` tool.

.. _DS_ping_period:

Fine tuning discovery server handshake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As explained :ref:`above <DS_key_concepts>` the *clients* send discovery messages to the *servers* at regular
intervals (ping period) until they receive as many message reception acknowledgement as remote locators
(server addresses) were specified.
Mind that this period also applies for those *servers* which connect to other *servers*.
The default value for this period is 450 ms, but it can be configured to a different value.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_CLIENT_PING
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-SERVER-CLIENT-PING<-->
        :end-before: <!--><-->
        :lines: 2-3,5-16
        :append: </profiles>

.. _DS_send_period:

Fine tuning the server send rate limiter |Pro|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``fastdds.discovery_server.send_period`` property controls the minimum interval (in
milliseconds) between consecutive flushes of accumulated discovery changes to the writer
histories. When set to a positive integer value (e.g. ``"1000"`` for one second), the server
still processes incoming data on every routine iteration (triggered by ``<clientAnnouncementPeriod>``),
but defers the send step until at least that many milliseconds have elapsed since the last flush.
All changes that accumulate in that window are sent together in one batch.

This is useful in large-scale scenarios where many participants join simultaneously: without
rate limiting, the server sends partial discovery data before all endpoints of a participant
have been received, requiring redundant retransmissions once the remaining data arrives.
A longer send period lets changes accumulate so that sends contain more complete information,
reducing overall traffic.

.. _DS_guidPrefix:

The GuidPrefix as an optional server unique identifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The |GuidPrefix_t-api| attribute belongs to the RTPS specification and univocally identifies each RTPSParticipant.
It consists on 12 bytes, and in Fast DDS is a key for the DomainParticipant used in the DDS domain.
Fast DDS defines the DomainParticipant |GuidPrefix_t-api| as a public data member of the
|WireProtocolConfigQos-api| class.
In the new Discovery Server mechanism, it is a completely optional parameter.
However, it might be required in specific scenarios to operate with Discovery Server entities of Fast DDS v2.x or
older, where the |GuidPrefix_t-api| was mandatory.

Server side setup
"""""""""""""""""

The examples below show how to manage the corresponding enum data member and XML tag.

.. tab-set::

   .. tab-item:: C++ - Option 1
       :sync: cpp

       .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_1
           :end-before: //!--
           :dedent: 8

   .. tab-item:: C++ - Option 2

       .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_2
           :end-before: //!--
           :dedent: 8

   .. tab-item:: XML
       :sync: xml

       .. literalinclude:: /../code/XMLTester.xml
           :language: xml
           :start-after: <!-->CONF-SERVER-SERVER-PREFIX<-->
           :end-before: <!--><-->
           :lines: 2-3,5-
           :append: </profiles>

.. important::
     When selecting a GUID prefix for the *server*, it is important to take into account that Fast DDS also uses this
     parameter to identify participants in the same host or process and translate locators to localhost or enable
     intra-process communications.
     It is recommended to let Fast DDS to automatically generate the GUID prefix to guarantee the correct behavior of
     these features.
     Setting two DomainParticipant GUID prefixes as intra-process compatible will result in no communication if the
     DomainParticipants run in separate processes.
     For more information, please refer to :ref:`intraprocess_delivery_guids`.

.. warning::
    Launching more than one server using the same GUID prefix is undefined behavior.

.. _DS_modify_server_list:

Modifying remote servers list at run time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once a *server* or *client* is running, it is possible to programmatically modify the participant's list of remote
*servers* to which the running *server* or *client* should connect.
This is done by calling |DomainParticipant::set_qos-api| with a |DomainParticipantQos-api| which has a modified
|WireProtocolConfigQos-api| (see :ref:`wireprotocolconfigqos`).
This feature allows to include a new remote server into the Discovery Server network or modify the remote server locator
in case that the remote server is relaunched with a different listening locator.

.. important::
     The updated list of remote *servers* will modify the ping routine of a *client* or *server*, but it will not
     affect the already established connections.
     Hence, deleting a locator from the list will not disconnect the *server* or *client* from the remote server.
     However, it will impede reconnection if the connection is lost.

.. note::
    The remote server list can also be modified using the ``ROS_DISCOVERY_SERVER`` environment variable.
    Please refer to :ref:`env_vars_fastdds_environment_file` for more information.

.. warning::
    It is strongly advised to use either the API or the environment file.
    Using both at the same time may cause undefined behavior.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_ADD_SERVERS
        :end-before: //!--
        :dedent: 8

.. _DS_dns_name:

Configure Discovery Server locators using names
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All the examples provided in :ref:`discovery_server` use IPv4 addresses to specify the servers' listening locators.
However, *Fast DDS* also allows to :ref:`specify locator addresses using names <transport_transportApi_ipLocator>`.

.. _DS_full_example:

Full example
^^^^^^^^^^^^

The following constitutes a full example on how to configure *server* and *client* both programmatically and using XML.
You may also have a look at the *eProsima Fast DDS* Github repository, which contains
:fastdds-tree:`an example <examples/cpp/discovery_server>` similar to the one discussed in this section, as well as
multiple other examples for different use cases.

Server side setup
"""""""""""""""""

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_FULL_EXAMPLE
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF_SERVER_FULL_EXAMPLE<-->
        :end-before: <!--><-->
        :lines: 2-3,5-36
        :append: </profiles>

Client side setup
"""""""""""""""""

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_CLIENT_FULL_EXAMPLE
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF_CLIENT_FULL_EXAMPLE<-->
        :end-before: <!--><-->
        :lines: 2-3,5-31
        :append: </profiles>

.. _DS_security:

Security |Pro|
^^^^^^^^^^^^^^

Configuring :ref:`security` on *servers* and *clients* is done the same way as for any other participant.
This section depicts the limitations imposed by the security enforcement on the communication between
*clients* and *servers*, and which discovery information is propagated by a *server* depending on the security
configuration of the *clients* and *servers* to which it is connected.

It is important to note that for enabling a secure discovery when using Discovery Server, *Fast DDS* must be compiled
with security support (see :ref:`cmake_options`), and the :ref:`domain_governance_doc` must explicitly encrypt the
discovery.

As in SDP, when using this feature, the Domain Governance Document of all *clients* and *servers* connecting to a
*server* must match that of the *server*, which implies that all |DomainParticipants| belonging to the same Discovery
Server network must configure the discovery protection in the same manner.

Although the *server* mediates the discovery process and creates connections between *clients*, the *clients* themselves
still go through the PKI (Public Key Infrastructure) exchange in order to have a secure communication between them.

.. important::

  In order to keep the behavior consistent with the QoS Policies, the *server* does not check the
  :ref:`domainparticipant_permissions_doc` of the |DomainParticipants| that it is connecting.

.. important::

  Security support for Discovery Server is only supported in Fast DDS Pro.
