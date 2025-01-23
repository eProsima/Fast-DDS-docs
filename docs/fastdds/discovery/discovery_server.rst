.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _discovery_server:

Discovery Server Settings
-------------------------

This mechanism is based on a client-server discovery paradigm, i.e. the metatraffic (message exchange among
|DomainParticipants| to identify each other) is managed by one or several server DomainParticipants (left figure), as
opposed to simple discovery (right figure), where metatraffic is exchanged using a message broadcast mechanism like an
IP multicast protocol.
A `Discovery-Server <https://eprosima-discovery-server.readthedocs.io/en/latest/index.html>`_ tool is available to
ease Discovery Server setup and testing.

.. note::

  :ref:`DDS Domain <dds_layer_domain>` concept does not apply when enabling the default Discovery Server mechanism,
  but it applies when using :ref:`ROS2_EASY_MODE<env_vars_easy_mode>`.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

.. figure:: /01-figures/fast_dds/discovery/discovery-server.svg
    :align: center
    :width: 50%

    Comparison of Discovery Server and Simple discovery mechanisms

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
    * Known *clients* will only receive the information they need to establish communication, i.e. the information
      about the DomainParticipants, DataWriters, and DataReaders to which they match.
      This means that the *server* runs a "matching" algorithm to sort out which information is required by which
      *client*.

- A |BACKUP| *server* is a *server* that persists its discovery database into a file.

    * This type of *server* can load the network graph from a file on start-up without the need of receiving any
      *client's* information.
    * It can be used to persist the *server* knowledge about the network between runs, thus securing the *server's*
      information in case of unexpected shutdowns.
    * It is important to note that the discovery times will be negatively affected when using this type of *server*,
      since periodically writing to a file is an expensive operation.

- A |CLIENT| is a participant that connects to one or more *servers* from which it receives only the discovery
  information they require to establish communication with matching endpoints.

    * *Clients* require prior knowledge of the *servers* to which they want to link.
      Basically, it consists of a list of locators where the *servers* are listening, namely, an IP address and a port.
      These locators also define the transport protocol (UDP or TCP) the client will use to contact the *server*.

- A |SUPER_CLIENT| is a *client* that receives the discovery information known by the *server*, in opposition to
  *clients*, which only receive the information they need.

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

.. tabs::

   .. tab:: **C++**

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF_SERVER_DISCOVERY_PROTOCOL
         :end-before: //!--
         :dedent: 8

   .. tab:: **XML**

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

.. tabs::

   .. tab:: **C++**

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF_SERVER_SERVER_LOCATORS
         :end-before: //!--
         :dedent: 8

   .. tab:: **XML**

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

.. tabs::

   .. tab:: **C++**

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF_SERVER_CLIENT_LOCATORS
         :end-before: //!--
         :dedent: 8

   .. tab:: **XML**

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

.. tabs::

  .. tab:: **C++**

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF_SERVER_CLIENT_PING
      :end-before: //!--
      :dedent: 8

  .. tab:: **XML**

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-SERVER-CLIENT-PING<-->
      :end-before: <!--><-->
      :lines: 2-3,5-16
      :append: </profiles>

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

.. tabs::

  .. tab:: **C++** - Option 1

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_1
        :end-before: //!--
        :dedent: 8

  .. tab:: **C++** - Option 2

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_2
        :end-before: //!--
        :dedent: 8

  .. tab:: **XML**

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

.. tabs::

  .. tab:: **C++**

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
You may also have a look at the *eProsima Fast DDS* Github repository, which contains `an example <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/discovery_server>`_
similar to the one discussed in this section, as well as multiple other examples for different use cases.

Server side setup
"""""""""""""""""

.. tabs::

  .. tab:: **C++**

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF_SERVER_FULL_EXAMPLE
       :end-before: //!--
       :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF_SERVER_FULL_EXAMPLE<-->
       :end-before: <!--><-->
       :lines: 2-3,5-36
       :append: </profiles>

Client side setup
"""""""""""""""""

.. tabs::

  .. tab:: **C++**

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF_CLIENT_FULL_EXAMPLE
       :end-before: //!--
       :dedent: 8

  .. tab:: **XML**

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF_CLIENT_FULL_EXAMPLE<-->
       :end-before: <!--><-->
       :lines: 2-3,5-31
       :append: </profiles>

.. _DS_security:

Security
^^^^^^^^

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

  Security support for Discovery Server is only supported from Fast DDS v2.10.0 onward.
