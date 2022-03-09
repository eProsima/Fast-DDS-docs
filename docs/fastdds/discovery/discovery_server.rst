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
  The only difference between them is on how they handle discovery traffic.
  The user traffic, that is, the traffic among the DataWriters and DataReaders they create, is role-independent.

- All *server* and *client* discovery information will be shared with linked *clients*.
  Note that a *server* may act as a *client* for other *servers*.

- A |SERVER| is a participant to which the *clients* (and maybe other *servers*) send their discovery information.
  The role of the *server* is to re-distribute the *clients* (and *servers*) discovery information to their known
  *clients* and *servers*.
  A *server* may connect to other *servers* to receive information about their *clients*.
  Known *servers* will receive all the information known by the *server*.
  Known *clients* will only receive the information they need to establish communication, i.e. the information about the
  DomainParticipants, DataWriters, and DataReaders to which they match.
  This means that the *server* runs a "matching" algorithm to sort out which information is required by which *client*.

- A |BACKUP| *server* is a *server* that persists its discovery database into a file.
  This type of *server* can load the network graph from a file on start-up without the need of receiving any *client's*
  information.
  It can be used to persist the *server* knowledge about the network between runs, thus securing the *server's*
  information in case of unexpected shutdowns.
  It is important to note that the discovery times will be negatively affected when using this type of *server*, since
  periodically writing to a file is an expensive operation.

- A |CLIENT| is a participant that connects to one or more *servers* from which it receives only the discovery
  information they require to establish communication with matching endpoints.

- *Clients* require a beforehand knowledge of the *servers* to which they want to link.
  Basically it is reduced to the *servers* identity (henceforth called |GuidPrefix_t-api|) and a list of locators
  where the *servers* are listening.
  These locators also define the transport protocol (UDP or TCP) the client will use to contact the *server*.

  - The |GuidPrefix_t-api| is the RTPS standard RTPSParticipant unique identifier, a 12-byte chain.
    This identifier allows *clients* to assess whether they are receiving messages from the right *server*, as each
    standard RTPS message contains this piece of information.

    The |GuidPrefix_t-api| is used because the *server's* IP address may not be a reliable enough server identifier,
    since several *servers* can be hosted in the same machine, thus having the same IP, and also because multicast
    addresses are acceptable addresses.

- A |SUPER_CLIENT| is a *client* that receives all the discovery information known by the *server*, in opposition to
  *clients*, which only receive the information they need.

- *Servers* do not require any beforehand knowledge of their *clients*, but their |GuidPrefix_t-api| and locator list
  (where they are listening) must match the one provided to the *clients*.
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

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_DISCOVERY_PROTOCOL                                                                    |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-DISCOVERY-PROTOCOL<-->                                                             |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-18                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_guidPrefix:

The GuidPrefix as the server unique identifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The |GuidPrefix_t-api| attribute belongs to the RTPS specification and univocally identifies each RTPSParticipant.
It consists on 12 bytes, and in Fast DDS is a key for the DomainParticipant used in the DDS domain.
Fast DDS defines the DomainParticipant |GuidPrefix_t-api| as a public data member of the
|WireProtocolConfigQos-api| class.
In the Discovery Server, it has the purpose to link a *server* to its *clients*.
It must be specified in *server* and *client* setups.

Server side setup
"""""""""""""""""

The examples below show how to manage the corresponding enum data member and XML tag.

+----------------------------------------------------------------------------------------------------------------------+
| **C++** - Option 1: Manual setting of the ``unsigned char`` in ASCII format.                                         |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_1                                                            |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **C++** - Option 2: Using the ``>>`` operator and the ``std::istringstream`` type.                                   |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_GUIDPREFIX_OPTION_2                                                            |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-SERVER-PREFIX<-->                                                                  |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-11                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

Note that a *server* can connect to other *servers*.
Thus, the following section may also apply.

.. important::
     When selecting a GUID prefix for the *server*, it is important to take into account that Fast DDS also uses this
     parameter to identify participants in the same process and enable intra-process communications.
     Setting two DomainParticipant GUID prefixes as intra-process compatible will result in no communication if the
     DomainParticipants run in separate processes.
     For more information, please refer to :ref:`intraprocess_delivery_guids`.

.. warning::
    Launching more than one server using the same GUID prefix is undefined behavior.

Client side setup
"""""""""""""""""

Each *client* must keep a list of the *servers* to which it wants to link.
Each single element represents an individual server, and a |GuidPrefix_t-api| must be provided.
The *server* list must be populated with |RemoteServerAttributes-api| objects with a valid |GuidPrefix_t-api| data
member.
In XML the server list and its elements are simultaneously specified.
Note that ``prefix`` is an element of the ``RemoteServer`` tag.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_CLIENT_GUIDPREFIX                                                                     |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-CLIENT-PREFIX<-->                                                                  |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-19                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_locators:

The server locator list
^^^^^^^^^^^^^^^^^^^^^^^

Each *server* must specify valid locators where it can be reached.
Any *client* must be given proper locators to reach each of its *servers*.
As in the :ref:`above section <DS_guidPrefix>`, here there is a *server* and a *client* side setup.

Server side setup
"""""""""""""""""

The examples below show how to setup the server locator list and XML tag.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_LOCATORS                                                                       |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-SERVER-LOCATORS<-->                                                                |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-19                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

Note that a *server* can connect to other *servers*, thus, the following section may also apply.

Client side setup
"""""""""""""""""

Each *client* must keep a list of locators associated to the *servers* to which it wants to link.
Each *server* specifies its own locator list which must be populated with |RemoteServerAttributes-api| objects with a
valid ``metatrafficUnicastLocatorList`` or ``metatrafficMulticastLocatorList``.
In XML the server list and its elements are simultaneously specified.
Note the ``metatrafficUnicastLocatorList`` or ``metatrafficMulticastLocatorList`` are elements of the ``RemoteServer``
tag.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_CLIENT_LOCATORS                                                                       |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-CLIENT-LOCATORS<-->                                                                |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-25                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_ping_period:

Fine tuning discovery server handshake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As explained :ref:`above <DS_key_concepts>` the *clients* send discovery messages to the *servers* at regular
intervals (ping period) until they receive message reception acknowledgement.
Mind that this period also applies for those *servers* which connect to other *servers*.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_CLIENT_PING                                                                           |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-CLIENT-PING<-->                                                                    |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-16                                                                                                  |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+

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
     The list of remote *servers* can only be modified to either add more *servers*, or modify the remote server
     locator, but not to remove any of the existing ones.
     This means that the new list passed to |DomainParticipant::set_qos-api| must be a superset of the existing one.

.. note::
    The remote server list can also be modified using the ``ROS_DISCOVERY_SERVER`` environment variable.
    Please refer to :ref:`env_vars_fastdds_environment_file` for more information.

.. warning::
    It is strongly advised to use either the API or the environment file.
    Using both at the same time may cause undefined behavior.

+---------------------------------------------------------------------+
| **C++**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                      |
|    :language: c++                                                   |
|    :start-after: //CONF_SERVER_ADD_SERVERS                          |
|    :end-before: //!--                                               |
|    :dedent: 8                                                       |
|                                                                     |
+---------------------------------------------------------------------+

.. _DS_dns_name:

Configure Discovery Server locators using names
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All the examples provided in :ref:`discovery_server` use IPv4 addresses to specify the servers' listening locators.
However, *Fast DDS* also allows to :ref:`specify locator addresses using names <transport_transportApi_ipLocator>`.

.. _DS_full_example:

Full example
^^^^^^^^^^^^

The following constitutes a full example on how to configure *server* and *client* both programmatically and using XML.

Server side setup
"""""""""""""""""

+---------------------------------------------------------------------+
| **C++**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                      |
|    :language: c++                                                   |
|    :start-after: //CONF_SERVER_FULL_EXAMPLE                         |
|    :end-before: //!--                                               |
|    :dedent: 8                                                       |
|                                                                     |
+---------------------------------------------------------------------+
| **XML**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                          |
|    :language: xml                                                   |
|    :start-after: <!-->CONF_SERVER_FULL_EXAMPLE<-->                  |
|    :end-before: <!--><-->                                           |
|    :lines: 2-3,5-47                                                 |
|    :append: </profiles>                                             |
+---------------------------------------------------------------------+


Client side setup
"""""""""""""""""

+---------------------------------------------------------------------+
| **C++**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                      |
|    :language: c++                                                   |
|    :start-after: //CONF_CLIENT_FULL_EXAMPLE                         |
|    :end-before: //!--                                               |
|    :dedent: 8                                                       |
|                                                                     |
+---------------------------------------------------------------------+
| **XML**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                          |
|    :language: xml                                                   |
|    :start-after: <!-->CONF_CLIENT_FULL_EXAMPLE<-->                  |
|    :end-before: <!--><-->                                           |
|    :lines: 2-3,5-40                                                 |
|    :append: </profiles>                                             |
+---------------------------------------------------------------------+
