.. include:: includes/aliases.rst

.. _discovery_server:

Server-Client Discovery Settings
--------------------------------

This mechanism is based on a client-server discovery paradigm, i.e. the metatraffic (message exchange among
DomainParticipants to identify each other) is managed by one or several server DomainParticipants (left figure), as
opposed to simple discovery (right figure), where metatraffic is exchanged using a message broadcast mechanism like an
IP multicast protocol.
A `Discovery-Server <https://eprosima-discovery-server.readthedocs.io/en/latest/index.html>`_ tool is available to
ease client-server setup and testing.

.. figure:: /01-figures/fast_dds/discovery/discovery-server.svg
    :align: center
    :width: 50%

    Comparison of Server-Client discovery and Simple discovery mechanisms

.. _DS_key_concepts:


Key concepts
^^^^^^^^^^^^

In this architecture there are several key concepts to understand:

- The Server-Client discovery mechanism reuses the RTPS discovery messages structure, as well as the standard DDS
  DataWriters and DataReaders.

- Discovery server DomainParticipants may be *clients* or *servers*.
  The only difference between them is how they handle meta-traffic.
  The user traffic, that is, the traffic among the DataWriters and DataReaders they create is role-independent.

- All *server* and *client* discovery information will be shared with linked *clients*.
  Note that a *server* may act as a *client* for other *servers*.

- *Clients* require a beforehand knowledge of the *servers* to which they want to link.
  Basically it is reduced to the *server* identity (henceforth called ``GuidPrefix``) and a list of locators where the
  *server* is listening.
  These locators also define the transport protocol (UDP or TCP) the client will use to contact the *server*.

  - The ``GuidPrefix`` is the RTPS standard RTPSParticipant unique identifier, a 12-byte chain.
    This identifier allows clients to assess whether they are receiving messages from the right server, as each
    standard RTPS message contains this piece of information.

    The ``GuidPrefix`` is used because the server's IP address may not be a reliable enough server identifier,
    since several servers can be hosted in the same machine, thus having the same IP, and also because multicast
    addresses are acceptable addresses.

- *Servers* do not require any beforehand knowledge of their *clients*, but their ``GuidPrefix`` and locator list (where
  they are listening) must match the one provided to the *clients*.
  In order to gather *client* discovery information the following handshake strategy is followed:

  - *Clients* send discovery messages to the *servers* at regular intervals (ping period) until they receive message
    reception acknowledgement.

  - *Servers* receive discovery messages from the clients, but they do not start processing them until a time interval
    has elapsed, which starts at the moment the server is instantiated.

In order to clarify this discovery setup, either on compile time (sources) or runtime (XML files), its explanation is
divided into two sections: focusing on the main concepts (:ref:`setup by concept <DS_setup_concepts>`), and on
the main setting structures and XML tags (:ref:`setup by QoS <DS_setup_attributes>`).

.. _DS_setup_concepts:

Server-Client setup by concept
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. csv-table::
    :header: "Concept", "Description"

    :ref:`Discovery protocol <DS_discovery_protocol>`, Make a participant a *client* or a *server*.
    :ref:`Server unique id <DS_guidPrefx>`, Link a *clients* to *servers*.
    :ref:`Seting up transport <DS_locators>`, Specify which transport to use and make *servers* reachable.
    :ref:`Pinging period <DS_ping_period>`, Fine tune server-client handshake.
    :ref:`Matching period <DS_match_period>`, Fine tune server deliver efficiency.

.. _DS_discovery_protocol:

Choosing between Client and Server
""""""""""""""""""""""""""""""""""

It is set by the :ref:`Discovery Protocol <discovery_protocol>` general setting. A participant can only play a role
(despite the fact that a *server* may act as a *client* of other server). It's mandatory to fill this value because it
defaults to *simple*.  The values associated with the Server-Client discovery are specified in :ref:`discovery settings
section <DS_DiscoverySettings>`. The examples below show how to manage the corresponding enum and XML tag.

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
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_guidPrefx:

The GuidPrefix as the server unique identifier
""""""""""""""""""""""""""""""""""""""""""""""

The ``GuidPrefix`` attribute belongs to the RTPS specification and univocally identifies each RTPSParticipant.
It consists on 12 bytes and in Fast DDS is a key for the DomainParticipant used in the DDS domain.
Fast DDS defines the DomainParticipant ``GuidPrefix`` as a public data member of the
|DomainParticipantQosWireProtocolClass| class.
In the Server-Client discovery, it has the purpose to link a *server* to its *clients*.
It must be mandatorily specified in: *server* and *client* setups.

Server side setup
*****************

The examples below show how to manage the corresponding enum data member and XML tag.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_GUIDPREFIX                                                                     |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-SERVER-PREFIX<-->                                                                  |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

Note that a *server* can act as a *client* of other *servers*.
Thus, the following section may also apply.

Client side setup
*****************

Each *client* must keep a list of the *servers* to which it wants to link.
Each single element represents an individual server and a ``GuidPrefix`` must be provided.
The *server* list must be populated with ``RemoteServerAttributes`` objects with a valid ``guidPrefix`` data member.
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
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_locators:

The server locator list
"""""""""""""""""""""""

Each *server* must specify valid locators where it can be reached.
Any *client* must be given proper locators to reach each of its *servers*.
As in the :ref:`above section <DS_guidPrefx>`, here there is a *server* and a *client* side setup.

Server side setup
*****************

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
+----------------------------------------------------------------------------------------------------------------------+

Note that a *server* can act as a client of other *servers*, thus, the following section may also apply.

Client side setup
*****************

Each *client* must keep a list of locators associated to the *servers* to which it wants to link.
Each *server* specifies its own locator list and must be populated with ``RemoteServerAttributes`` objects with a
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
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_ping_period:

Client ping period
""""""""""""""""""

As explained :ref:`above <DS_key_concepts>` the *clients* send discovery messages to the *servers* at regular
intervals (ping period) until they receive message reception acknowledgement.

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
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_match_period:

Server match period
"""""""""""""""""""

As explained :ref:`above <DS_key_concepts>`, the *servers* receive discovery messages from new clients to join the
communication.
However, the *servers* do not start processing them until a time interval, defined by this period, has
elapsed, which starts at the moment the server is instantiated.
Therefore, this member specifies a time interval in which the server's DataReader is disabled and incoming messages
are not processed.
It is a time interval intended to allow the server to initialize its resources.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_PING                                                                           |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-SERVER-PING<-->                                                                    |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_setup_attributes:

Server-Client setup by Qos
^^^^^^^^^^^^^^^^^^^^^^^^^^

The settings related with server-client discovery are:

.. csv-table::
    :header: "Name", "Description"

    :ref:`WireProtocolConfigQos <DS_WireProtocolConfigQos>` |br| (|DomainParticipantQosWireProtocol|), "
    Specifies wire protocol settings for a DomainParticipant. |br|
    Some of it data members must be modified in order to properly configure a Server. |br|
    An example is the |DomainParticipantQosWireProtocolPrefix| data member."
    :ref:`RTPS BuiltinAttributes <DS_BuiltinAttributes>` |br| (|DomainParticipantQosWireProtocolBuiltin|), "
    It is a public data member of the above |DomainParticipantQosWireProtocolClass| class. |br|
    Allows to specify some mandatory server discovery settings like the addresses were it |br|
    listens for clients discovery information."
    :ref:`DiscoverySettings <DS_DiscoverySettings>`, "
    It is a member of the above :class:`BuiltinAttributes` structure. |br|
    Allows to specify some mandatory and optional Server-Client discovery settings such as |br|
    whether the DomainParticipant is a client or a server, the list of servers it is linked to, |br|
    the client-ping, and the server-match frequencies."

.. _DS_WireProtocolConfigQos:

WireProtocolConfigQos
"""""""""""""""""""""

The |DomainParticipantQosWireProtocolPrefix| data member of the |DomainParticipantQosWireProtocol| class
specifies the server's identity.
This member has only significance if ``discovery_config.discoveryProtocol`` is **SERVER** or **BACKUP**.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_SERVER_GUIDPREFIX                                                                     |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-CLIENT-PREFIX                                                                      |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_BuiltinAttributes:

RTPS BuiltinAttributes
""""""""""""""""""""""

All discovery related information is gathered in the :ref:`DS_DiscoverySettings` ``discovery_config`` data member.

In order to receive client metatraffic, ``metatrafficUnicastLocatorList`` or
``metatrafficMulticastLocatorList`` must be populated with the addresses (IP and port) that were given to
the clients.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_METATRAFFICUNICAST                                                                    |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-METATRAFFICUNICASTLOCATOR                                                          |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _DS_DiscoverySettings:

DiscoverySettings
"""""""""""""""""

The :class:`discoveryProtocol` enum data member (:ref:`discovery_protocol`) specifies the participant's discovery kind.
As was explained before, to setup the Server-Client discovery it may be:

.. csv-table::
    :header: "enum value", "Description"
    :widths: 15, 100

    CLIENT, "
    Generates a client DomainParticipant, which relies on a server (or servers) to be notified of other clients |br|
    presence. This DomainParticipant can create DataWriters and DataReaders of any topic (static or dynamic) as |br|
    ordinary DomainParticipants do."
    SERVER, "
    Generates a server DomainParticipant, which receives, manages and spreads its matched client's metatraffic |br|
    assuring any single one is aware of the others. This DomainParticipant can create DataWriters and |br|
    DataReaders of any topic (static or dynamic) as ordinary DomainParticipants do. |br|
    Servers can link to other servers in order to share its clients information."
    BACKUP, "
    Generates a server DomainParticipant with additional functionality over **SERVER**. Specifically, it uses a |br|
    database to backup its client information, so that this information can be automatically restored at any |br|
    moment and continue spreading metatraffic to late joiners. A **SERVER** in the same scenario ought to |br|
    collect client information again, introducing a recovery delay."

A :class:`DiscoveryServers` that lists the servers linked to a client DomainParticipant. This member
has only significance if :ref:`discovery_protocol` is **CLIENT**, **SERVER** or **BACKUP**.
These member elements are :class:`RemoteServerAttributes` objects that identify each server and report where the
servers can be reached:

.. list-table::
   :header-rows: 1

   * - Data members
     - Description
   * - ``GuidPrefix_t guidPrefix``
     - Is the RTPS unique identifier of the remote server DomainParticipant.
   * - ``metatrafficUnicastLocatorList`` |br| ``metatrafficMulticastLocatorList``
     - Are ordinary ``LocatorList_t`` (see :ref:`LocatorListType`) where the server's |br|
       locators must be specified. At least one of them should be populated.
   * - ``Duration_t discoveryServer_client_syncperiod``
     - Has only significance if :ref:`discovery_protocol` is **CLIENT**, **SERVER** or |br|
       **BACKUP**. For a *client* it specifies the pinging period as explained in |br|
       :ref:`key concepts <DS_key_concepts>`. When a client has not yet established a |br|
       reliable connection to a server it *pings* until the server notices him and |br|
       establishes the connection. |br|
       For a *server* it specifies the match period as explained in :ref:`key concepts <DS_key_concepts>`. |br|
       When a *server* discovers new *clients* it only starts exchanging information with them |br|
       at regular intervals as a mechanism to bundle discovery information and optimize delivery. |br|
       The default value is half a second.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_SERVER_PING                                                                                  |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-SERVER-PING                                                                               |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+


