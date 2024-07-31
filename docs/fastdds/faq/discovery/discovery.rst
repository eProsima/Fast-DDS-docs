.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_discovery_questions:

Discovery Frequently Asked Questions
====================================


.. collapse::  What are the two main phases involved in the discovery process of Fast DDS?




    :Answer:

    The two main phases involved in the discovery process of Fast DDS are the **Participant Discovery Phase (PDP)** and the **Endpoint Discovery Phase (EDP)**.*
    The **Participant Discovery Phase (PDP)** involves **DomainParticipants** recognizing each other by sending periodic announcement messages with their unicast addresses. Matching occurs when they are in the same DDS Domain, using multicast by default, though unicast and announcement frequency can be customized. In the **Endpoint Discovery Phase (EDP)**, **DataWriters** and **DataReaders** acknowledge each other by sharing information about topics and data types over the established channels. Matching endpoints with the same topic and data type are then ready to exchange user data.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What discovery mechanisms does FastDDS provide?




    :Answer:

    There are four discovery mechanisms in DDS: **Simple Discovery**, which follows the RTPS standard for both PDP and EDP, ensuring compatibility with other DDS implementations; **Static Discovery**, which uses the **Simple Participant Discovery Protocol (SPDP)** but skips the Endpoint Discovery phase if endpoint details are pre-known; **Discovery Server**, which employs a centralized server for meta traffic discovery; and **Manual Discovery**, which disables the PDP and requires users to manually match RTPS participants and endpoints using external meta-information channels.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is an initial peer list?




    :Answer:

    An initial peer list contains one or more IP-port address pairs corresponding to remote DomainParticipants PDP discovery listening resources, so that the local DomainParticipant will not only send its PDP traffic to the default multicast address-port specified by its domain, but also to all the IP-port address pairs specified in the initial peers list.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  When could a static configuration of peers be used?




    :Answer:

    When all **DataWriters** and **DataReaders**, and their **Topics** and data types, are known beforehand, the **EDP** phase can be replaced with a static configuration of peers.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary difference between the Discovery Server mechanism and Simple discovery mechanism in terms of managing metatraffic?




    :Answer:

    The **Discovery Server** mechanism is based on a client-server discovery paradigm, the metatraffic is managed by one or several server **DomainParticipants**, as opposed to **simple discovery**, where metatraffic is exchanged using a message broadcast mechanism like an IP multicast protocol.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary function of a Discovery Server in the DDS architecture?




    :Answer:

    The role of the server is to redistribute its clients' discovery information to its known clients and servers.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary purpose of a "BACKUP" server in the Discovery Server mechanism?




    :Answer:

    A ``BACKUP`` server is a server that persists its discovery database into a file. This type of server can load the network graph from a file on start-up without the need of receiving any client's information.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is a client in this context?




    :Answer:

    A ``CLIENT`` is a participant that connects to one or more servers from which it receives only the discovery information they require to establish communication with matching endpoints.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the difference between a CLIENT and a SUPER_CLIENT?




    :Answer:

    A ``SUPER_CLIENT`` is a client that receives the discovery information known by the server, in opposition to clients, which only receive the information they need.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of each server specifying its own locator list in the context of discovery configuration?




    :Answer:

    Each client must keep a list of locators associated to the servers to which it wants to link. Each server specifies its own locator list which must be populated with ``RemoteServerAttributes`` objects with a valid ``metatrafficUnicastLocatorList`` or ``metatrafficMulticastLocatorList``. In XML the server list and its elements are simultaneously specified.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the typical interval of time between discovery messages sent by clients to servers, as described in the text?




    :Answer:

    As explained above the clients send discovery messages to the servers at regular intervals (ping period) until they receive message reception acknowledgement. The default value for this period is 450 ms.

|
