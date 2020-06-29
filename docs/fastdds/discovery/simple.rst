.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _simple_disc_settings:

SIMPLE Discovery Settings
-------------------------

The SIMPLE discovery protocol resolves the establishment of the end-to-end connection between various DDS Entities.
*eProsima Fast DDS* implements the SIMPLE discovery protocol to provide compatibility with the
`RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_.
The specification splits up the SIMPLE discovery protocol into two independent protocols:

- **Simple Participant Discovery Protocol (SPDP):** specifies how DomainParticipants discover each other in the
  network; it announces and detects the presence of DomainParticipants within the same domain.

- **Simple Endpoint Discovery Protocol (SEDP):** defines the protocol adopted by the discovered DomainParticipants for
  the exchange of information in order to discover the DDS Entities contained in each of them, i.e. the |DataWriter| and
  |DataReader|.

+-----------------------------+----------------------------------------------------------------------------------------+
| Name                        | Description                                                                            |
+=============================+========================================================================================+
| `Initial Announcements`_    | It defines the behavior of the DomainParticipants initial announcements.               |
+-----------------------------+----------------------------------------------------------------------------------------+
| `Simple EDP Attributes`_    | It defines the use of the SIMPLE protocol as a discovery protocol.                     |
+-----------------------------+----------------------------------------------------------------------------------------+
| :ref:`Simple Initial Peers` | A list of DomainParticipant's IP/port pairs to which the SPDP announcements            |
|                             | are sent.                                                                              |
+-----------------------------+----------------------------------------------------------------------------------------+

.. _`Initial Announcements`:

Initial Announcements
^^^^^^^^^^^^^^^^^^^^^

`RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ simple discovery mechanism requires the
DomainParticipants to send announcements of their presence in the domain.
These announcements are not delivered in a reliable fashion, and can be disposed of by the network.
In order to avoid the discovery delay induced by message disposal, the initial announcement can be set up to make
several shots, in order to increase proper reception chances.
See |InitialAnnouncementConfig-api|.

Initial announcements only take place upon participant creation.
Once this phase is over, the only announcements enforced are the standard ones based on the
|leaseDuration_announcementperiod| period (not the |InitialAnnouncementConfig::period-api|).

+---------------------------+-------------------------------------------------------------+------------------+---------+
| Name                      | Description                                                 | Type             | Default |
+===========================+=============================================================+==================+=========+
| count                     | It defines the number of announcements to send at start-up. | ``uint32_t``     | 5       |
+---------------------------+-------------------------------------------------------------+------------------+---------+
| period                    | It defines the specific period for initial announcements.   | |Duration_t-api| | 100ms   |
+---------------------------+-------------------------------------------------------------+------------------+---------+

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT                                                             |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT<-->                                                      |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _`Simple EDP Attributes`:

Simple EDP Attributes
^^^^^^^^^^^^^^^^^^^^^

+-----------------------------+----------------------------------------------------------------+-------------+---------+
| Name                        | Description                                                    | Type        | Default |
+=============================+================================================================+=============+=========+
| SIMPLE EDP                  | It defines the use of the SIMPLE protocol as a discovery |br|  | ``bool``    | true    |
|                             | protocol for EDP phase. A DomainParticipant may create |br|    |             |         |
|                             | DataWriters, DataReaders, both or neither.                     |             |         |
+-----------------------------+----------------------------------------------------------------+-------------+---------+
| Publication writer and |br| | It is intended for DomainParticipants that implement only |br| | ``bool``    | true    |
| Subscription reader         | one or more DataWriters, i.e. do not implement DataReaders.    |             |         |
|                             | |br| It allows the creation of only DataReader discovery       |             |         |
|                             | related EDP endpoints.                                         |             |         |
+-----------------------------+----------------------------------------------------------------+-------------+---------+
| Publication reader and |br| | It is intended for DomainParticipants that implement only |br| | ``bool``    | true    |
| Subscription writer         | one or more DataReaders, i.e. do not implement DataWriters.    |             |         |
|                             | |br| It allows the creation of only DataWriter discovery       |             |         |
|                             | related |br| EDP endpoints.                                    |             |         |
+-----------------------------+----------------------------------------------------------------+-------------+---------+

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF-QOS-DISCOVERY-EDP-ATTRIBUTES                                                                 |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-QOS-DISCOVERY-EDP-ATTRIBUTES                                                              |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

 .. _`Simple Initial Peers`:

Initial peers
^^^^^^^^^^^^^

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each
|RTPSParticipant-api|
must listen for incoming Participant Discovery Protocol (PDP) discovery metatraffic in two different ports, one linked
with a multicast address, and another one linked to a unicast address.
*Fast DDS* allows for the configuration of an initial peers list which contains one or more such IP-port address
pairs corresponding to remote DomainParticipants PDP discovery listening resources, so that the local
DomainParticipant will not only send its PDP traffic to the default multicast address-port specified by its domain,
but also to all the IP-port address pairs specified in the initial peers list.

A DomainParticipant's initial peers list contains the list of IP-port address pairs of all other DomainParticipants
with which it will communicate.
It is a list of addresses that a DomainParticipant will use in the unicast discovery mechanism, together or as an
alternative to multicast discovery.
Therefore, this approach also applies to those scenarios in which multicast functionality is not available.

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the RTPSParticipants'
discovery traffic unicast listening ports are calculated using the following equation:
7400 + 250 * `domainID` + 10 + 2 * `participantID`.
Thus, if for example a RTPSParticipant operates in Domain 0 (default
domain) and its ID is 1, its discovery traffic unicast listening port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.
By default *eProsima Fast DDS* uses as initial peers the Metatraffic Multicast Locators.

The following constitutes an example configuring an Initial Peers list with one peer on host 192.168.10.13 with
DomainParticipant ID 1 in domain 0.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_INITIAL_PEERS_BASIC                                                                          |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_INITIAL_PEERS_BASIC<-->                                                                   |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+
