.. _initial-peers:

Initial peers
-------------

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each participant must
listen for incoming Participant Discovery Protocol (PDP) discovery metatraffic in two different ports, one linked with a
multicast address, and another one linked to a unicast address (see :ref:`discovery`).
Fast-RTPS allows for the configuration of an initial peers list which contains one or more such address-port pairs
corresponding to remote participants PDP discovery listening resources, so that the local participant will not only
send its PDP traffic to the default multicast address-port specified by its domain, but also to all the address-port
pairs specified in the initial-peers list.

A participant's initial peers list contains the list of address-port pairs of all other participants with
which it will communicate.
It is a list of addresses that a participant will use in the unicast discovery mechanism, together or as an alternative
to multicast discovery.
Therefore, this approach also applies to those scenarios in which multicast functionality is not available.

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the participants'
discovery traffic unicast listening ports are calculated using the following equation:
7400 + 250 * `domainID` + 10 + 2 * `participantID`.
Thus, if for example a participant operates in Domain 0 (default
domain) and its ID is 1, its discovery traffic unicast listening port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.
By default *eProsima Fast RTPS* uses as initial peers the Metatraffic Multicast Locators.

The following constitutes an example configuring an Initial Peers list with one peer on host 192.168.10.13 with
participant ID 1 in domain 0.

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp             |
|    :language: c++                                       |
|    :start-after: //CONF_INITIAL_PEERS_BASIC             |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF_INITIAL_PEERS_BASIC<-->      |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

.. These locators are used to know where to send initial discovery network messages. You can set your own locators using
.. attribute ``rtps.builtin.initialPeersList``.
