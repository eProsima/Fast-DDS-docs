.. _simple_disc_settings:

SIMPLE Discovery Settings
-------------------------

The SIMPLE discovery protocol resolves the establishment of the end-to-end connection between various RTPS entities
communicating via the RTPS protocol.
Fast-RTPS implements the SIMPLE discovery protocol to provide compatibility with the
`RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_.
The specification splits up the SIMPLE discovery protocol into two independent protocols:

- **Simple Participant Discovery Protocol (SPDP):** specifies how Participants discover each other in the network; it
  announces and detects the presence of participants in a domain.

- **Simple Endpoint Discovery Protocol (SEDP):** defines the protocol adopted by the discovered participants for the
  exchange of information in order to discover the RTPS entities contained in each of them, i.e. the writer and
  reader Endpoints.

+------------------------------+-----------------------------------------------------------------------+
| Name                         | Description                                                           |
+==============================+=======================================================================+
| `Initial Announcements`_     | It defines the behavior of the RTPSParticipant initial announcements. |
+------------------------------+-----------------------------------------------------------------------+
| `Simple EDP Attributes`_     | It defines the use of the SIMPLE protocol as a discovery protocol.    |
+------------------------------+-----------------------------------------------------------------------+
| :ref:`Simple Initial Peers`  | A list of endpoints to which the SPDP announcements are sent.         |
+------------------------------+-----------------------------------------------------------------------+

.. _`Initial Announcements`:

Initial Announcements
^^^^^^^^^^^^^^^^^^^^^

`RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ simple discovery mechanism requires the participant to
send announcements. These announcements are not delivered in a reliable fashion, and can be disposed of by the network.
In order to avoid the discovery delay induced by message disposal, the initial announcement can be set up to make
several shots, in order to increase proper reception chances.

Initial announcements only take place upon participant creation. Once this phase is over, the only announcements
enforced are the standard ones based on the ``leaseDuration_announcementperiod`` period (not the
``initial_announcements.period``).

+---------+--------------------------------------------------------------------+----------------+---------+
| Name    | Description                                                        | Type           | Default |
+=========+====================================================================+================+=========+
| count   | It defines the number of announcements to send at start-up.        | ``uint32``     | 5       |
+---------+--------------------------------------------------------------------+----------------+---------+
| period  | It defines the specific period for initial announcements.          | ``Duration_t`` | 100ms   |
+---------+--------------------------------------------------------------------+----------------+---------+

+-----------------------------------------------------------------+
| **C++**                                                         |
+-----------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                     |
|    :language: c++                                               |
|    :start-after: //DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT        |
|    :end-before: //!--                                           |
+-----------------------------------------------------------------+
| **XML**                                                         |
+-----------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                      |
|    :language: xml                                               |
|    :start-after: <!-->DISCOVERY-CONFIG-INITIAL-ANNOUNCEMENT<--> |
|    :end-before: <!--><-->                                       |
+-----------------------------------------------------------------+

.. _`Simple EDP Attributes`:

Simple EDP Attributes
^^^^^^^^^^^^^^^^^^^^^

+----------------------------------------+------------------------------------------------------+----------+---------+
| Name                                   | Description                                          | Type     | Default |
+========================================+======================================================+==========+=========+
| SIMPLE EDP                             | It defines the use of the SIMPLE protocol as a       | ``bool`` | true    |
|                                        | discovery protocol for EDP phase.                    |          |         |
|                                        | A participant may create publishers, subscribers,    |          |         |
|                                        | both or neither.                                     |          |         |
+----------------------------------------+------------------------------------------------------+----------+---------+
| Publication writer and                 | It is intended for participants that                 | ``bool`` | true    |
| Subscription reader                    | implement only one or more publishers, i.e. do not   |          |         |
|                                        | implement subscribers.                               |          |         |
|                                        | It allows the creation of only subscriber discovery  |          |         |
|                                        | related EDP endpoints                                |          |         |
+----------------------------------------+------------------------------------------------------+----------+---------+
| Publication reader and                 | It is intended for participants that implement only  | ``bool`` | true    |
| Subscription writer                    | one or more subscribers, i.e. do not implement       |          |         |
|                                        | publishers.                                          |          |         |
|                                        | It allows the creation of only publisher discovery   |          |         |
|                                        | related EDP endpoints.                               |          |         |
+----------------------------------------+------------------------------------------------------+----------+---------+

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp             |
|    :language: c++                                       |
|    :start-after: //CONF-QOS-DISCOVERY-EDP-ATTRIBUTES    |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF-QOS-DISCOVERY-EDP-ATTRIBUTES |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

.. _`Simple Initial Peers`:

Initial Peers
^^^^^^^^^^^^^

By default, the SPDP protocol uses a well known multicast address for the participant discovery phase.
With Fast-RTPS, it is possible to expand the list of endpoints to which the participant announcements are sent by
configuring a list of initial peers, as explained in :ref:`initial-peers`.
