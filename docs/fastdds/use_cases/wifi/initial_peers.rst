.. _use-case-initial-peers:

Configuring Initial Peers
=========================

.. _RTPS v2.2 standard: https://www.omg.org/spec/DDSI-RTPS/2.2/

A complete description of the initial peers list and its configuration can be found in
:ref:`Simple Initial Peers`.
For convenience, this example shows how to configure an initial peers list with one peer
on host ``192.168.10.13`` with participant ID ``1`` in domain ``0``.

.. note::

    Note that the port number used here is not arbitrary, as discovery ports are defined by
    the `RTPS v2.2 standard`_.
    Refer to :ref:`listening_locators_defaultPorts` to learn about these standard port numbers.

    If the participant ID is not known, setting :ref:`transport_transportApi_transportDescriptor`
    ``maxInitialPeersRange`` to at least the maximum expected number of DomainParticipants will ensure discovery and
    communication.


+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp          |
|    :language: c++                                       |
|    :start-after: //CONF_INITIAL_PEERS_BASIC             |
|    :end-before: //!--                                   |
|    :dedent: 8                                           |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF_INITIAL_PEERS_BASIC<-->      |
|    :end-before: <!--><-->                               |
|    :lines: 2-3,5-                                       |
|    :append: </profiles>                                 |
+---------------------------------------------------------+


