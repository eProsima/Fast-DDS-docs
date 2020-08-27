.. _whitelist-interfaces:

Interface Whitelist
-------------------

It is possible to define various network interfaces on the :ref:`transport_tcp_tcp`
or :ref:`transport_udp_udp` in order to force Fast DDS entities to use exclusively these network interfaces.
This is achieved by adding the IP address corresponding to the interface to the ``interfaceWhiteList``
field in the :ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
Thus, the communication of the entities that define an ``interfaceWhiteList`` is forced through the addresses that
are defined in it, avoiding the use of the rest of the network interfaces available in the system.
The values on this list should match the IPs of your machine in that networks.
For example:


+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp   |
|    :language: c++                                |
|    :start-after: //TRANSPORT-DESCRIPTORS         |
|    :end-before: //!--                            |
|    :dedent: 8                                    |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->TRANSPORT-DESCRIPTORS      |
|    :end-before: <!--><-->                        |
|    :lines: 2-3,5-                                |
|    :append: </profiles>                          |
+--------------------------------------------------+

.. warning::

  The interface whitelist feature applies to network interfaces.
  Therefore, it is only available on :ref:`transport_tcp_tcp` and :ref:`transport_udp_udp`.

