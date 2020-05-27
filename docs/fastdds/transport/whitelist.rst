.. _whitelist-interfaces:

Whitelist Interfaces
--------------------

It is possible to block some network interfaces on the :ref:`transport_tcp_tcp`
or :ref:`transport_udp_udp` to avoid Fast DDS using them.
To block an interface, just add the IP address of assigned to the interface to the ``interfaceWhiteList``
field in the :ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
The values on this list should match the IPs of your machine in that networks.
For example:

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TRANSPORT-DESCRIPTORS    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TRANSPORT-DESCRIPTORS |
|    :lines: 1-8,48                                |
+--------------------------------------------------+

.. note::

  The interface whitelist feature applies to network interfaces.
  Therefore, it is only available on :ref:`transport_tcp_tcp` and :ref:`transport_udp_udp`.