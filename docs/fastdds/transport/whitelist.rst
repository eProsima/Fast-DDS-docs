.. _whitelist-interfaces:

Interface Whitelist
-------------------

It is possible to block some network interfaces on the :ref:`transport_tcp_tcp`
or :ref:`transport_udp_udp` to avoid *Fast DDS* using them.
To block an interface, just add the IP address of assigned to the interface to the ``interfaceWhiteList``
field in the :ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
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

