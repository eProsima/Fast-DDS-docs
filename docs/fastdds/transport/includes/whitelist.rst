.. _whitelist-interfaces:

Whitelist Interfaces
--------------------

There could be situations where you want to block some network interfaces to avoid connections or sending data through
them.
This can be managed using the field *interface whitelist* in the transport descriptors, and with them, you can set the
interfaces you want to use to send or receive packets.
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
