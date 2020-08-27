.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _whitelist-interfaces:

Interface Whitelist
-------------------

Using *Fast DDS*, it is possible to limit the network interfaces used by :ref:`transport_tcp_tcp` and
:ref:`transport_udp_udp`.
This is achieved by adding the interfaces' IP addresses to the ``interfaceWhiteList``
field in the :ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
Thus, the communication interfaces used by the |DomainParticipants| whose :class:`TransportDescriptor` defines an
``interfaceWhiteList`` is limited to the interfaces' IP addresses defined in that list,
therefore avoiding the use of the rest of the network interfaces available in the system.
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

