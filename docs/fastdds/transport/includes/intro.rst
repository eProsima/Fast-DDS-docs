*eProsima Fast DDS* implements an architecture of pluggable transports.
Current version implements five transports: UDPv4, UDPv6, TCPv4, TCPv6 and SHM (shared memory).
By default, when a :class:`Participant` is created, one built-in UDPv4 transport is configured.

You can add custom transports using the attribute ``rtps.userTransports``.

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp         |
|    :language: c++                                   |
|    :start-after: //CONF-COMMON-TRANSPORT-SETTING    |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-COMMON-TRANSPORT-SETTING |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+

All Transport configuration options can be found in the section :ref:`transportdescriptors`.
