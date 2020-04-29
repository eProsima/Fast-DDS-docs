.. _transports_tips:

Tips
----

**Disabling all multicast traffic**

+-----------------------------------------------+
| **C++**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp   |
|    :language: c++                             |
|    :start-after: //CONF-DISABLE-MULTICAST     |
|    :end-before: //!--                         |
+-----------------------------------------------+
| **XML**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml    |
|    :language: xml                             |
|    :start-after: <!-->CONF-DISABLE-MULTICAST  |
|    :end-before: <!--><-->                     |
+-----------------------------------------------+

**Non-blocking write on sockets**

For UDP transport, it is possible to configure whether to use non-blocking write calls on the sockets.

+-----------------------------------------------+
| **C++**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp   |
|    :language: c++                             |
|    :start-after: //CONF-NON-BLOCKING-WRITE    |
|    :end-before: //!--                         |
+-----------------------------------------------+
| **XML**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml    |
|    :language: xml                             |
|    :start-after: <!-->CONF-NON-BLOCKING-WRITE |
|    :end-before: <!--><-->                     |
+-----------------------------------------------+

**XML Configuration**

The :ref:`xml_profiles` section contains the full information about how to setup *Fast RTPS* through an
*XML file*.
