.. _transport_disableMulticast:

Disabling all Multicast Traffic
===============================

It is possible to disable all multicast traffic on the :ref:`dds_layer_domainParticipant`,
following the following example:


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



