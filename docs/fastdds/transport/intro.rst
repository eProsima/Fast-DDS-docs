

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
