.. Como generar los ficheros PEM

.. _security-configuration-examples:

Example: configuring the DomainParticipant
---------------------------------------------

This example show you how to configure a Participant to activate and configure :ref:`auth-pki-dh`,
:ref:`access-permissions` and :ref:`crypto-aes-gcm-gmac` plugins.

**Participant attributes**

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //SECURITY_CONF_ALL_PLUGINS     |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->SECURITY_CONF_ALL_PLUGINS  |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

This example shows you how to configure a Participant to activate and configure :ref:`auth-pki-dh` and
:ref:`crypto-aes-gcm-gmac` plugins, without and Access control plugin.
It also configures Participant to encrypt its RTPS messages, Writer and Reader to encrypt their RTPS submessages and
a writer to encrypt the payload (user data).

**Participant attributes**

+------------------------------------------------------------+
| **C++**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                |
|    :language: c++                                          |
|    :start-after: //SECURITY_CONF_AUTH_AND_CRYPT_PLUGINS    |
|    :end-before: //!--                                      |
+------------------------------------------------------------+
| **XML**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                 |
|    :language: xml                                          |
|    :start-after: <!-->SECURITY_CONF_AUTH_AND_CRYPT_PLUGINS |
|    :end-before: <!--><-->                                  |
+------------------------------------------------------------+

**Publisher attributes**

+----------------------------------------------------------------------+
| **C++**                                                              |
+----------------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                          |
|    :language: c++                                                    |
|    :start-after: //SECURITY_CONF_PUBLISHER_AUTH_AND_CRYPT_PLUGINS    |
|    :end-before: //!--                                                |
+----------------------------------------------------------------------+
| **XML**                                                              |
+----------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                           |
|    :language: xml                                                    |
|    :start-after: <!-->SECURITY_CONF_PUBLISHER_AUTH_AND_CRYPT_PLUGINS |
|    :end-before: <!--><-->                                            |
+----------------------------------------------------------------------+

**Subscriber attributes**

+-----------------------------------------------------------------------+
| **C++**                                                               |
+-----------------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                           |
|    :language: c++                                                     |
|    :start-after: //SECURITY_CONF_SUBSCRIBER_AUTH_AND_CRYPT_PLUGINS    |
|    :end-before: //!--                                                 |
+-----------------------------------------------------------------------+
| **XML**                                                               |
+-----------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                            |
|    :language: xml                                                     |
|    :start-after: <!-->SECURITY_CONF_SUBSCRIBER_AUTH_AND_CRYPT_PLUGINS |
|    :end-before: <!--><-->                                             |
+-----------------------------------------------------------------------+
