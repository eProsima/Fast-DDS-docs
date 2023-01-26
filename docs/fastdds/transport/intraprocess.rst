.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _intraprocess-delivery:

Intra-process delivery
**********************

eProsima Fast DDS allows to speed up communications between entities within the same process by avoiding any of the
overhead involved in the transport layer.
Instead, the :ref:`dds_layer_publisher_publisher` directly calls the reception functions of the
:ref:`dds_layer_subscriber_subscriber`.
This not only avoids the copy or send operations of the transport, but also ensures the message is received by the
Subscriber, avoiding the acknowledgement mechanism.

This feature is enabled by default, and can be configured using :ref:`xml_profiles`.
Currently the following options are available:

* **INTRAPROCESS_OFF**: The feature is disabled.
* **INTRAPROCESS_USER_DATA_ONLY**: Discovery metadata keeps using ordinary transport.
* **INTRAPROCESS_FULL**: Default value. Both user data and discovery metadata using Intra-process delivery.

+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-LIBRARY-SETTINGS         |
|    :end-before: <!--><-->                           |
|    :dedent: 4                                       |
+-----------------------------------------------------+

.. _intraprocess_delivery_guids:

GUID Prefix considerations for intra-process delivery
------------------------------------------------------

Fast DDS utilizes the |DomainParticipant|'s |GuidPrefix_t-api| to identify peers running in the same process.
Two participants with identical 8 first bytes on the |GuidPrefix_t-api| are considered to be running in the same
process, and therefore intra-process delivery is used.
This mechanism works out-of-the-box when letting Fast DDS set the GUID prefixes for the created DomainParticipants.
However, special consideration is required when setting the |GuidPrefix_t-api| manually, either programmatically or when
using XML

+----------------------------------------------------------------------------------------------------------------------+
| **C++** - Option 1: Manual setting of the ``unsigned char`` in ASCII format.                                         |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_GUIDPREFIX_OPTION_1                                                                          |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **C++** - Option 2: Using the ``>>`` operator and the ``std::istringstream`` type.                                   |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_GUIDPREFIX_OPTION_2                                                                          |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-GUID-PREFIX<-->                                                                           |
|    :end-before: <!--><-->                                                                                            |
|    :lines: 2-3,5-                                                                                                    |
|    :append: </profiles>                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
