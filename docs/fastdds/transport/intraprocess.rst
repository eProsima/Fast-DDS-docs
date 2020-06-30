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



