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

This feature is enabled by default, and can be configured using :ref:`xml_profiles`
(see :ref:`intra_process_delivery_xml_profile`).
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
|is_on_same_process_as-api| API is provided to check this condition.
This mechanism works out-of-the-box when letting Fast DDS set the GUID prefixes for the created DomainParticipants.
However, special consideration is required when setting the |GuidPrefix_t-api| manually, either programmatically or when
using XML.

.. important::

    Fast DDS assigns GUID prefixes considering several host parameters.
    Among them, the network interfaces enabled.
    Thus, if at runtime, the network interfaces change, any new DomainParticipant will have a different GUID prefix and
    will be considered to be running on another host.

.. tabs::

  .. tab:: **C++** - ASCII format.

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_GUIDPREFIX_OPTION_1
        :end-before: //!--
        :dedent: 8

  .. tab:: **C++** - Extraction operator.

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF_GUIDPREFIX_OPTION_2
        :end-before: //!--
        :dedent: 8

  .. tab:: **XML**

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-GUID-PREFIX<-->
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>
