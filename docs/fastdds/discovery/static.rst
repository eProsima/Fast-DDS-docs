.. _discovery_static:

STATIC Discovery Settings
-------------------------

Fast DDS allows for the substitution of the SEDP protocol for the EDP phase with a static version that completely
eliminates EDP meta traffic.
This can become useful when dealing with limited network bandwidth and a well-known schema of DataWriters and
DataReaders.
If all DataWriters and DataReaders, and their Topics and data types, are known beforehand, the EDP phase can be replaced
with a static configuration of peers.
It is important to note that by doing this, no EDP discovery meta traffic will be generated, and only those peers
defined in the configuration will be able to communicate.
The STATIC endpoint discovery related settings are:

+------------------------------+-----------------------------------------------------------------------------------+
| Name                         | Description                                                                       |
+==============================+===================================================================================+
| :ref:`static_edp`            | It activates the STATIC endpoint discovery protocol                               |
+------------------------------+-----------------------------------------------------------------------------------+
| :ref:`static_xml`            | Specifies an XML file containing a description of the remote endpoints.           |
+------------------------------+-----------------------------------------------------------------------------------+
| :ref:`Initial Announcements` | It defines the behavior of the RTPSParticipant initial announcements (PDP phase). |
+------------------------------+-----------------------------------------------------------------------------------+

.. _static_edp:

STATIC EDP
^^^^^^^^^^

To activate the STATIC EDP, the SEDP must be disabled on the participant attributes.
This can be done either by code or using an XML configuration file:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                                                                          |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_STATIC_DISCOVERY_CODE                                                                        |
|    :end-before: //!                                                                                                  |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_STATIC_DISCOVERY_CODE                                                                     |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _`static_xml`:

STATIC EDP XML Files Specification
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Since activating STATIC EDP suppresses all EDP meta traffic, the information about the remote entities (publishers and
subscribers) must be statically specified, which is done using dedicated XML files.
A participant may load several of such configuration files so that the information about different endpoints can be
contained in one file, or split into different files to keep it more organized.
Fast-RTPS  provides a
`Static Endpoint Discovery example <https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/StaticHelloWorldExample>`_
that implements this EDP discovery protocol.

The following table describes all the possible attributes of a STATIC EDP XML configuration file.
A full example of such file can be found in :ref:`static_xml_example`.

.. Some large words outside of table. Then table fit maximum line length

.. |besteffort| replace:: :class:`BEST_EFFORT_RELIABILITY_QOS`
.. |reliable| replace:: :class:`RELIABLE_RELIABILITY_QOS`
.. |volatile| replace:: :class:`VOLATILE_DURABILITY_QOS`
.. |transientlocal| replace:: :class:`TRANSIENT_LOCAL_DURABILITY_QOS`
.. |transient| replace:: :class:`TRANSIENT_DURABILITY_QOS`

+------------------------+-------------------------------------------------------+-------------------+-----------------+
| Name                   | Description                                           | Values            | Default         |
+========================+=======================================================+===================+=================+
| ``<userId>``           | Mandatory.                                            | ``uint16_t``      | 0               |
|                        | Uniquely identifies the endpoint.                     |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<entityID>``         | EntityId of the endpoint.                             | ``uint16_t``      | 0               |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<expectsInlineQos>`` | It indicates if QOS is                                | ``bool``          | ``false``       |
|                        | expected inline.                                      |                   |                 |
|                        | (reader **only**)                                     |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<topicName>``        | Mandatory.                                            | ``string_255``    |                 |
|                        | The topic of the remote endpoint.                     |                   |                 |
|                        | Should match with one of the                          |                   |                 |
|                        | topics of the local participant.                      |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<topicDataType>``    | Mandatory.                                            | ``string_255``    |                 |
|                        | The data type of the topic.                           |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<topicKind>``        | The kind of topic.                                    | :class:`NO_KEY`   | :class:`NO_KEY` |
|                        |                                                       +-------------------+                 |
|                        |                                                       | :class:`WITH_KEY` |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<partitionQos>``     | The name of a partition of the                        | ``string``        |                 |
|                        | remote peer. Repeat to configure                      |                   |                 |
|                        | several partitions.                                   |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<unicastLocator>``   | Unicast locator of the                                |                   |                 |
|                        | participant.                                          |                   |                 |
|                        | See :ref:`staticLocators`.                            |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<multicastLocator>`` | Multicast locator of the                              |                   |                 |
|                        | participant.                                          |                   |                 |
|                        | See :ref:`staticLocators`.                            |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<reliabilityQos>``   | See the :ref:`reliability`                            | |besteffort|      | |besteffort|    |
|                        | section.                                              +-------------------+                 |
|                        |                                                       | |reliable|        |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<durabilityQos>``    | See the                                               | |volatile|        | |volatile|      |
|                        | :ref:`SettingDataDurability`                          +-------------------+                 |
|                        | section.                                              | |transientlocal|  |                 |
|                        |                                                       +-------------------+                 |
|                        |                                                       | |transient|       |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<ownershipQos>``     | See                                                   |                   |                 |
|                        | :ref:`ownershipQos`.                                  |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+
| ``<livelinessQos>``    | Defines the liveliness of the                         |                   |                 |
|                        | remote peer.                                          |                   |                 |
|                        | See :ref:`livelinessQos`.                             |                   |                 |
+------------------------+-------------------------------------------------------+-------------------+-----------------+

.. _staticLocators:

Locators definition
"""""""""""""""""""

Locators for remote peers are configured using ``<unicastLocator>`` and ``<multicastLocator>`` tags.
These take no value, and the locators are defined using tag attributes.
Locators defined with ``<unicastLocator>`` and ``<multicastLocator>`` are accumulative, so they can be repeated to
assign several remote endpoints locators to the same peer.

* :class:`address`: a mandatory ``string`` representing the locator address.
* :class:`port`: an optional ``uint16_t`` representing a port on that address.

.. _ownershipQos:

Ownership QoS
"""""""""""""

The ownership of the topic can be configured using ``<ownershipQos>`` tag.
It takes no value, and the configuration is done using tag attributes:

* :class:`kind`: can be one of :class:`SHARED_OWNERSHIP_QOS` or :class:`EXCLUSIVE_OWNERSHIP_QOS`.
  This attribute is mandatory withing the tag.

* :class:`strength`: an optional ``uint32_t`` specifying how strongly the remote participant owns the topic.
  This attribute can be set on writers **only**.
  If not specified, default value is zero.

.. _livelinessQos:

Liveliness QoS
""""""""""""""

The :ref:`livelinessqospolicy` of the remote peer is configured using ``<livelinessQos>`` tag.
It takes no value, and the configuration is done using tag attributes:

* :class:`kind`: can be any of :class:`AUTOMATIC_LIVELINESS_QOS`, :class:`MANUAL_BY_PARTICIPANT_LIVELINESS_QOS` or
  :class:`MANUAL_BY_TOPIC_LIVELINESS_QOS`. This attribute is mandatory withing the tag.

* :class:`leaseDuration_ms`: an optional ``UInt32`` specifying the lease duration for the remote peer.
  The special value :class:`INF` can be used to indicate infinite lease duration.
  If not specified, default value is :class:`INF`

.. _static_xml_example:

STATIC EDP XML Example
""""""""""""""""""""""

The following is a complete example of a configuration XML file for two remote participants, a publisher and a
subscriber.
This configuration **must** agree with the configuration used to create the remote endpoint.
Otherwise, communication between endpoints may be affected.
If any non-mandatory element is missing, it will take the default value.
As a rule of thumb, all the elements that were specified on the remote endpoint creation should be configured.

+-------------------------------------------------+
| **XML**                                         |
+-------------------------------------------------+
| .. literalinclude:: /../code/StaticTester.xml   |
|    :language: xml                               |
|    :start-after: <!-->STATIC_DISCOVERY_CONF<--> |
|    :end-before: <!--><-->                       |
+-------------------------------------------------+

.. _`static_xml_load`:

Loading STATIC EDP XML Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Statically discovered remote endpoints **must** define a unique *userID* on their profile, whose value **must** agree
with the one specified in the discovery configuration XML.
This is done by setting the user ID on the entity attributes:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                                                                          |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_QOS_STATIC_DISCOVERY_USERID                                                                  |
|    :end-before: //!                                                                                                  |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_QOS_STATIC_DISCOVERY_USERID                                                               |
|    :end-before: <!-->                                                                                                |
+----------------------------------------------------------------------------------------------------------------------+

On the local participant, loading STATIC EDP configuration files is done by:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp                                                                          |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_STATIC_DISCOVERY_XML                                                                         |
|    :end-before: //!                                                                                                  |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_STATIC_DISCOVERY_XML                                                                      |
|    :end-before: <!-->                                                                                                |
+----------------------------------------------------------------------------------------------------------------------+

