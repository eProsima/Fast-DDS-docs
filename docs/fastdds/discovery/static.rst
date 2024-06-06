.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _discovery_static:

STATIC Discovery Settings
-------------------------

*Fast DDS* allows for the substitution of the SEDP protocol for the EDP phase with a static version that completely
eliminates EDP meta traffic.
This can become useful when dealing with limited network bandwidth and a well-known schema of |DataWriters| and
|DataReaders|.
If all DataWriters and DataReaders, and their |Topics| and data types, are known beforehand, the EDP phase can be
replaced with a static configuration of peers.
It is important to note that by doing this, no EDP discovery meta traffic will be generated, and only those peers
defined in the configuration will be able to communicate.
The STATIC discovery related settings are:

+------------------------------+---------------------------------------------------------------------------------------+
| Name                         | Description                                                                           |
+==============================+=======================================================================================+
| :ref:`static_edp`            | It activates the STATIC discovery protocol.                                           |
+------------------------------+---------------------------------------------------------------------------------------+
| :ref:`static_xml`            | Specifies an XML content with a description of the remote DataWriters and |br|        |
|                              | DataReaders.                                                                          |
+------------------------------+---------------------------------------------------------------------------------------+
| :ref:`Initial Announcements` | It defines the behavior of the DomainParticipant initial announcements (PDP phase).   |
+------------------------------+---------------------------------------------------------------------------------------+

.. _static_edp:

STATIC EDP
^^^^^^^^^^

To activate the STATIC EDP, the SEDP must be disabled on the |WireProtocolConfigQos-api|.
This can be done either by code or using an XML configuration file:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_STATIC_DISCOVERY_CODE                                                                        |
|    :end-before: //!                                                                                                  |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_STATIC_DISCOVERY_CODE                                                                     |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

Currently two different formats of exchanging information in the Participant Discovery Phase (PDP) are supported:
the default one and another that reduces the network bandwidth used.
:ref:`property_policies_edp_exchange_format` explains how to change this.

.. _`static_xml`:

STATIC EDP XML Configuration Specification
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Since activating STATIC EDP suppresses all EDP meta traffic, the information about the remote entities (DataWriters
and DataReaders) must be statically specified, which is done using dedicated XML files.
A |DomainParticipant| may load several of such configuration files so that the information about different entities can
be contained in one file, or split into different files to keep it more organized.
*Fast DDS*  provides a
`Static Discovery example <https://github.com/eProsima/Fast-DDS/blob/master/examples/cpp/dds/StaticHelloWorldExample>`_
that implements this EDP discovery protocol.

The following table describes all the possible elements of a STATIC EDP XML configuration file.
A full example of such file can be found in :ref:`static_xml_example`.

.. Some large words outside of table. Then table fit maximum line length

.. |besteffort| replace:: |BEST_EFFORT_RELIABILITY_QOS-api|
.. |reliable| replace:: |RELIABLE_RELIABILITY_QOS-api|
.. |volatile| replace:: |VOLATILE_DURABILITY_QOS-api|
.. |transientlocal| replace:: |TRANSIENT_LOCAL_DURABILITY_QOS-api|
.. |transient| replace:: |TRANSIENT_DURABILITY_QOS-api|


.. list-table::
   :header-rows: 1

   * - Name
     - Description
     - Values
     - Default
   * - ``<userId>``
     - Mandatory. |br|
       Uniquely identifies the DataReader/DataWriter.
     - ``uint16_t``
     - 0
   * - ``<entityID>``
     - EntityId of the DataReader/DataWriter.
     - ``uint16_t``
     - 0
   * - ``<expects_inline_qos>``
     - It indicates if QOS is expected inline |br|
       (DataReader **only**).
     - ``bool``
     - ``false``
   * - ``<topicName>``
     - Mandatory. |br|
       The topic of the remote DataReader/DataWriter. |br|
       Should match with one of the topics of the local DataReaders/DataWriters.
     - ``string_255``
     -
   * - ``<topicDataType>``
     - Mandatory. |br|
       The data type of the topic.
     - ``string_255``
     -
   * - ``<topicKind>``
     - The kind of topic.
     - :class:`NO_KEY` |br|
       :class:`WITH_KEY` |br|
     - :class:`NO_KEY` |br|
   * - ``<partitionQos>``
     - The name of a partition of the remote peer. |br|
       Repeat to configure several partitions.
     - ``string``
     -
   * - ``<unicastLocator>``
     - Unicast locator of the DomainParticipant. |br|
       See :ref:`staticLocators`.
     -
     -
   * - ``<multicastLocator>``
     - Multicast locator of the DomainParticipant. |br|
       See :ref:`staticLocators`.
     -
     -
   * - ``<reliabilityQos>``
     - See the :ref:`reliabilityqospolicy` section.
     - |besteffort| |br|
       |reliable|
     - |besteffort|
   * - ``<durabilityQos>``
     - See the :ref:`durabilityqospolicy` section.
     - |volatile| |br|
       |transientlocal| |br|
       |transient|
     - |volatile|
   * - ``<ownershipQos>``
     - See :ref:`ownershipQos`.
     -
     -
   * - ``<livelinessQos>``
     - Defines the liveliness of the remote peer. |br|
       See :ref:`livelinessQos`.
     -
     -
   * - ``<disablePositiveAcks>``
     - See :ref:`disablepositiveacksqospolicy`.
     - See :ref:`xml_disablepositiveacks`
     -

.. _staticLocators:

Locators definition
"""""""""""""""""""

Locators for remote peers are configured using ``<unicastLocator>`` and ``<multicastLocator>`` tags.
These take no value, and the locators are defined using tag elements.
Locators defined with ``<unicastLocator>`` and ``<multicastLocator>`` are accumulative, so they can be repeated to
assign several remote endpoints locators to the same peer.

* :class:`address`: a mandatory ``string`` representing the locator address.
* :class:`port`: an optional ``uint16_t`` representing a port on that address.

.. _ownershipQos:

Ownership QoS
"""""""""""""

The ownership of the topic can be configured using ``<ownershipQos>`` tag.
It takes no value, and the configuration is done using tag elements:

* :class:`kind`: can be one of |SHARED_OWNERSHIP_QOS-api| or |EXCLUSIVE_OWNERSHIP_QOS-api|.
  This element is mandatory withing the tag.

* :class:`strength`: an optional ``uint32_t`` specifying how strongly the remote DomainParticipant owns the |Topic|.
  This QoS can be set on DataWriters **only**.
  If not specified, default value is zero.

.. _livelinessQos:

Liveliness QoS
""""""""""""""

The :ref:`livelinessqospolicy` of the remote peer is configured using ``<livelinessQos>`` tag.
It takes no value, and the configuration is done using tag elements:

* :class:`kind`: can be any of |AUTOMATIC_LIVELINESS_QOS-api|, |MANUAL_BY_PARTICIPANT_LIVELINESS_QOS-api| or
  |MANUAL_BY_TOPIC_LIVELINESS_QOS-api|. This element is mandatory withing the tag.

* :class:`leaseDuration_ms`: an optional ``uint32`` specifying the lease duration for the remote peer.
  The special value :class:`INF` can be used to indicate infinite lease duration.
  If not specified, default value is :class:`INF`

.. _check_static_xml_file:

Checking STATIC EDP XML Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before loading a static EDP XML file, it would be useful to check its validity and make sure the file will be
successfully loaded.
This verification can be performed on :ref:`dds_layer_domainParticipantFactory` using
|DomainParticipantFactory::check_xml_static_discovery-api|, using either XML files or the configuration directly,
as in the examples below.

.. tabs::

    .. tab:: File

        .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: // Check XML static discovery from file
           :end-before: //!--
           :dedent: 8

    .. tab:: Data

        .. literalinclude:: /../code/DDSCodeTester.cpp
           :language: c++
           :start-after: // Check XML static discovery from data
           :end-before: //!--
           :dedent: 8

.. _static_xml_example:

STATIC EDP XML Example
""""""""""""""""""""""

The following is a complete example of a configuration XML file for two remote DomainParticipant, a DataWriter and
a DataReader.
This configuration **must** agree with the configuration used to create the remote DataReader/DataWriter.
Otherwise, communication between DataReaders and DataWriters may be affected.
If any non-mandatory element is missing, it will take the default value.
As a rule of thumb, all the elements that were specified on the remote DataReader/DataWriter creation should be
configured.

+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/StaticTester.xml                                                                        |
|    :language: xml                                                                                                    |
|    :start-after: <!-->STATIC_DISCOVERY_CONF<-->                                                                      |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _static_xml_load:

Loading STATIC EDP XML Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Statically discovered remote DataReaders/DataWriters **must** define a unique *userID* on their profile, whose value
**must** agree with the one specified in the discovery configuration XML.
This is done by setting the user ID on the |DataReaderQoS|/|DataWriterQoS|:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF_QOS_STATIC_DISCOVERY_USERID                                                                  |
|    :end-before: //!                                                                                                  |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF_QOS_STATIC_DISCOVERY_USERID                                                               |
|    :end-before: <!-->                                                                                                |
+----------------------------------------------------------------------------------------------------------------------+

On the local DomainParticipant, you can load STATIC EDP configuration content specifying the file containing it.

+------------------------------------------------------+
| **C++**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp       |
|    :language: c++                                    |
|    :start-after: //CONF_STATIC_DISCOVERY_XML_FILE    |
|    :end-before: //!                                  |
|    :dedent: 8                                        |
+------------------------------------------------------+
| **XML**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml           |
|    :language: xml                                    |
|    :start-after: <!-->CONF_STATIC_DISCOVERY_XML_FILE |
|    :end-before: <!-->                                |
+------------------------------------------------------+

Or you can specify the STATIC EDP configuration content directly.

+------------------------------------------------------+
| **C++**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp       |
|    :language: c++                                    |
|    :start-after: //CONF_STATIC_DISCOVERY_XML_DATA    |
|    :end-before: //!                                  |
|    :dedent: 8                                        |
+------------------------------------------------------+
