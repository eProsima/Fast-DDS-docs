.. include:: includes/aliases.rst

.. _participantprofiles:

DomainParticipant profiles
--------------------------

The |DomainParticipant| profiles allow the definition of the configuration of the |DomainParticipants| through
XML profile files.
These profiles are defined within the ``<participant>`` XML tags.
The ``<participant>`` element has two child elements: ``<domain_id>`` and ``<rtps>``.
All the |DomainParticipant| configuration options belong to the ``<rtps>`` element, except for the DDS |DomainId| which
is define by the ``<domain_id>`` element.
Attribute ``profile_name`` is the name under which the ``<participant>`` profile is registered in the DDS Domain, so
that it can be loaded later by a |DomainParticipant|, as shown in :ref:`loadingapplyingprofiles`.
Below a list with the configuration XML elements is presented:

+----------------+----------------------------------------------------------------------------+--------------+---------+
| Name           | Description                                                                | Values       | Default |
+================+============================================================================+==============+=========+
| ``<domainId>`` | |DomainId| to be used by the |DomainParticipant|.                          | ``uint32``   | 0       |
+----------------+----------------------------------------------------------------------------+--------------+---------+
| ``<rtps>``     | Fast DDS |DomainParticipant| configurations. |br|                          |  :ref:`RTPS` |         |
|                | These are explained in :ref:`RTPS`.                                        |              |         |
+----------------+----------------------------------------------------------------------------+--------------+---------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

    - For :class:`ALLOCATION` details, please refer to :ref:`ParticipantAllocationType`.


.. _RTPS:

DomainParticipant Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following is a list with the possible child XML elements of the ``<rtps>`` element.
These elements allows the user to define the DomainParticipant configuration.

.. |PartAlloc| replace:: :ref:`DomainParticipantAllocationType <ParticipantAllocationType>`
.. |PolicyType| replace:: :ref:`PropertiesPolicyType <PropertiesPolicyType>`


+-----------------------------------+--------------------------------------------------+---------------------+---------+
| Name                              | Description                                      | Values              | Default |
+===================================+==================================================+=====================+=========+
| ``<name>``                        | DomainParticipant's name. It is not the |br|     | ``string_255``      |         |
|                                   | same field that ``profile_name`` attribute.      |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<defaultUnicastLocatorList>``   | List of default input unicast locators. |br|     | ``Locator``         |         |
|                                   | It expects a :ref:`LocatorListType`.             |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<defaultMulticastLocatorList>`` | List of default input multicast locators. |br|   | ``Locator``         |         |
|                                   | It expects a :ref:`LocatorListType`.             |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<sendSocketBufferSize>``        | Size in bytes of the output socket buffer. |br|  | ``uint32``          | 0       |
|                                   | If the value is zero then Fast DDS will use |br| |                     |         |
|                                   | the default size sockets configuration, |br|     |                     |         |
|                                   | using a minimum size of 65536 bytes.             |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<listenSocketBufferSize>``      | Size in bytes of the input socket buffer. |br|   | ``uint32``          | 0       |
|                                   | If the value is zero then Fast DDS will use |br| |                     |         |
|                                   | the default size sockets configuration, |br|     |                     |         |
|                                   | using a minimum size of 65536 bytes.             |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<builtin>``                     | |DomainParticipantQosWireProtocolBuiltin|        | :ref:`builtin`      |         |
|                                   | public data member of the |br|                   |                     |         |
|                                   | |DomainParticipantQosWireProtocolClass|          |                     |         |
|                                   | class. |br|                                      |                     |         |
|                                   | See the                                          |                     |         |
|                                   | :ref:`builtin` section.                          |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<port>``                        | Allows defining the port and gains related |br|  | `Port`_             |         |
|                                   | to the RTPS protocol. See the `Port`_ section.   |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<participantID>``               | DomainParticipant's identifier. Typically |br|   | ``int32``           | 0       |
|                                   | it will be automatically generated by the |br|   |                     |         |
|                                   | |DomainParticipantFactory|.                      |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<throughputController>``        | Allows defining a maximum throughput. |br|       | :ref:`Throughput`   |         |
|                                   | See the :ref:`Throughput` section.               |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<userTransports>``              | Transport descriptors to be used by the |br|     | ``List <string>``   |         |
|                                   | DomainParticipant.                               |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<useBuiltinTransports>``        | Boolean field to indicate the system |br|        | ``bool``            | true    |
|                                   | that the DomainParticipant will use the |br|     |                     |         |
|                                   | default                                          |                     |         |
|                                   | |DomainParticipantQosWireProtocolBuiltin|        |                     |         |
|                                   | transport independently |br|                     |                     |         |
|                                   | of its ``<userTransports>``.                     |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<propertiesPolicy>``            | Additional configuration properties. |br|        | |PolicyType|        |         |
|                                   | It expects a |PolicyType|.                       |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+
| ``<allocation>``                  | Configuration regarding allocation behavior.     | |PartAlloc|         |         |
|                                   | |br| It expects a |br| |PartAlloc|.              |                     |         |
+-----------------------------------+--------------------------------------------------+---------------------+---------+

.. _Port:

Port Configuration
""""""""""""""""""

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the
|RTPSParticipants|'discovery traffic unicast listening ports are calculated using the following equation:
:math:`7400 + 250 * DomainId + 10 + 2 * ParticipantId`.
Therefore the following parameters can be specified:

+------------------------------+------------------------------------------------------+-----------------+--------------+
| Name                         | Description                                          | Values          | Default      |
+==============================+======================================================+=================+==============+
| ``<portBase>``               | Base ``port``.                                       | ``uint16``      | 7400         |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<domainIDGain>``           | Gain in |DomainId|.                                  | ``uint16``      | 250          |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<participantIDGain>``      | Gain in |DomainParticipant_Id|.                      | ``uint16``      | 2            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd0>``               | Multicast metadata offset.                           | ``uint16``      | 0            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd1>``               | Unicast metadata offset.                             | ``uint16``      | 10           |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd2>``               | Multicast user data offset.                          | ``uint16``      | 1            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd3>``               | Unicast user data offset.                            | ``uint16``      | 11           |
+------------------------------+------------------------------------------------------+-----------------+--------------+

.. _ParticipantAllocationType:

ParticipantAllocationType
"""""""""""""""""""""""""

The ``ParticipantAllocationType`` defines the ``<allocation>`` element, which allows setting of the parameters
related with the allocation behavior on the DomainParticipant.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<max_unicast_locators>``
     - Maximum number of unicast locators expected on a |br|
       remote  entity. It is recommended to use the maximum |br|
       number of network interfaces available on the machine |br|
       on which DomainParticipant is running.
     - :ref:`CommonAlloc`
     - 4
   * - ``<max_multicast_locators>``
     - Maximum number of multicast locators expected on a |br|
       remote entity. May be set to zero to disable multicast |br|
       traffic.
     - ``UInt32``
     - 1
   * - ``<total_participants>``
     - DomainParticipant :ref:`CommonAlloc` to specify the |br|
       total number of DomainParticipants in the domain |br|
       (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_readers>``
     - DomainParticipant :ref:`CommonAlloc` to specify the |br|
       total number of DataReader on each DomainParticipant |br|
       (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_writers>``
     - DomainParticipant :ref:`CommonAlloc` related to the |br|
       total number of DataWriters on each DomainParticipant |br|
       (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<max_partitions>``
     - Maximum size of the partitions submessage. |br|
       Set to zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_user_data>``
     - Maximum size of the user data submessage. |br|
       Set to zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_properties>``
     - Maximum size of the properties submessage. |br|
       Set to zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT-ALLOCATION<-->
    :end-before: <!--><-->

.. _builtin:

Built-in parameters
"""""""""""""""""""

By calling the |DomainParticipantQosWireProtocol| member function of the |DomainParticipantQos|, it is possible to
access the |DomainParticipantQosWireProtocolBuiltin| public data member of the |DomainParticipantQosWireProtocolClass|
class.
This section specifies the available XML members for the configuration of this
|DomainParticipantQosWireProtocolBuiltin| parameters.

.. Some large words outside of table. Then table fit maximum line length

.. |loclist| replace:: A set of ``<locator>`` members. |br| See :ref:`LocatorListType`
.. |mempol| replace:: :ref:`HistoryMemoryPolicy <mempol>`
.. |mempoldefault| replace:: :class:`PREALLOCATED_WITH_REALLOC`

+---------------------------------------+--------------------------------------+---------------------+-----------------+
| Name                                  | Description                          | Values              | Default         |
+=======================================+======================================+=====================+=================+
| ``<discovery_config>``                | This is the main element within |br| |                     |                 |
|                                       | which discovery-related  |br|        |                     |                 |
|                                       | settings can be configured. |br|     |                     |                 |
|                                       | See :ref:`discovery`.                |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<avoid_builtin_multicast>``         | Restricts metatraffic multicast |br| | ``Boolean``         | :class:`true`   |
|                                       | traffic to PDP only.                 |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<use_WriterLivelinessProtocol>``    | Indicates whether to use the |br|    | ``Boolean``         | :class:`true`   |
|                                       | DataWriterLiveliness protocol.       |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<metatrafficUnicastLocatorList>``   | Metatraffic Unicast Locator List.    | |loclist|           |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<metatrafficMulticastLocatorList>`` | Metatraffic Multicast Locator List.  | |loclist|           |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<initialPeersList>``                | Initial peers.                       | |loclist|           |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<DataReaderHistoryMemoryPolicy>``   | Memory policy for DataReaders. |br|  | |mempol|            | |mempoldefault| |
|                                       | See :ref:`historyqospolicykind`.     |                     |                 |
|                                       |                                      |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<DataWriterHistoryMemoryPolicy>``   | Memory policy for DataWriters. |br|  | |mempol|            | |mempoldefault| |
|                                       | See :ref:`historyqospolicykind`.     |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<DataReaderPayloadSize>``           | Maximum payload size for |br|        | ``UInt32``          | 512             |
|                                       | DataReaders.                         |                     |                 |
|                                       |                                      |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<DataWriterPayloadSize>``           | Maximum payload size for |br|        | ``UInt32``          | 512             |
|                                       | DataWriters.                         |                     |                 |
|                                       |                                      |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+
| ``<mutation_tries>``                  | Number of different ports |br|       | ``UInt32``          | 100             |
|                                       | to try if DataReader's physical |br| |                     |                 |
|                                       | port is already in use.              |                     |                 |
+---------------------------------------+--------------------------------------+---------------------+-----------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

.. _dconf:

discovery_config
****************

Through the ``<discovery_config>`` element, Fast DDS allows the configuration of the discovery mechanism via an XML
profile file.
Please refer to the :ref:`discovery` section for more detail on the various types of discovery mechanisms and
configurable settings.

+---------------------------------+------------------------------------------------+---------------------+-------------+
| Name                            | Description                                    | Values              | Default     |
+=================================+================================================+=====================+=============+
| ``<discoveryProtocol>``         | Indicates which discovery protocol |br|        | |SIMPLE|            | |SIMPLE|    |
|                                 | the DomainParticipant will use. |br|           +---------------------+             |
|                                 | See :ref:`disc_mechanisms`.                    | |CLIENT|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |SERVER|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |BACKUP|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |NONE|              |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<ignoreParticipantFlags>``    | Restricts metatraffic using several |br|       | :ref:`partfiltering`| |NO_FILTER| |
|                                 | filtering criteria.                            |                     |             |
|                                 | See :ref:`discovery_ignore_flags`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<EDP>``                       | If set to ``SIMPLE``, ``<simpleEDP>`` |br|     | ``SIMPLE``          | ``SIMPLE``  |
|                                 | element would be used. |br|                    +---------------------+             |
|                                 | If set to ``STATIC``, |EDPStatic| will be |br| | ``STATIC``          |             |
|                                 | performed configured with the contents |br|    |                     |             |
|                                 | of the XML file set in                         |                     |             |
|                                 | ``<staticEndpointXMLFilename>``. |br|          |                     |             |
|                                 | See :ref:`discovery`.                          |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<simpleEDP>``                 | Attributes of the Simple Discovery |br|        | :ref:`sedp`         |             |
|                                 | Protocol. See :ref:`Simple EDP Attributes`.    |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseDuration>``             | Indicates how long the DomainParticipant |br|  | :ref:`DurationType` | 20 seconds  |
|                                 | should consider remote DomainParticipant |br|  |                     |             |
|                                 | alive.                                         |                     |             |
|                                 | See :ref:`discovery_lease_dur`.                |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseAnnouncement>``         | The period for the DomainParticipant to |br|   | :ref:`DurationType` | 3 seconds   |
|                                 | send its discovery message to all other |br|   |                     |             |
|                                 | discovered DomainParticipant as well as |br|   |                     |             |
|                                 | to all Multicast ports.                        |                     |             |
|                                 | See :ref:`discovery_lease_announ`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<initialAnnouncements>``      | Allows the user to configure the number |br|   | :ref:`InitAnnounce` |             |
|                                 | and period of the initial DomainParticipant's  |                     |             |
|                                 | |br| discovery messages.                       |                     |             |
|                                 | See :ref:`Initial Announcements`.              |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<staticEndpointXMLFilename>`` | The XML filename with the static EDP |br|      | ``string``          |             |
|                                 | configuration. Only necessary if |br|          |                     |             |
|                                 | the ``<EDP>`` member is set to |br|            |                     |             |
|                                 | ``STATIC``. See :ref:`discovery_static`.       |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+

.. _partfiltering:

ignoreParticipantFlags
######################

+--------------------------+-------------------------------------------------------------------------------------------+
| Possible values          | Description                                                                               |
+==========================+===========================================================================================+
| NO_FILTER                | All Discovery traffic is processed.                                                       |
+--------------------------+-------------------------------------------------------------------------------------------+
| FILTER_DIFFERENT_HOST    | Discovery traffic from another host is discarded.                                         |
+--------------------------+-------------------------------------------------------------------------------------------+
| FILTER_DIFFERENT_PROCESS | Discovery traffic from another process on the same host is discarded.                     |
+--------------------------+-------------------------------------------------------------------------------------------+
| FILTER_SAME_PROCESS      | Discovery traffic from DomainParticipant's own process is discarded.                      |
+--------------------------+-------------------------------------------------------------------------------------------+
| FILTER_DIFFERENT_PROCESS | Discovery traffic from DomainParticipant's own host is discarded.                         |
+--------------------------+                                                                                           |
| FILTER_SAME_PROCESS      |                                                                                           |
+--------------------------+-------------------------------------------------------------------------------------------+

.. _sedp:

simpleEDP
#########

+---------------------------+------------------------------------------------------------+-------------+---------------+
| Name                      | Description                                                | Values      | Default       |
+===========================+============================================================+=============+===============+
| ``<PUBWRITER_SUBREADER>`` | Indicates if the participant must use |br|                 | ``Boolean`` | ``true``      |
|                           | Publication DataWriter and Subscription DataReader.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+
| ``<PUBREADER_SUBWRITER>`` | Indicates if the participant must use |br|                 | ``Boolean`` | ``true``      |
|                           | Publication DataReader and Subscription DataWriter.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+

.. _InitAnnounce:

Initial Announcements
*********************

+--------------+-----------------------------------------------------------------------+---------------------+---------+
| Name         | Description                                                           | Values              | Default |
+==============+=======================================================================+=====================+=========+
| ``<count>``  | Number of initial discovery messages to send at the period            | ``Uint32``          | 5       |
|              | specified by |br|                                                     |                     |         |
|              | ``<period>``. After these announcements, the DomainParticipant will   |                     |         |
|              | continue |br|                                                         |                     |         |
|              | sending its discovery messages at the ``<leaseAnnouncement>`` rate.   |                     |         |
+--------------+-----------------------------------------------------------------------+---------------------+---------+
| ``<period>`` | The period for the DomainParticipant to send its discovery messages.  | :ref:`DurationType` | 100 ms  |
+--------------+-----------------------------------------------------------------------+---------------------+---------+
