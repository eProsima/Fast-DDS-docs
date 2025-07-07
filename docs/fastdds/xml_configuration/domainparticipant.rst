.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _participantprofiles:

DomainParticipant profiles
--------------------------

The |DomainParticipant| profiles allow the definition of the configuration of |DomainParticipants-api| through
XML files.
These profiles are defined within the ``<participant>`` XML tags.

.. _domainparticipantattributes:

DomainParticipant XML attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<participant>`` element has two attributes defined: ``profile_name`` and ``is_default_profile``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``profile_name``
     - Sets the name under which the ``<participant>`` profile is registered in the DDS Domain, 
       so that it can be loaded later by the |DomainParticipantFactory-api|, as shown in 
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<participant>`` profile as the default profile. Thus, if a default profile 
       exists, it will be used when no other DomainParticipant profile is specified at the 
       DomainParticipant's creation.
     - Optional

.. _domainparticipantconfig:

DomainParticipant configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``<participant>`` element has two child elements: ``<domainId>`` and ``<rtps>``.
All the DomainParticipant configuration options belong to the ``<rtps>`` element, except for the DDS |DomainId-api|
which is defined by the ``<domainId>`` element.
Below a list with the configuration XML elements is presented:

+----------------+----------------------------------------------------------------------------+--------------+---------+
| Name           | Description                                                                | Values       | Default |
+================+============================================================================+==============+=========+
| ``<domainId>`` | DomainId to be used by the DomainParticipant.                              | ``uint32_t`` | 0       |
|                | See :ref:`dds_layer_domainParticipant_creation_profile`.                   |              |         |
+----------------+----------------------------------------------------------------------------+--------------+---------+
| ``<rtps>``     | *Fast DDS* DomainParticipant configurations.                               |  :ref:`RTPS` |         |
|                | See :ref:`RTPS`.                                                           |              |         |
+----------------+----------------------------------------------------------------------------+--------------+---------+

.. _RTPS:

RTPS element type
"""""""""""""""""

The following is a list with all the possible child XML elements of the ``<rtps>`` element.
These elements allow the user to define the DomainParticipant configuration.

.. |PartAlloc| replace:: :ref:`DomainParticipantAllocationType <ParticipantAllocationType>`
.. |PolicyType| replace:: :ref:`PropertiesPolicyType <PropertiesPolicyType>`
.. |BuiltinTransportType| replace:: :ref:`BuiltinTransportType <BuiltinTransportType>`


.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<name>``
     - The DomainParticipant's name.
     - ``string_255``
     -
   * - ``<defaultUnicastLocatorList>``
     - List of default reception unicast locators 
       for user data traffic (see 
       ``<metatrafficUnicastLocatorList>`` 
       defined in :ref:`builtin`). 
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<defaultMulticastLocatorList>``
     - List of default reception multicast 
       locators for user data traffic (see 
       ``<metatrafficMulticastLocatorList>`` 
       defined in :ref:`builtin`). 
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<default_external_unicast_locators>``
     - List of :ref:`external_locators` 
       to announce for the default user traffic of 
       this participant.
     - :ref:`externalLocatorListType`
     -
   * - ``<ignore_non_matching_locators>``
     - Whether to ignore locators received on 
       announcements from other participants when 
       they don't match with any of the locators 
       announced by this participant.
     - ``bool``
     - false
   * - ``<sendSocketBufferSize>``
     - Size in bytes of the send socket buffer. 
       If the value is zero then *Fast DDS* will 
       use the system default socket size.
     - ``uint32_t``
     - 0
   * - ``<listenSocketBufferSize>``
     - Size in bytes of the reception socket 
       buffer. If the value is zero then 
       *Fast DDS* will use the system default 
       socket size.
     - ``uint32_t``
     - 0
   * - ``<netmask_filter>``
     - Participant's netmask 
       filtering configuration. 
       See the :ref:`netmask_filtering` section.
     - |NetmaskFilterKind-api|
     - |NetmaskFilterKind::AUTO-api|
   * - ``<builtin>``
     - |WireProtocolConfigQos::builtin-api| public data member of the 
       |WireProtocolConfigQos-api| class. 
       See the :ref:`builtin` section.
     - :ref:`builtin`
     -
   * - ``<port>``
     - Allows defining the port and gains related 
       to the RTPS protocol. See the `Port`_ section.
     - `Port`_
     -
   * - ``<participantID>``
     - DomainParticipant's identifier. Typically 
       it will be automatically generated by the 
       |DomainParticipantFactory|.
     - ``int32_t``
     - 0
   * - ``<easy_mode_ip>``
     - IP address of the remote discovery server 
       to connect to using
       `Discovery Server Easy Mode <https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html>`__.
     - ``string``
     - Empty
   * - ``<userTransports>``
     - Transport descriptors to be used by the 
       DomainParticipant. See 
       :ref:`transportdescriptors`.
     - ``List <string>``
     -
   * - ``<useBuiltinTransports>``
     - Boolean field to indicate the system 
       whether the DomainParticipant will use the 
       default |WireProtocolConfigQos::builtin-api| transports 
       in addition to its ``<userTransports>``.
     - ``bool``
     - true
   * - ``<builtinTransports>``
     - Configuration option to determine which transports 
       will be instantiated if the ``useBuiltinTransports`` is 
       set to true. See :ref:`rtps_layer_builtin_transports`.
     - |BuiltinTransportType|
     -
   * - ``<propertiesPolicy>``
     - Additional configuration properties. 
       See :ref:`propertypolicyqos`.
     - |PolicyType|
     -
   * - ``<allocation>``
     - Configuration regarding allocation behavior. 
       It expects a |PartAlloc|.
     - |PartAlloc|
     -
   * - ``<userData>``
     - Additional information attached to the DomainParticipant 
       and transmitted with the discovery information. 
       See :ref:`userdataqospolicy`.
     - ``List <string>``
     - Empty
   * - ``<prefix>``
     - |DomainParticipant|'s |GuidPrefix_t-api| identifies peers 
       running in the same process. Two participants with identical 
       8 first bytes on the |GuidPrefix_t-api| are considered to be 
       running in the same process, and therefore intra-process 
       delivery is used. See :ref:`intraprocess-delivery`.
     - ``string``
     - Empty
   * - ``<builtin_controllers_sender_thread>``
     - |ThreadSettings| for the builtin flow controllers sender thread.
     - |ThreadSettings|
     -
   * - ``<timed_events_thread>``
     - |ThreadSettings| participant's timed events thread.
     - |ThreadSettings|
     -
   * - ``<discovery_server_thread>``
     - |ThreadSettings| for the discovery server thread.
     - |ThreadSettings|
     -
   * - ``<typelookup_service_thread>``
     - |ThreadSettings| for the threads used by the builtin TypeLookup service 
       to discover unknown remote types. 
       See :ref:`xtypes_discovery_matching`.
     - |ThreadSettings|
     -
   * - ``<builtin_transports_reception_threads>``
     - |ThreadSettings| for the builtin transports reception threads.
     - |ThreadSettings|
     -
   * - ``<security_log_thread>``
     - |ThreadSettings| for the security log thread.
     - |ThreadSettings|
     -
   * - ``<flow_controller_descriptor_list>``
     - Defined flow controller descriptors to be used by the 
       DomainParticipant. See :ref:`flowcontrollers_xml`.
     - |FlowControllersQos|
     -

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-150, 152-153

.. note::

    - :class:`LOCATOR_LIST` means a :ref:`LocatorListType` is expected.

    - :class:`EXTERNAL_LOCATOR_LIST` means a :ref:`externalLocatorListType` is expected.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - For :class:`ALLOCATION` details, please refer to :ref:`ParticipantAllocationType`.

.. _builtin:

Builtin parameters
""""""""""""""""""

By calling the |DomainParticipantQos::wire_protocol-api| member function of the |DomainParticipantQos-api|,
it is possible to
access the |WireProtocolConfigQos::builtin-api| public data member of the |WireProtocolConfigQos-api|
class.
This section specifies the available XML members for the configuration of this
|WireProtocolConfigQos::builtin-api| parameters.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<discovery_config>``
     - This is the main element within 
       which discovery-related 
       settings can be configured. 
       See :ref:`discovery`.
     - :ref:`dconf`
     -
   * - ``<avoid_builtin_multicast>``
     - Restricts multicast metatraffic 
       to PDP only.
     - ``bool``
     - :class:`true`
   * - ``<use_WriterLivelinessProtocol>``
     - Indicates whether to use the 
       DataWriterLiveliness protocol.
     - ``bool``
     - :class:`true`
   * - ``<metatrafficUnicastLocatorList>``
     - Metatraffic Unicast Locator List.
     - A set of ``<locator>``  members.  See :ref:`LocatorListType`
     -
   * - ``<metatrafficMulticastLocatorList>``
     - Metatraffic Multicast Locator List.
     - A set of ``<locator>``  members.  See :ref:`LocatorListType`
     -
   * - ``<initialPeersList>``
     - The list of IP-port address 
       pairs of all other 
       |DomainParticipants| with which 
       a |DomainParticipant| will 
       communicate. See 
       :ref:`Simple Initial Peers`
     - A set of ``<locator>``  members.  See :ref:`LocatorListType`
     -
   * - ``<metatraffic_external_unicast_locators>``
     - List of :ref:`external_locators` 
       to announce for the metatraffic of 
       this participant.
     - :ref:`externalLocatorListType`
     -
   * - ``<readerHistoryMemoryPolicy>``
     - Memory policy for DataReaders. 
       See :ref:`historyqospolicykind`.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<writerHistoryMemoryPolicy>``
     - Memory policy for DataWriters. 
       See :ref:`historyqospolicykind`.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<readerPayloadSize>``
     - Maximum DataReader's History 
       payload size. Allows to reserve 
       all the required memory at 
       DataReader initialization. 
       See :ref:`memorymanagementpolicy`.
     - ``uint32_t``
     - 512
   * - ``<writerPayloadSize>``
     - Maximum DataWriter's History 
       payload size. Allows to reserve 
       all the required memory at 
       DataWriter initialization. 
       See :ref:`memorymanagementpolicy`.
     - ``uint32_t``
     - 512
   * - ``<mutation_tries>``
     - Number of different ports 
       to try if DataReader's physical 
       port is already in use.
     - ``uint32_t``
     - 100
   * - ``<flow_controller_name>``
     - :ref:`flowcontrollersqos` name.
     - ``string``
     - Empty


**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

.. _dconf:

discovery_config
****************

Through the ``<discovery_config>`` element, *Fast DDS* allows the configuration of the discovery mechanism via an XML
file.
Please refer to the :ref:`discovery` section for more detail on the various types of discovery mechanisms and
configurable settings.

+---------------------------------+------------------------------------------------+---------------------+-------------+
| Name                            | Description                                    | Values              | Default     |
+=================================+================================================+=====================+=============+
| ``<discoveryProtocol>``         | Indicates which discovery protocol             | |SIMPLE|            | |SIMPLE|    |
|                                 | the DomainParticipant will use.                +---------------------+             |
|                                 | See :ref:`disc_mechanisms`. If not set to      | |CLIENT|            |             |
|                                 | |SIMPLE| or |NONE|,                            +---------------------+             |
|                                 | ``<discoveryServersList>``                     |                     |             |
|                                 | element would be used.                         | |SERVER|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |BACKUP|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |SUPER_CLIENT|      |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |NONE|              |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<discoveryServersList>``      | Describes locators of servers from which       | ``<locator>``       |             |
|                                 | it receives only the discovery information     |                     |             |
|                                 | they require to establish communication        |                     |             |
|                                 | with matching endpoints.                       |                     |             |
|                                 | It expects a :ref:`LocatorListType`.           |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<ignoreParticipantFlags>``    | Restricts metatraffic using several            | :ref:`partfiltering`| |NO_FILTER| |
|                                 | filtering criteria.                            |                     |             |
|                                 | See :ref:`discovery_ignore_flags`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<EDP>``                       | If set to |SIMPLE|, ``<simpleEDP>``            | |SIMPLE|            | |SIMPLE|    |
|                                 | element would be used.                         +---------------------+             |
|                                 | If set to ``STATIC``, |EDPStatic| will be      | ``STATIC``          |             |
|                                 | performed, configured with the contents        |                     |             |
|                                 | of the XML file set in                         |                     |             |
|                                 | ``<static_edp_xml_config>``.                   |                     |             |
|                                 | See :ref:`discovery`.                          |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<simpleEDP>``                 | Attributes of the Simple Discovery             | :ref:`sedp`         |             |
|                                 | Protocol. See :ref:`Simple EDP Attributes`.    |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseDuration>``             | Indicates how long the DomainParticipant       | :ref:`DurationType` | 20s         |
|                                 | should consider remote DomainParticipants      |                     |             |
|                                 | alive.                                         |                     |             |
|                                 | See :ref:`discovery_lease_dur`.                |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseAnnouncement>``         | The period for the DomainParticipant to        | :ref:`DurationType` | 3s          |
|                                 | send its discovery message to all other        |                     |             |
|                                 | discovered DomainParticipants as well as       |                     |             |
|                                 | to all Multicast ports.                        |                     |             |
|                                 | See :ref:`discovery_lease_announ`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<initialAnnouncements>``      | Allows the user to configure the number        | :ref:`InitAnnounce` |             |
|                                 | and period of the DomainParticipant's initial  |                     |             |
|                                 | discovery messages.                            |                     |             |
|                                 | See :ref:`Initial Announcements`.              |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<clientAnnouncementPeriod>``  | The period for the DomainParticipant to        | :ref:`DurationType` | 450 ms      |
|                                 | send its Discovery Message to its servers      |                     |             |
|                                 | and check for EDP endpoints matching.          |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<static_edp_xml_config>``     | The XML filename(s) with the static EDP        | ``List <string>``   |             |
|                                 | configuration. Only necessary if               |                     |             |
|                                 | the ``<EDP>`` member is set to                 |                     |             |
|                                 | ``STATIC``. See :ref:`discovery_static`.       |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+

.. _partfiltering:

ignoreParticipantFlags
######################

.. list-table::
  :header-rows: 1
  :align: left

  * - Possible values
    - Description
  * - |NO_FILTER|
    - All Discovery traffic is processed.
  * - |FILTER_DIFFERENT_HOST|
    - Discovery traffic from another host is discarded.
  * - |FILTER_DIFFERENT_PROCESS|
    - Discovery traffic from another process on the same host is discarded.
  * - |FILTER_SAME_PROCESS|
    - Discovery traffic from DomainParticipant's own process is discarded.

This option also supports the OR (``|``) operator to filter discovery traffic from other configurations.
For instance, ``FILTER_DIFFERENT_PROCESS|FILTER_SAME_PROCESS`` value discards discovery traffic from the
DomainParticipant's own host.

.. _sedp:

simpleEDP
#########

+---------------------------+------------------------------------------------------------+-------------+---------------+
| Name                      | Description                                                | Values      | Default       |
+===========================+============================================================+=============+===============+
| ``<PUBWRITER_SUBREADER>`` | Indicates if the participant must use                      | ``bool``    | ``true``      |
|                           | Publication DataWriter and Subscription DataReader.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+
| ``<PUBREADER_SUBWRITER>`` | Indicates if the participant must use                      | ``bool``    | ``true``      |
|                           | Publication DataReader and Subscription DataWriter.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+

.. _InitAnnounce:

Initial Announcements
#####################

+--------------+-----------------------------------------------------------------------+---------------------+---------+
| Name         | Description                                                           | Values              | Default |
+==============+=======================================================================+=====================+=========+
| ``<count>``  | Number of initial discovery messages to send at the period            | ``uint32_t``        | 5       |
|              | specified by                                                          |                     |         |
|              | ``<period>``. After these announcements, the DomainParticipant will   |                     |         |
|              | continue                                                              |                     |         |
|              | sending its discovery messages at the ``<leaseAnnouncement>`` rate.   |                     |         |
+--------------+-----------------------------------------------------------------------+---------------------+---------+
| ``<period>`` | The period for the DomainParticipant to send its discovery messages.  | :ref:`DurationType` | 100 ms  |
+--------------+-----------------------------------------------------------------------+---------------------+---------+

.. _Port:

Port Configuration
""""""""""""""""""

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the
|RTPSParticipants-api|' discovery traffic unicast listening ports are calculated using the following equation:
:math:`7400 + 250 * DomainId + 10 + 2 * ParticipantId`.
Therefore the following parameters can be specified:

+------------------------------+------------------------------------------------------+-----------------+--------------+
| Name                         | Description                                          | Values          | Default      |
+==============================+======================================================+=================+==============+
| ``<portBase>``               | Base ``port``.                                       | ``uint16_t``    | 7400         |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<domainIDGain>``           | Gain in DomainId.                                    | ``uint16_t``    | 250          |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<participantIDGain>``      | Gain in |WireProtocolConfigQos::participant_id-api|. | ``uint16_t``    | 2            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd0>``               | Multicast metadata offset.                           | ``uint16_t``    | 0            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd1>``               | Unicast metadata offset.                             | ``uint16_t``    | 10           |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd2>``               | Multicast user data offset.                          | ``uint16_t``    | 1            |
+------------------------------+------------------------------------------------------+-----------------+--------------+
| ``<offsetd3>``               | Unicast user data offset.                            | ``uint16_t``    | 11           |
+------------------------------+------------------------------------------------------+-----------------+--------------+

.. warning::

  Changing these default parameters may break compatibility with other RTPS compliant implementations, as well as
  with other *Fast DDS* applications with default port settings.

.. _ParticipantAllocationType:

ParticipantAllocationType
"""""""""""""""""""""""""

The ``ParticipantAllocationType`` defines the ``<allocation>`` element, which allows setting of the parameters
related with the allocation behavior on the DomainParticipant.
Please refer to :ref:`participantresourcelimitsqos` for a detailed documentation on DomainParticipants allocation
configuration.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
   * - ``<remote_locators>``
     - Defines the limits for the remote locators' collections. 
       See :ref:`remotelocatorsallocationattributes`.
     - :ref:`remote_locators_allocations`
   * - ``<total_participants>``
     - DomainParticipant :ref:`CommonAlloc` to specify the 
       total number of DomainParticipants in the domain 
       (local and remote). See 
       :ref:`ResourceLimitedContainerConfig`.
     - :ref:`CommonAlloc`
   * - ``<total_readers>``
     - DomainParticipant :ref:`CommonAlloc` to specify the 
       total number of DataReader on each DomainParticipant 
       (local and remote). See 
       :ref:`ResourceLimitedContainerConfig`.
     - :ref:`CommonAlloc`
   * - ``<total_writers>``
     - DomainParticipant :ref:`CommonAlloc` related to the 
       total number of DataWriters on each DomainParticipant 
       (local and remote).
       See :ref:`resourcelimitedcontainerconfig`.
     - :ref:`CommonAlloc`
   * - ``<max_partitions>``
     - Maximum size of the partitions submessage. 
       Set to zero for no limit.
     - ``uint32_t``
   * - ``<max_user_data>``
     - Maximum size of the user data submessage. 
       Set to zero for no limit.
     - ``uint32_t``
   * - ``<max_properties>``
     - Maximum size of the properties submessage. 
       Set to zero for no limit.
     - ``uint32_t``
   * - ``<send_buffers>``
     - Allocation behaviour for the send buffer 
       manager.
     - :ref:`SendBuffers`

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT-ALLOCATION<-->
    :end-before: <!--><-->

.. _remote_locators_allocations:

Remote Locators Allocations
***************************

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<max_unicast_locators>``
     - Maximum number of unicast locators expected on a 
       remote  entity. It is recommended to use the maximum 
       number of network interfaces available on the machine 
       on which DomainParticipant is running. 
       See :ref:`remotelocatorsallocationattributes`.
     - ``uint32_t``
     - 4
   * - ``<max_multicast_locators>``
     - Maximum number of multicast locators expected on a 
       remote entity. May be set to zero to disable multicast 
       traffic. See :ref:`remotelocatorsallocationattributes`.
     - ``uint32_t``
     - 1

.. _SendBuffers:

Send buffers
************

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<preallocated_number>``
     - Initial number of send buffers to allocate. See 
       :ref:`sendbuffersallocationattributes`.
     - ``uint32_t``
     - 0
   * - ``<dynamic>``
     - Whether the number of send buffers is allowed to 
       grow. See :ref:`sendbuffersallocationattributes`.
     - ``bool``
     - false
   * - ``<network_buffers_config>``
     - Network buffer :ref:`CommonAlloc` to specify the 
       number of network buffers to be allocated for each 
       send buffer. See :ref:`ResourceLimitedContainerConfig`.
     - :ref:`CommonAlloc`
     - (16, inf, 16)

.. note::
    The default value ``0`` of ``<preallocated_number>`` will perform an initial guess of the number of buffers
    required, based on the number of threads from which a send operation could be started.
    So it does not mean there are no buffers, instead it would use the maximum amount of buffers available.
    On the contrary, ``<network_buffers_config>`` will default to an initial number of 16 buffers, with an infinite
    maximum and an increment of 16 buffers per send buffer.
    An initial value of ``0`` will imply more dynamic allocations, especially at the beginning of the execution.
    In case of doubt, it should be left to the default values.
