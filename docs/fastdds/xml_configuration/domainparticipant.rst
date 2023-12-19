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
     - Sets the name under which the ``<participant>`` profile is registered in the DDS Domain, |br|
       so that it can be loaded later by the |DomainParticipantFactory-api|, as shown in |br|
       :ref:`loadingapplyingprofiles`.
     - Mandatory
   * - ``is_default_profile``
     - Sets the ``<participant>`` profile as the default profile. Thus, if a default profile |br|
       exists, it will be used when no other DomainParticipant profile is specified at the |br|
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
| ``<rtps>``     | *Fast DDS* DomainParticipant configurations. |br|                          |  :ref:`RTPS` |         |
|                | See :ref:`RTPS`.                                                           |              |         |
+----------------+----------------------------------------------------------------------------+--------------+---------+

.. _RTPS:

RTPS element type
"""""""""""""""""

The following is a list with all the possible child XML elements of the ``<rtps>`` element.
These elements allow the user to define the DomainParticipant configuration.

.. |PartAlloc| replace:: :ref:`DomainParticipantAllocationType <ParticipantAllocationType>`
.. |PolicyType| replace:: :ref:`PropertiesPolicyType <PropertiesPolicyType>`


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
     - List of default reception unicast locators |br|
       for user data traffic (see |br|
       ``<metatrafficUnicastLocatorList>`` |br|
       defined in :ref:`builtin`). |br|
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<defaultMulticastLocatorList>``
     - List of default reception multicast |br|
       locators for user data traffic (see |br|
       ``<metatrafficMulticastLocatorList>`` |br|
       defined in :ref:`builtin`). |br|
       It expects a :ref:`LocatorListType`.
     - ``<locator>``
     -
   * - ``<default_external_unicast_locators>``
     - List of :ref:`external_locators` |br|
       to announce for the default user traffic of |br|
       this participant.
     - :ref:`externalLocatorListType`
     -
   * - ``<ignore_non_matching_locators>``
     - Whether to ignore locators received on |br|
       announcements from other participants when |br|
       they don't match with any of the locators |br|
       announced by this participant.
     - ``bool``
     - false
   * - ``<sendSocketBufferSize>``
     - Size in bytes of the send socket buffer. |br|
       If the value is zero then *Fast DDS* will |br|
       use the system default socket size.
     - ``uint32_t``
     - 0
   * - ``<listenSocketBufferSize>``
     - Size in bytes of the reception socket |br|
       buffer. If the value is zero then |br|
       *Fast DDS* will use the system default |br|
       socket size.
     - ``uint32_t``
     - 0
   * - ``<builtin>``
     - |WireProtocolConfigQos::builtin-api| public data member of the |br|
       |WireProtocolConfigQos-api| class. |br|
       See the :ref:`builtin` section.
     - :ref:`builtin`
     -
   * - ``<port>``
     - Allows defining the port and gains related |br|
       to the RTPS protocol. See the `Port`_ section.
     - `Port`_
     -
   * - ``<participantID>``
     - DomainParticipant's identifier. Typically |br|
       it will be automatically generated by the |br|
       |DomainParticipantFactory|.
     - ``int32_t``
     - 0
   * - ``<userTransports>``
     - Transport descriptors to be used by the |br|
       DomainParticipant. See |br|
       :ref:`transportdescriptors`.
     - ``List <string>``
     -
   * - ``<useBuiltinTransports>``
     - Boolean field to indicate the system |br|
       whether the DomainParticipant will use the |br|
       default |WireProtocolConfigQos::builtin-api| transports |br|
       in addition to its ``<userTransports>``.
     - ``bool``
     - true
   * - ``<builtinTransports>``
     - Configuration option to determine which transports |br|
       will be instantiated if the ``useBuiltinTransports`` is |br|
       set to true. See :ref:`rtps_layer_builtin_transports`.
     - ``string_255``
     - DEFAULT
   * - ``<propertiesPolicy>``
     - Additional configuration properties. |br|
       See :ref:`propertypolicyqos`.
     - |PolicyType|
     -
   * - ``<allocation>``
     - Configuration regarding allocation behavior. |br|
       It expects a |br|
       |PartAlloc|.
     - |PartAlloc|
     -
   * - ``<userData>``
     - Additional information attached to the DomainParticipant |br|
       and transmitted with the discovery information. |br|
       See :ref:`userdataqospolicy`.
     - ``List <string>``
     - Empty
   * - ``<prefix>``
     - |DomainParticipant|'s |GuidPrefix_t-api| identifies peers |br|
       running in the same process. Two participants with identical |br|
       8 first bytes on the |GuidPrefix_t-api| are considered to be |br|
       running in the same process, and therefore intra-process |br|
       delivery is used. See :ref:`intraprocess-delivery`.
     - ``string``
     - Empty

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->
<<<<<<< HEAD
    :lines: 2-3, 5-88, 90
=======
    :lines: 2-4, 6-126, 128-129
>>>>>>> b3bf26c (Methods to configure transport scenarios Documentation (#619))

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
     - This is the main element within |br|
       which discovery-related |br|
       settings can be configured. |br|
       See :ref:`discovery`.
     - :ref:`dconf`
     -
   * - ``<avoid_builtin_multicast>``
     - Restricts multicast metatraffic |br|
       to PDP only.
     - ``bool``
     - :class:`true`
   * - ``<use_WriterLivelinessProtocol>``
     - Indicates whether to use the |br|
       DataWriterLiveliness protocol.
     - ``bool``
     - :class:`true`
   * - ``<metatrafficUnicastLocatorList>``
     - Metatraffic Unicast Locator List.
     - A set of ``<locator>`` |br| members. |br| See :ref:`LocatorListType`
     -
   * - ``<metatrafficMulticastLocatorList>``
     - Metatraffic Multicast Locator List.
     - A set of ``<locator>`` |br| members. |br| See :ref:`LocatorListType`
     -
   * - ``<initialPeersList>``
     - The list of IP-port address |br|
       pairs of all other |br|
       |DomainParticipants| with which |br|
       a |DomainParticipant| will |br|
       communicate. See |br|
       :ref:`Simple Initial Peers`
     - A set of ``<locator>`` |br| members. |br| See :ref:`LocatorListType`
     -
   * - ``<metatraffic_external_unicast_locators>``
     - List of :ref:`external_locators` |br|
       to announce for the metatraffic of |br|
       this participant.
     - :ref:`externalLocatorListType`
     -
   * - ``<readerHistoryMemoryPolicy>``
     - Memory policy for DataReaders. |br|
       See :ref:`historyqospolicykind`.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<writerHistoryMemoryPolicy>``
     - Memory policy for DataWriters. |br|
       See :ref:`historyqospolicykind`.
     - :ref:`historymemorypoliciesXML`
     - |PREALLOCATED-xml-api|
   * - ``<readerPayloadSize>``
     - Maximum DataReader's History |br|
       payload size. Allows to reserve |br|
       all the required memory at |br|
       DataReader initialization. |br|
       See :ref:`memorymanagementpolicy`.
     - ``uint32_t``
     - 512
   * - ``<writerPayloadSize>``
     - Maximum DataWriter's History |br|
       payload size. Allows to reserve |br|
       all the required memory at |br|
       DataWriter initialization. |br|
       See :ref:`memorymanagementpolicy`.
     - ``uint32_t``
     - 512
   * - ``<mutation_tries>``
     - Number of different ports |br|
       to try if DataReader's physical |br|
       port is already in use.
     - ``uint32_t``
     - 100


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
| ``<discoveryProtocol>``         | Indicates which discovery protocol |br|        | |SIMPLE|            | |SIMPLE|    |
|                                 | the DomainParticipant will use. |br|           +---------------------+             |
|                                 | See :ref:`disc_mechanisms`. If set to  |br|    | |CLIENT|            |             |
|                                 | |CLIENT|, ``<discoveryServersList>`` |br|      +---------------------+             |
|                                 | element would be used.                         | |SERVER|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |BACKUP|            |             |
|                                 |                                                +---------------------+             |
|                                 |                                                | |NONE|              |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<discoveryServersList>``      | Describes servers from which it receives |br|  | :ref:`discserverlst`|             |
|                                 | only the discovery information they |br|       |                     |             |
|                                 | require to establish communication with |br|   |                     |             |
|                                 | matching endpoints. |br|                       |                     |             |
|                                 | See :ref:`discovery_server`                    |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<ignoreParticipantFlags>``    | Restricts metatraffic using several |br|       | :ref:`partfiltering`| |NO_FILTER| |
|                                 | filtering criteria.                            |                     |             |
|                                 | See :ref:`discovery_ignore_flags`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<EDP>``                       | If set to |SIMPLE|, ``<simpleEDP>`` |br|       | |SIMPLE|            | |SIMPLE|    |
|                                 | element would be used. |br|                    +---------------------+             |
|                                 | If set to ``STATIC``, |EDPStatic| will be |br| | ``STATIC``          |             |
|                                 | performed, configured with the contents |br|   |                     |             |
|                                 | of the XML file set in                         |                     |             |
|                                 | ``<static_edp_xml_config>``. |br|              |                     |             |
|                                 | See :ref:`discovery`.                          |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<simpleEDP>``                 | Attributes of the Simple Discovery |br|        | :ref:`sedp`         |             |
|                                 | Protocol. See :ref:`Simple EDP Attributes`.    |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseDuration>``             | Indicates how long the DomainParticipant |br|  | :ref:`DurationType` | 20s         |
|                                 | should consider remote DomainParticipants |br| |                     |             |
|                                 | alive.                                         |                     |             |
|                                 | See :ref:`discovery_lease_dur`.                |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<leaseAnnouncement>``         | The period for the DomainParticipant to |br|   | :ref:`DurationType` | 3s          |
|                                 | send its discovery message to all other |br|   |                     |             |
|                                 | discovered DomainParticipants as well as |br|  |                     |             |
|                                 | to all Multicast ports.                        |                     |             |
|                                 | See :ref:`discovery_lease_announ`.             |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<initialAnnouncements>``      | Allows the user to configure the number |br|   | :ref:`InitAnnounce` |             |
|                                 | and period of the DomainParticipant's initial  |                     |             |
|                                 | |br| discovery messages.                       |                     |             |
|                                 | See :ref:`Initial Announcements`.              |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<clientAnnouncementPeriod>``  | The period for the DomainParticipant to |br|   | :ref:`DurationType` | 450 ms      |
|                                 | send its Discovery Message to its servers |br| |                     |             |
|                                 | and check for EDP endpoints matching.          |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+
| ``<static_edp_xml_config>``     | The XML filename(s) with the static EDP |br|   | ``List <string>``   |             |
|                                 | configuration. Only necessary if |br|          |                     |             |
|                                 | the ``<EDP>`` member is set to |br|            |                     |             |
|                                 | ``STATIC``. See :ref:`discovery_static`.       |                     |             |
+---------------------------------+------------------------------------------------+---------------------+-------------+


.. _discserverlst:

discoveryServersList
####################

Contains a list of ``<RemoteServer>`` (*server*) elements, which are defined by the |GuidPrefix_t-api| and their own
locator list which must be populated with |RemoteServerAttributes-api|.
The DomainParticipant set as *client* would discover the *servers* described in this section.


**Server attributes**

The ``<RemoteServer>`` element has a mandatory attribute defined: ``prefix``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Use
   * - ``prefix``
     - RTPS standard participant unique identifier, a 12-byte chain. |br|
       This identifier allows clients to assess whether they are |br|
       receiving messages from the right server.
     - Mandatory

**RemoteServer configuration**

Each *client* (or a *server* connecting to another *server*) must keep a list of locators associated to the *servers*
to which it wants to link.
Those locator would be defined as ``metatrafficUnicastLocatorList`` or ``metatrafficMulticastLocatorList``.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
  * - ``<metatrafficUnicastLocatorList>``
    - Metatraffic Unicast Locator List.
    - A set of ``<locator>`` members. |br| See :ref:`LocatorListType`
  * - ``<metatrafficMulticastLocatorList>``
    - Metatraffic Multicast Locator List.
    - A set of ``<locator>`` members. |br| See :ref:`LocatorListType`

**Example**

.. literalinclude:: /../code/XMLTester.xml
   :language: xml
   :start-after: <!-->CONF_DS_MAIN_SCENARIO_CLIENT<-->
   :end-before: <!--><-->
   :lines: 2-3,5-
   :append: </profiles>

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
| ``<PUBWRITER_SUBREADER>`` | Indicates if the participant must use |br|                 | ``bool``    | ``true``      |
|                           | Publication DataWriter and Subscription DataReader.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+
| ``<PUBREADER_SUBWRITER>`` | Indicates if the participant must use |br|                 | ``bool``    | ``true``      |
|                           | Publication DataReader and Subscription DataWriter.        |             |               |
+---------------------------+------------------------------------------------------------+-------------+---------------+

.. _InitAnnounce:

Initial Announcements
#####################

+--------------+-----------------------------------------------------------------------+---------------------+---------+
| Name         | Description                                                           | Values              | Default |
+==============+=======================================================================+=====================+=========+
| ``<count>``  | Number of initial discovery messages to send at the period            | ``uint32_t``        | 5       |
|              | specified by |br|                                                     |                     |         |
|              | ``<period>``. After these announcements, the DomainParticipant will   |                     |         |
|              | continue |br|                                                         |                     |         |
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
     - Default
   * - ``<remote_locators>``
     - Defines the limits for the remote locators' collections. |br|
       See :ref:`remotelocatorsallocationattributes`.
     - :ref:`remote_locators_allocations`
     -
   * - ``<total_participants>``
     - DomainParticipant :ref:`CommonAlloc` to specify the |br|
       total number of DomainParticipants in the domain |br|
       (local and remote). See |br|
       :ref:`ResourceLimitedContainerConfig`.
     - :ref:`CommonAlloc`
     -
   * - ``<total_readers>``
     - DomainParticipant :ref:`CommonAlloc` to specify the |br|
       total number of DataReader on each DomainParticipant |br|
       (local and remote). See |br|
       :ref:`ResourceLimitedContainerConfig`.
     - :ref:`CommonAlloc`
     -
   * - ``<total_writers>``
     - DomainParticipant :ref:`CommonAlloc` related to the |br|
       total number of DataWriters on each DomainParticipant |br|
       (local and remote).
       See :ref:`resourcelimitedcontainerconfig`.
     - :ref:`CommonAlloc`
     -
   * - ``<max_partitions>``
     - Maximum size of the partitions submessage. |br|
       Set to zero for no limit.
     - ``uint32_t``
     -
   * - ``<max_user_data>``
     - Maximum size of the user data submessage. |br|
       Set to zero for no limit.
     - ``uint32_t``
     -
   * - ``<max_properties>``
     - Maximum size of the properties submessage. |br|
       Set to zero for no limit.
     - ``uint32_t``
     -
   * - ``<send_buffers>``
     - Allocation behaviour for the send buffer |br|
       manager.
     - :ref:`SendBuffers`
     -

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
     - Maximum number of unicast locators expected on a |br|
       remote  entity. It is recommended to use the maximum |br|
       number of network interfaces available on the machine |br|
       on which DomainParticipant is running. |br|
       See :ref:`remotelocatorsallocationattributes`.
     - ``uint32_t``
     - 4
   * - ``<max_multicast_locators>``
     - Maximum number of multicast locators expected on a |br|
       remote entity. May be set to zero to disable multicast |br|
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
     - Initial number of send buffers to allocate. See |br|
       :ref:`sendbuffersallocationattributes`.
     - ``uint32_t``
     - 0
   * - ``<dynamic>``
     - Whether the number of send buffers is allowed to grow. |br|
       See :ref:`sendbuffersallocationattributes`.
     - ``bool``
     - false

.. note::
    The default value ``0`` of ``<preallocated_number>`` will perform an initial guess of the number of buffers
    required, based on the number of threads from which a send operation could be started.
    So it does not mean there are no buffers, instead it would use the maximum amount of buffers available.
