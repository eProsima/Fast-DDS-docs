.. _participantprofiles:

Participant profiles
--------------------

Participant profiles allow declaring :ref:`participantconfiguration` from an XML file.
All the configuration options for the participant, except from ``domainId``, belong to the ``<rtps>`` label.
The attribute ``profile_name`` will be the name that the ``Domain`` will associate to the profile to load it
as shown in :ref:`loadingapplyingprofiles`.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT<-->
    :end-before: <!--><-->

List with the configuration tags:

+-----------------------------------+-----------------------------------+----------------------------------+---------+
| Name                              | Description                       | Values                           | Default |
+===================================+===================================+==================================+=========+
| ``<domainId>``                    | DomainId to be used by            | ``UInt32``                       | 0       |
|                                   | the RTPSParticipant.              |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<rtps>``                        | RTPS parameters. Explained        |  `RTPS`_                         |         |
|                                   | in `RTPS`_.                       |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+

.. _RTPS:

RTPS Configuration
^^^^^^^^^^^^^^^^^^


List with the possible configuration parameter:

+-----------------------------------+-----------------------------------+----------------------------------+---------+
| Name                              | Description                       | Values                           | Default |
+===================================+===================================+==================================+=========+
| ``<name>``                        | Participant's name.               | ``string_255``                   |         |
|                                   | It's not the same                 |                                  |         |
|                                   | field that ``profile_name``.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<defaultUnicastLocatorList>``   | List of default input             | ``LocatorListType``              |         |
|                                   | unicast locators.                 |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`LocatorListType`.           |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<defaultMulticastLocatorList>`` | List of default input             | ``LocatorListType``              |         |
|                                   | multicast locators.               |                                  |         |
|                                   | It expects                        |                                  |         |
|                                   | a :ref:`LocatorListType`.         |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<sendSocketBufferSize>``        | Size in bytes of the output       | ``uint32``                       | 0       |
|                                   | socket buffer.                    |                                  |         |
|                                   | If the value is zero then         |                                  |         |
|                                   | FastRTPS will use the default     |                                  |         |
|                                   | size from  the configuration      |                                  |         |
|                                   | of the sockets, using a           |                                  |         |
|                                   | minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<listenSocketBufferSize>``      | Size in bytes of the input        | ``uint32``                       | 0       |
|                                   | socket buffer.                    |                                  |         |
|                                   | If the value is zero then         |                                  |         |
|                                   | FastRTPS will use the default     |                                  |         |
|                                   | size from  the configuration      |                                  |         |
|                                   | of the sockets, using a           |                                  |         |
|                                   | minimum size of 65536 bytes.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<builtin>``                     | Built-in parameters.              | :ref:`builtin`                   |         |
|                                   | Explained in the                  |                                  |         |
|                                   | :ref:`builtin` section.           |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<port>``                        | Allows defining the port          | `Port`_                          |         |
|                                   | parameters and gains related      |                                  |         |
|                                   | to the RTPS protocol.             |                                  |         |
|                                   | Explained in the `Port`_ section. |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<participantID>``               | Participant's identifier.         | ``int32``                        | 0       |
|                                   | Typically it will be              |                                  |         |
|                                   | automatically generated           |                                  |         |
|                                   | by the ``Domain``.                |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<throughputController>``        | Allows defining a maximum         | :ref:`Throughput`                |         |
|                                   | throughput.                       |                                  |         |
|                                   | Explained in the                  |                                  |         |
|                                   | :ref:`Throughput` section.        |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<userTransports>``              | Transport descriptors             | ``List <string>``                |         |
|                                   | to be used by the                 |                                  |         |
|                                   | participant.                      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<useBuiltinTransports>``        | Boolean field to indicate to      | ``bool``                         | true    |
|                                   | the system that the participant   |                                  |         |
|                                   | will use  the default builtin     |                                  |         |
|                                   | transport independently of its    |                                  |         |
|                                   | ``<userTransports>``.             |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<propertiesPolicy>``            | Additional configuration          | :ref:`PropertiesPolicyType`      |         |
|                                   | properties.                       |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`PropertiesPolicyType`.      |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+
| ``<allocation>``                  | Configuration regarding           | :ref:`ParticipantAllocationType` |         |
|                                   | allocation behavior.              |                                  |         |
|                                   | It expects a                      |                                  |         |
|                                   | :ref:`ParticipantAllocationType`  |                                  |         |
+-----------------------------------+-----------------------------------+----------------------------------+---------+

.. | ``<userData>``    | Allows adding custom information.  | ``string``  |         |
.. +-------------------+------------------------------------+-------------+---------+

.. note::

    - :class:`LOCATOR_LIST` means it expects a :ref:`LocatorListType`.

    - :class:`PROPERTIES_POLICY` means that the label is a :ref:`PropertiesPolicyType` block.

    - :class:`DURATION` means it expects a :ref:`DurationType`.

    - For :class:`BUILTIN` details, please refer to :ref:`builtin`.

    - For :class:`ALLOCATION` details, please refer to :ref:`ParticipantAllocationType`.

.. _Port:

Port Configuration
""""""""""""""""""

+-------------------------+-----------------------------+------------+---------+
| Name                    | Description                 | Values     | Default |
+=========================+=============================+============+=========+
| ``<portBase>``          | Base ``port``.              | ``uint16`` | 7400    |
+-------------------------+-----------------------------+------------+---------+
| ``<domainIDGain>``      | Gain in ``domainId``.       | ``uint16`` | 250     |
+-------------------------+-----------------------------+------------+---------+
| ``<participantIDGain>`` | Gain in ``participantId``.  | ``uint16`` | 2       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd0>``          | Multicast metadata offset.  | ``uint16`` | 0       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd1>``          | Unicast metadata offset.    | ``uint16`` | 10      |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd2>``          | Multicast user data offset. | ``uint16`` | 1       |
+-------------------------+-----------------------------+------------+---------+
| ``<offsetd3>``          | Unicast user data offset.   | ``uint16`` | 11      |
+-------------------------+-----------------------------+------------+---------+

.. _ParticipantAllocationType:

Participant allocation parameters
"""""""""""""""""""""""""""""""""

This section of the :class:`Participant's rtps` configuration allows defining parameters related with allocation
behavior on the participant.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PARTICIPANT-ALLOCATION<-->
    :end-before: <!--><-->

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<max_unicast_locators>``
     - Maximum number of unicast locators expected on a remote entity.
       It is recommended to use the maximum number of network interfaces found on any machine the
       participant will connect to.
     - ``UInt32``
     - 4
   * - ``<max_multicast_locators>``
     - Maximum number of multicast locators expected on a remote entity.
       May be set to zero to disable multicast traffic.
     - ``UInt32``
     - 1
   * - ``<total_participants>``
     - Participant :ref:`CommonAlloc` related to the total number of participants in the domain (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_readers>``
     - Participant :ref:`CommonAlloc` related to the total number of readers on each participant (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<total_writers>``
     - Participant :ref:`CommonAlloc` related to the total number of writers on each participant (local and remote).
     - :ref:`CommonAlloc`
     -
   * - ``<max_partitions>``
     - Maximum size of the partitions submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_user_data>``
     - Maximum size of the user data submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -
   * - ``<max_properties>``
     - Maximum size of the properties submessage. Zero for no limit. See :ref:`MessageMaxSize`.
     - ``UInt32``
     -

.. _builtin:

Built-in parameters
"""""""""""""""""""

This section of the :class:`Participant's rtps` configuration allows defining built-in parameters.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-BUILTIN<-->
    :end-before: <!--><-->

.. Some large words outside of table. Then table fit maximum line length
.. |usewriliv| replace:: ``<use_WriterLivelinessProtocol>``
.. |metuniloc| replace:: ``<metatrafficUnicastLocatorList>``
.. |metmulloc| replace:: ``<metatrafficMulticastLocatorList>``
.. |loclist| replace:: List of :ref:`LocatorListType`
.. |mempol| replace:: :ref:`historyMemoryPolicy <mempol>`
.. |readhistmem| replace:: ``<readerHistoryMemoryPolicy>``
.. |writhistmem| replace:: ``<writerHistoryMemoryPolicy>``
.. |readpaysize| replace:: ``<readerPayloadSize>``
.. |writpaysize| replace:: ``<writerPayloadSize>``
.. |mutTries| replace:: ``<mutation_tries>``
.. |mempoldefault| replace:: :class:`PREALLOCATED_WITH_REALLOC`

+--------------------------------+----------------------------------+-------------------------+-----------------------+
| Name                           | Description                      | Values                  | Default               |
+================================+==================================+=========================+=======================+
| ``<discovery_config>``         | This is the main tag where       |                         |                       |
|                                | discovery-related settings can   | :ref:`discovery_config  |                       |
|                                | be configured.                   | <dconf>`                |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| ``<avoid_builtin_multicast>``  | Restricts metatraffic multicast  | ``Boolean``             | :class:`true`         |
|                                | traffic to PDP only.             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |usewriliv|                    | Indicates to use the             | ``Boolean``             | :class:`true`         |
|                                | WriterLiveliness protocol.       |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |metuniloc|                    | Metatraffic Unicast Locator      | |loclist|               |                       |
|                                | List                             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |metmulloc|                    | Metatraffic Multicast Locator    | |loclist|               |                       |
|                                | List                             |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| ``<initialPeersList>``         | Initial peers.                   | |loclist|               |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |readhistmem|                  | Memory policy for builtin        | |mempol|                | |mempoldefault|       |
|                                | readers.                         |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |writhistmem|                  | Memory policy for builtin        | |mempol|                | |mempoldefault|       |
|                                | writers.                         |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |readpaysize|                  | Maximum payload size for         | ``UInt32``              | 512                   |
|                                | builtin readers.                 |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |writpaysize|                  | Maximum payload size for         | ``UInt32``              | 512                   |
|                                | builtin writers.                 |                         |                       |
|                                |                                  |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+
| |mutTries|                     | Number of different ports        | ``UInt32``              | 100                   |
|                                | to try if reader's physical port |                         |                       |
|                                | is already in use.               |                         |                       |
+--------------------------------+----------------------------------+-------------------------+-----------------------+

.. _dconf:

**discovery_config**

.. More large words outside of table. Then table fit maximum line length
.. |staendxml| replace:: ``<staticEndpointXMLFilename>``
.. |protocol| replace:: :class:`SIMPLE`, :class:`CLIENT`, :class:`SERVER`, :class:`BACKUP`
.. |igpartf| replace:: ``<ignoreParticipantFlags>``
.. |filterlist| replace:: :ref:`ignoreParticipantFlags <Participantfiltering>`

+----------------------------+---------------------------------------+-------------------------+----------------------+
| Name                       | Description                           | Values                  | Default              |
+============================+=======================================+=========================+======================+
| ``<discoveryProtocol>``    | Indicates which kind of PDP protocol  | |protocol|              | :class:`SIMPLE`      |
|                            | the participant must use.             |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| |igpartf|                  | Restricts metatraffic using           | |filterlist|            | :class:`NO_FILTER`   |
|                            | several filtering criteria.           |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<EDP>``                  | - If set to :class:`SIMPLE`,          | :class:`SIMPLE`,        | :class:`SIMPLE`      |
|                            |   ``<simpleEDP>`` would be used.      | :class:`STATIC`         |                      |
|                            | - If set to :class:`STATIC`,          |                         |                      |
|                            |   StaticEDP based on an XML           |                         |                      |
|                            |   file would be used with the         |                         |                      |
|                            |   contents of                         |                         |                      |
|                            |   ``<staticEndpointXMLFilename>``.    |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<simpleEDP>``            | Attributes of the SimpleEDP           | :ref:`simpleEDP <sedp>` |                      |
|                            | protocol                              |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<leaseDuration>``        | Indicates how long this               |  :ref:`DurationType`    |  20 s                |
|                            | RTPSParticipant should consider       |                         |                      |
|                            | remote RTPSParticipants alive.        |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<leaseAnnouncement>``    | The period for the RTPSParticipant    |  :ref:`DurationType`    |  3 s                 |
|                            | to send its Discovery Message to all  |                         |                      |
|                            | other discovered RTPSParticipants     |                         |                      |
|                            | as well as to all Multicast ports.    |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| ``<initialAnnouncements>`` | Allows the user to configure          |  :ref:`Initial          |                      |
|                            | the number and period of the initial  |  Announcements          |                      |
|                            | RTPSparticipant's Discovery messages. |  <InitAnnounce>`        |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+
| |staendxml|                | StaticEDP XML filename.               | ``string``              |                      |
|                            | Only necessary if ``<EDP>``           |                         |                      |
|                            | is set to :class:`STATIC`             |                         |                      |
+----------------------------+---------------------------------------+-------------------------+----------------------+

.. _Participantfiltering:

**ignoreParticipantFlags**

+-----------------------------------------------------------+------------------------+
| Possible values                                           | Description            |
+===========================================================+========================+
| :class:`NO_FILTER`                                        | All Discovery traffic  |
|                                                           | is processed           |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_HOST`                            | Discovery traffic from |
|                                                           | another host is        |
|                                                           | discarded              |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_PROCESS`                         | Discovery traffic from |
|                                                           | another process on the |
|                                                           | same host is discarded |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_SAME_PROCESS`                              | Discovery traffic from |
|                                                           | participant's own      |
|                                                           | process is discarded.  |
+-----------------------------------------------------------+------------------------+
| :class:`FILTER_DIFFERENT_PROCESS | FILTER_SAME_PROCESS`   | Discovery traffic from |
|                                                           | participant's own host |
|                                                           | is discarded.          |
+-----------------------------------------------------------+------------------------+

.. _sedp:

**simpleEDP**

+---------------------------+---------------------------------------------+-------------+---------------+
| Name                      | Description                                 | Values      | Default       |
+===========================+=============================================+=============+===============+
| ``<PUBWRITER_SUBREADER>`` | Indicates if the participant must use       | ``Boolean`` | :class:`true` |
|                           | Publication Writer and Subscription Reader. |             |               |
+---------------------------+---------------------------------------------+-------------+---------------+
| ``<PUBREADER_SUBWRITER>`` | Indicates if the participant must use       | ``Boolean`` | :class:`true` |
|                           | Publication Reader and Subscription Writer. |             |               |
+---------------------------+---------------------------------------------+-------------+---------------+

.. _InitAnnounce:

**Initial Announcements**

+---------------------------+---------------------------------------------+---------------------+---------------+
| Name                      | Description                                 | Values              | Default       |
+===========================+=============================================+=====================+===============+
| ``<count>``               | Number of Discovery Messages to send at the | ``Uint32``          | 5             |
|                           | period specified by ``<period>``.           |                     |               |
|                           | After these announcements, the              |                     |               |
|                           | RTPSParticipant will continue sending its   |                     |               |
|                           | Discovery Messages at the                   |                     |               |
|                           | ``<leaseAnnouncement>`` rate.               |                     |               |
+---------------------------+---------------------------------------------+---------------------+---------------+
| ``<period>``              | The period for the RTPSParticipant to send  | :ref:`DurationType` | 100 ms        |
|                           | its first ``<count>`` Discovery Messages.   |                     |               |
+---------------------------+---------------------------------------------+---------------------+---------------+
