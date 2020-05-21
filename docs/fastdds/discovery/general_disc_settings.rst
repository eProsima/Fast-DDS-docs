.. include:: includes/aliases.rst

.. _discovery_general_settings:

General Discovery Settings
--------------------------

Some discovery settings are shared across the different discovery mechanisms.
Those are:

+-------------------------------+------------------------------------+---------------------------------+---------------+
| Name                          | Description                        | Type                            |     Default   |
+===============================+====================================+=================================+===============+
| :ref:`discovery_protocol`     | The discovery protocol to use |br| | ``DiscoveryProtocol_t``         | ``SIMPLE``    |
|                               | (see :ref:`disc_mechanisms`).      |                                 |               |
+-------------------------------+------------------------------------+---------------------------------+---------------+
| :ref:`discovery_ignore_flags` | Filter discovery traffic for |br|  | ``ParticipantFilteringFlags_t`` | ``NO_FILTER`` |
|                               | DomainParticipants in the |br|     |                                 |               |
|                               | same process, in different |br|    |                                 |               |
|                               | processes, or in different hosts.  |                                 |               |
+-------------------------------+------------------------------------+---------------------------------+---------------+
| :ref:`discovery_lease_dur`    | Indicates for how much time |br|   | ``Duration_t``                  |     20 s      |
|                               | should a remote                    |                                 |               |
|                               | DomainParticipant |br| consider    |                                 |               |
|                               | the local DomainParticipant |br|   |                                 |               |
|                               | to be alive.                       |                                 |               |
+-------------------------------+------------------------------------+---------------------------------+---------------+
| :ref:`discovery_lease_announ` | The period for the                 | ``Duration_t``                  |     3 s       |
|                               | DomainParticipant |br|             |                                 |               |
|                               | to send PDP announcements.         |                                 |               |
+-------------------------------+------------------------------------+---------------------------------+---------------+

.. _discovery_protocol:

Discovery Protocol
^^^^^^^^^^^^^^^^^^

Specifies the discovery protocol to use (see :ref:`disc_mechanisms`).
The possible values are:

+---------------------+---------------------+--------------------------------------------------------------------------+
| Discovery Mechanism | Possible values     | Description                                                              |
+=====================+=====================+==========================================================================+
| Simple              | ``SIMPLE``          | Simple discovery protocol as specified in                                |
|                     |                     | `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_            |
+---------------------+---------------------+--------------------------------------------------------------------------+
| Static              | ``STATIC``          | SPDP with manual EDP specified in XML files.                             |
+---------------------+---------------------+--------------------------------------------------------------------------+
| Server-Client       | ``SERVER``          | The DomainParticipant acts as a hub for discovery traffic, receiving     |
|                     |                     | |br| and distributing discovery information.                             |
|                     +---------------------+--------------------------------------------------------------------------+
|                     | ``CLIENT``          | The DomainParticipant acts as a client for discovery traffic. |br|       |
|                     |                     | It send its discovery information to the server, and receives |br|       |
|                     |                     | all other discovery information from the server.                         |
|                     +---------------------+--------------------------------------------------------------------------+
|                     | ``BACKUP``          | Creates a SERVER DomainParticipant which has a persistent ``sqlite``     |
|                     |                     | |br| database. A BACKUP server can load the a database on start. |br|    |
|                     |                     | This type of sever makes the Server-Client architecture resilient |br|   |
|                     |                     | to server destruction.                                                   |
+---------------------+---------------------+--------------------------------------------------------------------------+
| Manual              | ``NONE``            | Disables PDP phase, therefore the is no EDP phase. |br|                  |
|                     |                     | All matching must be done manually through the |br|                      |
|                     |                     | ``addReaderLocator``, ``addReaderProxy``, ``addWriterProxy`` |br|        |
|                     |                     | RTPS layer methods.                                                      |
+---------------------+---------------------+--------------------------------------------------------------------------+

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF-DISCOVERY-PROTOCOL                                                                           |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-DISCOVERY-PROTOCOL                                                                        |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _discovery_ignore_flags:

Ignore Participant flags
^^^^^^^^^^^^^^^^^^^^^^^^

Defines a filter to ignore some discovery traffic when received.
This is useful to add an extra level of DomainParticipant isolation.
The possible values are:

+-----------------------------------------------+----------------------------------------------------------------------+
| Possible values                               | Description                                                          |
+===============================================+======================================================================+
| ``NO_FILTER``                                 | All Discovery traffic is processed.                                  |
+-----------------------------------------------+----------------------------------------------------------------------+
| ``FILTER_DIFFERENT_HOST``                     | Discovery traffic from another host is discarded.                    |
+-----------------------------------------------+----------------------------------------------------------------------+
| ``FILTER_DIFFERENT_PROCESS``                  | Discovery traffic from another process on the same host is           |
|                                               | discarded,                                                           |
+-----------------------------------------------+----------------------------------------------------------------------+
| ``FILTER_SAME_PROCESS``                       | Discovery traffic from DomainParticipant's own process is discarded. |
+-----------------------------------------------+----------------------------------------------------------------------+
| ``FILTER_DIFFERENT_PROCESS``                  | Discovery traffic from DomainParticipant's own host is discarded.    |
| ``FILTER_SAME_PROCESS``                       |                                                                      |
+-----------------------------------------------+----------------------------------------------------------------------+

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF-DISCOVERY-IGNORE-FLAGS                                                                       |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-DISCOVERY-IGNORE-FLAGS                                                                    |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _discovery_lease_dur:

Lease Duration
^^^^^^^^^^^^^^

Indicates for how much time should a remote DomainParticipant consider the local DomainParticipant to be alive.
If the liveliness of the local DomainParticipant has not being asserted within this time, the remote DomainParticipant
considers the local DomainParticipant dead and destroys all the information regarding the local DomainParticipant and
all its endpoints.

The local DomainParticipant's liveliness is asserted on the remote DomainParticipant any time the remote
DomainParticipant receives any kind of traffic from the local DomainParticipant.

The lease duration is specified as a time expressed in seconds and nanosecond using a ``Duration_t``.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF-DISCOVERY-LEASE-DURATION                                                                     |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-DISCOVERY-LEASE-DURATION                                                                  |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. _discovery_lease_announ:

Announcement Period
^^^^^^^^^^^^^^^^^^^

It specifies the periodicity of the DomainParticipant's PDP announcements.  For liveliness' sake it is recommend that
the announcement period is shorter than the lease duration, so that the DomainParticipant's liveliness is asserted even
when there is no data traffic.  It is important to note that there is a trade-off involved in the setting of the
announcement period, i.e. too frequent announcements will bloat the network with meta traffic, but too scarce ones will
delay the discovery of late joiners.

DomainParticipant's announcement period is specified as a time expressed in seconds and nanosecond using a ``Duration_t``.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: //CONF-DISCOVERY-LEASE-ANNOUNCEMENT                                                                 |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->CONF-DISCOVERY-LEASE-ANNOUNCEMENT                                                              |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+
