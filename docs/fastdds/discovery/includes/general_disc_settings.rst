.. _discovery_general_settings:

General discovery settings
--------------------------

Some discovery settings are shared across the different discovery mechanisms.
Those are:

+-------------------------------+-----------------------------------+---------------------------------+---------------+
| Name                          | Description                       | Type                            |     Default   |
+===============================+===================================+=================================+===============+
| :ref:`discovery_protocol`     | The discovery protocol to use     | ``DiscoveryProtocol_t``         | ``SIMPLE``    |
|                               | (see :ref:`disc_mechanisms`)      |                                 |               |
+-------------------------------+-----------------------------------+---------------------------------+---------------+
| :ref:`discovery_ignore_flags` | Filter discovery traffic for      | ``ParticipantFilteringFlags_t`` | ``NO_FILTER`` |
|                               | participants in the same process, |                                 |               |
|                               | in different processes,           |                                 |               |
|                               | or in different hosts             |                                 |               |
+-------------------------------+-----------------------------------+---------------------------------+---------------+
| :ref:`discovery_lease_dur`    | Indicates for how much time       | ``Duration_t``                  |     20 s      |
|                               | should a remote participant       |                                 |               |
|                               | consider the local participant    |                                 |               |
|                               | to be alive.                      |                                 |               |
+-------------------------------+-----------------------------------+---------------------------------+---------------+
| :ref:`discovery_lease_announ` | The period for the participant    | ``Duration_t``                  |     3 s       |
|                               | to send PDP announcements.        |                                 |               |
+-------------------------------+-----------------------------------+---------------------------------+---------------+

.. _discovery_protocol:

Discovery Protocol
^^^^^^^^^^^^^^^^^^

Specifies the discovery protocol to use (see :ref:`disc_mechanisms`).
The possible values are:

+---------------------+---------------------+-------------------------------------------------------------------------+
| Discovery Mechanism | Possible values     | Description                                                             |
+=====================+=====================+=========================================================================+
| Simple              | ``SIMPLE``          | Simple discovery protocol as specified in                               |
|                     |                     | `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_           |
+---------------------+---------------------+-------------------------------------------------------------------------+
| Static              | ``STATIC``          | SPDP with manual EDP specified in XML files                             |
+---------------------+---------------------+-------------------------------------------------------------------------+
| Server-Client       | ``SERVER``          | The participant acts as a hub for discovery traffic, receiving and      |
|                     |                     | distributing discovery information.                                     |
|                     +---------------------+-------------------------------------------------------------------------+
|                     | ``CLIENT``          | The participant acts as a client for discovery traffic.                 |
|                     |                     | It send its discovery information to the server, and receives all other |
|                     |                     | discovery information from the server.                                  |
|                     +---------------------+-------------------------------------------------------------------------+
|                     | ``BACKUP``          | Creates a SERVER participant which has a persistent ``sqlite`` database.|
|                     |                     | A BACKUP server can load the a database on start.                       |
|                     |                     | This type of sever makes the Server-Client architecture resilient to    |
|                     |                     | server destruction.                                                     |
+---------------------+---------------------+-------------------------------------------------------------------------+
| Manual              | ``NONE``            | Disables PDP phase, therefore the is no EDP phase.                      |
|                     |                     | All matching must be done manually through the ``addReaderLocator``,    |
|                     |                     | ``addReaderProxy``, ``addWriterProxy`` methods.                         |
+---------------------+---------------------+-------------------------------------------------------------------------+

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp         |
|    :language: c++                                   |
|    :start-after: //CONF-DISCOVERY-PROTOCOL          |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-DISCOVERY-PROTOCOL       |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+

.. _discovery_ignore_flags:

Ignore Participant flags
^^^^^^^^^^^^^^^^^^^^^^^^

Defines a filter to ignore some discovery traffic when received.
This is useful to add an extra level of participant isolation.
The possible values are:

+----------------------------------------------------+----------------------------------------------------------------+
| Possible values                                    | Description                                                    |
+====================================================+================================================================+
| ``NO_FILTER``                                      | All Discovery traffic is processed.                            |
+----------------------------------------------------+----------------------------------------------------------------+
| ``FILTER_DIFFERENT_HOST``                          | Discovery traffic from another host is discarded.              |
+----------------------------------------------------+----------------------------------------------------------------+
| ``FILTER_DIFFERENT_PROCESS``                       | Discovery traffic from another process on the same host is     |
|                                                    | discarded,                                                     |
+----------------------------------------------------+----------------------------------------------------------------+
| ``FILTER_SAME_PROCESS``                            | Discovery traffic from participant's own process is discarded. |
+----------------------------------------------------+----------------------------------------------------------------+
| ``FILTER_DIFFERENT_PROCESS | FILTER_SAME_PROCESS`` | Discovery traffic from participant's own host is discarded.    |
+----------------------------------------------------+----------------------------------------------------------------+

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp         |
|    :language: c++                                   |
|    :start-after: //CONF-DISCOVERY-IGNORE-FLAGS      |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-DISCOVERY-IGNORE-FLAGS   |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+

.. _discovery_lease_dur:

Lease Duration
^^^^^^^^^^^^^^

Indicates for how much time should a remote participant consider the local participant to be alive.
If the liveliness of the local participant has not being asserted within this time, the remote participant considers the
local participant dead and destroys all the information regarding the local participant and all its endpoints.

The local participant's liveliness is asserted on the remote participant any time the remote participant receives any
kind of traffic from the local participant.

The lease duration is specified as a time expressed in seconds and nanosecond using a ``Duration_t``.

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp         |
|    :language: c++                                   |
|    :start-after: //CONF-DISCOVERY-LEASE-DURATION    |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-DISCOVERY-LEASE-DURATION |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+

.. _discovery_lease_announ:

Announcement Period
^^^^^^^^^^^^^^^^^^^

It specifies the periodicity of the participant's PDP announcements.  For liveliness' sake it is recommend that the
announcement period is shorter than the lease duration, so that the participant's liveliness is asserted even when there
is no data traffic.  It is important to note that there is a trade-off involved in the setting of the announcement
period, i.e. too frequent announcements will bloat the network with meta traffic, but too scarce ones will delay the
discovery of late joiners.

Participant's announcement period is specified as a time expressed in seconds and nanosecond using a ``Duration_t``.

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp             |
|    :language: c++                                       |
|    :start-after: //CONF-DISCOVERY-LEASE-ANNOUNCEMENT    |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF-DISCOVERY-LEASE-ANNOUNCEMENT |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+
