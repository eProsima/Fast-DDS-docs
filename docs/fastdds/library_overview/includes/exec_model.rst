Programming and execution model
-------------------------------

*Fast DDS* is concurrent and event-based.
The following explains the multithreading model that governs the operation of *Fast DDS* as well as the possible events.

Concurrency and multithreading
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS* implements a concurrent multithreading system.
Each DomainParticipant spawns a set of threads to take care of background tasks such as logging, message reception, and
asynchronous communication.
This should not impact the way you use the library, i.e. the *Fast DDS* API is thread safe, so you can fearlessly call
any methods on the same DomainParticipant from different threads.
However, this multithreading implementation must be taken into account when external functions access to resources that
are modified by threads running internally in the library.
An example of this is the modified resources in the entity listener callbacks.

The complete set of threads spawned by Fast DDS is shown below.
Transport related threads (marked as UDP, TCP and SHM types) are only created when the appropriate Transport is used.

.. list-table::
    :header-rows: 1
    :align: left

    * - Name
      - Type
      - Cardinality
      - Description
    * - Event
      - General
      - One per DomainParticipant
      - Processes periodic and triggered time events
    * - Asynchronous Writer
      - General
      - One
      - Manages asynchronous writes for all DomainParticipants.

        Even for synchronous writers, some forms of communication must be initiated in the background.
    * - Datasharing Listener
      - General
      - One per DomainParticipant
      - Listener thread that processes messages received via Datasharing
    * - Reception
      - UDP
      - One per port
      - Listener thread that processes incoming UDP messages
    * - Reception
      - TCP
      - One per port
      - Listener thread that processes incoming TCP messages
    * - Keep Alive
      - TCP
      - One per port
      - Keep alive thread for TCP connections.
    * - Reception
      - SHM
      - One per port
      - Listener thread that processes incoming messages via SHM segments
    * - Logging
      - SHM
      - One per SHM descriptor
      - Stores and dumps transferred packets to a file.
    * - Watchdog
      - SHM
      - One
      - Monitors health of open shared memory segments.
    * - General Logging
      - Log
      - One
      - Accumulates and writes to the appropriate consumer log entries.
    * - Security Logging
      - Log
      - One
      - Accumulates and writes security log entries.

Some of these threads are only spawned when certain conditions are met.
Datasharing listener thread is created only when Datasharing is in use.
TCP keep alive thread requires the keep alive period to be configured to a value greater than zero.
Security logging and Shared Memory packet logging threads both require certain configuration options to be enabled.

Event-driven architecture
^^^^^^^^^^^^^^^^^^^^^^^^^

There is a time-event system that enables *Fast DDS* to respond to certain conditions, as well as schedule periodic
operations.
Few of them are visible to the user since most are related to DDS and RTPS metadata.
However, the user can define in their application periodic time-events by inheriting from the :class:`TimedEvent`
class.
