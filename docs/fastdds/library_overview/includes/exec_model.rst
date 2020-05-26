Programming and execution model
-------------------------------

Fast DDS is concurrent and event-based.
The following explains the multithreading model that governs the operation of Fast DDS as well as the possible events.

Concurrency and multithreading
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS implements a concurrent multithreading system.
Each DomainParticipant spawns a set of threads to take care of background tasks such as logging, message reception, and
asynchronous communication.
This should not impact the way you use the library, i.e. the Fast DDS API is thread safe, so you can fearlessly call any
methods on the same DomainParticipant from different threads.
However, this multithreading implementation must be taken into account when external functions access to resources that
are modified by threads running internally in the library.
An example of this is the modified resources in the entity listener callbacks.
The following is a brief overview of how Fast DDS multithreading schedule work:

* Main thread: Managed by the application.
* Event thread: Each DomainParticipant owns one of these. It processes periodic and triggered time events.
* Asynchronous writer thread: This thread manages asynchronous writes for all DomainParticipants.
  Even for synchronous writers, some forms of communication must be initiated in the background.
* Reception threads: DomainParticipants spawn a thread for each reception channel, where the concept of a channel
  depends on the transport layer (e.g. a UDP port).

Event-driven architecture
^^^^^^^^^^^^^^^^^^^^^^^^^

There is a time-event system that enables Fast DDS to respond to certain conditions, as well as schedule periodic
operations.
Few of them are visible to the user since most are related to DDS and RTPS metadata.
However, the user can define in their application periodic time-events by inheriting from the :class:`TimedEvent`
class.
