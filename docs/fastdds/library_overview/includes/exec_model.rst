Programming and execution model
-------------------------------

*Fast DDS* is concurrent and event-based.
The following explains the multithreading model that governs the operation of *Fast DDS* as well as the possible events.

.. _concurrency_multithreading:

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
      - OS thread name
      - Description
    * - Event
      - General
      - One per DomainParticipant
      - :code:`dds.ev.<participant_id>`
      - Processes periodic and triggered time events. 
        See :ref:`dds_layer_domainParticipantQos`.
    * - Discovery Server Event
      - General
      - One per DomainParticipant
      - :code:`dds.ds_ev.<participant_id>`
      - Synchronizes access to the Discovery Server  Database. 
        See :ref:`dds_layer_domainParticipantQos`.
    * - Asynchronous Writer
      - General
      - One per enabled asynchronous  flow controller.
        Minimum 1.
      - :code:`dds.asyn.<participant_id>.` 
        :code:`<async_flow_controller_index>`
      - Manages asynchronous writes.
        Even for synchronous writers, some forms of  communication must be initiated in the  background. 
        See :ref:`dds_layer_domainParticipantQos` and :ref:`flowcontrollersqos`.
    * - Datasharing Listener
      - General
      - One per  DataReader
      - :code:`dds.dsha.<reader_id>`
      - Listener thread that processes messages  received via Datasharing. 
        See :ref:`dds_layer_subscriber_dataReaderQos`.
    * - Reception
      - UDP
      - One per port
      - :code:`dds.udp.<port>`
      - Listener thread that processes incoming  UDP messages. 
        See :ref:`transportconfigqos` and :ref:`transport_udp_transportDescriptor`.
    * - Reception
      - TCP
      - One per TCP connection
      - :code:`dds.tcp.<port>`
      - Listener thread that processes incoming  TCP messages. 
        See :ref:`transport_tcp_transportDescriptor`.
    * - Accept
      - TCP
      - One per TCP transport
      - :code:`dds.tcp_accept`
      - Thread that processes incoming TCP connection requests. 
        See :ref:`transport_tcp_transportDescriptor`.
    * - Keep Alive
      - TCP
      - One per TCP transport
      - :code:`dds.tcp_keep`
      - Keep alive thread for TCP connections. 
        See :ref:`transport_tcp_transportDescriptor`.
    * - Reception
      - SHM
      - One per port
      - :code:`dds.shm.<port>`
      - Listener thread that processes incoming  messages via SHM segments. 
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - Logging
      - SHM
      - One per port
      - :code:`dds.shmd.<port>`
      - Stores and dumps transferred packets to a file. 
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - Watchdog
      - SHM
      - One
      - :code:`dds.shm.wdog`
      - Monitors health of open shared memory  segments. 
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - General Logging
      - Log
      - One
      - :code:`dds.log`
      - Accumulates and writes to the appropriate  consumer log entries. 
        See :ref:`dds_layer_log_thread`.
    * - Security Logging
      - Log
      - One per  DomainParticipant
      - :code:`dds.slog.<participant_id>`
      - Accumulates and writes security log entries. 
        See :ref:`dds_layer_domainParticipantQos`.
    * - Watchdog
      - Filewatch
      - One
      - :code:`dds.fwatch`
      - Tracks the status of the watched file for  modifications. 
        See :ref:`dds_layer_domainParticipantFactoryQos`.
    * - Callback
      - Filewatch
      - One
      - :code:`dds.fwatch.cb`
      - Runs the registered callback when the  watched file changes. 
        See :ref:`dds_layer_domainParticipantFactoryQos`.
    * - Reception
      - TypeLookup Service
      - Two per DomainParticipant
      - :code:`dds.tls.replies.<participant_id>` 
        :code:`dds.tls.requests.<participant_id>`
      - Runs when remote endpoint discovery information has been received 
        with unknown data type.

Some of these threads are only spawned when certain conditions are met:

* Datasharing listener thread is created only when Datasharing is in use.
* Discovery Server Event thread is only created when the DomainParticipant is configured as a Discovery Server SERVER.
* TCP keep alive thread requires the keep alive period to be configured to a value greater than zero.
* Security logging and Shared Memory packet logging threads both require certain configuration options to be enabled.
* Filewatch threads are only spawned if the :ref:`env_vars_fastdds_environment_file` is in use.

Regarding transport threads, Fast DDS by default uses both a UDP and a Shared Memory transport.
Port configuration can be configured to suit the specific needs of the deployment,
but the default configuration is to always use a metatraffic port and a unicast user traffic port.
This applies both to UDP and Shared Memory since TCP does not support multicast.
More information can be found at the :ref:`listening_locators_default` page.

*Fast DDS* offers the possibility of configuring certain attributes of the threads it creates by means of the
:ref:`threadsettingsqos`.

Event-driven architecture
^^^^^^^^^^^^^^^^^^^^^^^^^

There is a time-event system that enables *Fast DDS* to respond to certain conditions, as well as schedule periodic
operations.
Few of them are visible to the user since most are related to DDS and RTPS metadata.
However, the user can define in their application periodic time-events by inheriting from the :class:`TimedEvent`
class.
