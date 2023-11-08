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
      - ``dds.ev.<participant_id>``
      - Processes periodic and triggered time events. |br|
        See :ref:`dds_layer_domainParticipantQos`.
    * - Discovery Server Event
      - General
      - One per DomainParticipant
      - ``dds.ds_ev.<participant_id>``
      - Synchronizes access to the Discovery Server |br| Database. |br|
        See :ref:`dds_layer_domainParticipantQos`.
    * - Asynchronous Writer
      - General
      - One per enabled asynchronous |br| flow controller.
        Minimum 1.
      - ``dds.asyn.<participant_id>.``
        ``<async_flow_controller_index>``
      - Manages asynchronous writes.
        Even for synchronous writers, some forms of |br| communication must be initiated in the |br| background. |br|
        See :ref:`dds_layer_domainParticipantQos` and :ref:`flowcontrollersqos`.
    * - Datasharing Listener
      - General
      - One per |br| DataReader
      - ``dds.dsha.<reader_id>``
      - Listener thread that processes messages |br| received via Datasharing. |br|
        See :ref:`dds_layer_subscriber_dataReaderQos`.
    * - Reception
      - UDP
      - One per port
      - ``dds.udp.<port>``
      - Listener thread that processes incoming |br| UDP messages. |br|
        See :ref:`transportconfigqos` and :ref:`transport_udp_transportDescriptor`.
    * - Reception
      - TCP
      - One per TCP connection
      - ``dds.tcp.<port>``
      - Listener thread that processes incoming |br| TCP messages. |br|
        See :ref:`transport_tcp_transportDescriptor`.
    * - Accept
      - TCP
      - One per TCP transport
      - ``dds.tcp_accept``
      - Thread that processes incoming TCP connection requests. |br|
        See :ref:`transport_tcp_transportDescriptor`.
    * - Keep Alive
      - TCP
      - One per TCP transport
      - ``dds.tcp_keep``
      - Keep alive thread for TCP connections. |br|
        See :ref:`transport_tcp_transportDescriptor`.
    * - Reception
      - SHM
      - One per port
      - ``dds.shm.<port>``
      - Listener thread that processes incoming |br| messages via SHM segments. |br|
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - Logging
      - SHM
      - One per port
      - ``dds.shmd.<port>``
      - Stores and dumps transferred packets to a file. |br|
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - Watchdog
      - SHM
      - One
      - ``dds.shm.wdog``
      - Monitors health of open shared memory |br| segments. |br|
        See :ref:`transportconfigqos` and :ref:`transport_sharedMemory_transportDescriptor`.
    * - General Logging
      - Log
      - One
      - ``dds.log``
      - Accumulates and writes to the appropriate |br| consumer log entries. |br|
        See :ref:`dds_layer_log_thread`.
    * - Security Logging
      - Log
      - One per |br| DomainParticipant
      - ``dds.slog.<participant_id>``
      - Accumulates and writes security log entries. |br|
        See :ref:`dds_layer_domainParticipantQos`.
    * - Watchdog
      - Filewatch
      - One
      - ``dds.fwatch``
      - Tracks the status of the watched file for |br| modifications. |br|
        See :ref:`dds_layer_domainParticipantFactoryQos`.
    * - Callback
      - Filewatch
      - One
      - ``dds.fwatch.cb``
      - Runs the registered callback when the |br| watched file changes. |br|
        See :ref:`dds_layer_domainParticipantFactoryQos`.

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
