.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _RTPS standard: https://www.omg.org/spec/DDSI-RTPS/2.2

.. _rtps_layer:

RTPS Layer
==========

The lower level RTPS Layer of *eprosima Fast DDS* serves an implementation of the protocol defined in the
`RTPS standard`_.
This layer provides more control over the internals of the communication protocol than the :ref:`dds_layer`, so advanced
users have finer control over the library's functionalities.


Relation to the DDS Layer
-------------------------

Elements of this layer map one-to-one with elements from the :ref:`dds_layer`, with a few additions.
This correspondence is shown in the following table:

+----------------------------------------+-------------------+
| :ref:`dds_layer`                       | :ref:`rtps_layer` |
+========================================+===================+
| :ref:`dds_layer_domain`                | RTPSDomain        |
+----------------------------------------+-------------------+
| :ref:`dds_layer_domainParticipant`     | RTPSParticipant   |
+----------------------------------------+-------------------+
| :ref:`dds_layer_publisher_dataWriter`  | RTPSWriter        |
+----------------------------------------+-------------------+
| :ref:`dds_layer_subscriber_dataReader` | RTPSReader        |
+----------------------------------------+-------------------+

How to use the RTPS Layer
-------------------------

We will now go over the use of the RTPS Layer like we did with the :ref:`dds_layer` one,
explaining the new features it presents.

We recommend you to look at the two examples describing how to use the RTPS layer that come with the distribution
while reading this section.
They are located in
`examples/cpp/rtps/AsSocket <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/rtps/AsSocket>`_ and
`examples/cpp/rtps/Registered <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/rtps/Registered>`_

Managing the Participant
^^^^^^^^^^^^^^^^^^^^^^^^

Creating a |RTPSParticipant-api| is done with |RTPSDomain::createParticipant-api|.
|RTPSParticipantAttributes-api| structure is used to configure the :class:`RTPSParticipant` upon creation.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_CREATE_PARTICIPANT
    :end-before: //!--
    :dedent: 4

Managing the Writers and Readers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the RTPS standard specifies, |RTPSWriters-api| and |RTPSReaders-api| are always associated
with a |History-api| element.
In the :ref:`dds_layer`, its creation and management is hidden,
but in the :ref:`rtps_layer`, you have full control over its creation and configuration.

Writers are created with |RTPSDomain::createRTPSWriter-api| and configured with a |WriterAttributes-api| structure.
They also need a |WriterHistory-api| which is configured with a |HistoryAttributes-api| structure.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_HISTORY
    :end-before: //!--
    :dedent: 4

Similar to the creation of Writers, Readers are created with |RTPSDomain::createRTPSReader-api|
and configured with a |ReaderAttributes-api| structure.
A |HistoryAttributes-api| structure is used to configure the required |ReaderHistory-api|.
Note that in this case, you can provide a specialization of |ReaderListener-api| class that implements your
callbacks:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_CONF_HISTORY
    :end-before: //!--
    :dedent: 8

Using the History to Send and Receive Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the RTPS Protocol, Readers and Writers save the data about a topic in their associated Histories.
Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as |CacheChange_t-api|.
Changes are always managed by the History.

You can add a new :class:`CacheChange_t` to the History of the Writer to send data.
The procedure is as follows:

1. Request a :class:`CacheChange_t` from the Writer with |RTPSWriters::new_change-api|.
   In order to allocate enough memory,
   you need to provide a callback that returns the maximum number bytes in the payload.
2. Fill the :class:`CacheChange_t` with the data.
3. Add it to the History with |WriterHistory::add_change-api|.

The Writer will take care of everything to communicate the data to the Readers.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITE_SAMPLE
    :end-before: //!--
    :dedent: 4

If your topic data type has several fields, you will have to provide functions to serialize and deserialize
your data in and out of the :class:`CacheChange_t`.
*Fast DDS-Gen* does this for you.

You can receive data from within the |ReaderListener::onNewCacheChangeAdded-api| callback,
as we did in the :ref:`dds_layer`:

1. The callback receives a :class:`CacheChange_t` parameter containing the received data.
2. Process the data within the received :class:`CacheChange_t`.
3. Inform the Reader's History that the change is not needed anymore.


.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_LISTENER
    :end-before: //!--

.. _rtps_layer_builtin_transports:

Managing the Builtin Transports
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DDS uses the :ref:`comm-transports-configuration` to allow communication between DDS entities. *eProsima Fast DDS* comes
with five transports already implemented. However, these transports are not always exclusive between them
and in some cases they can be used simultaneously.

You can choose what transports you want to use by disabling the use of builtin transports and manually
adding them (see :ref:`transportconfigqos`) or using the default builtin transports behavior and selecting
one of the configuration options listed below. Each option modifies the kind of transports that will be
instantiated.

+----------------------------+------------------------------------------------------------------------------+
| Builtin Transports Options | Description                                                                  |
+============================+==============================================================================+
| ``NONE``                   | No transport will be instantiated. Hence, the user must manually add         |
|                            | the desired |br| transports. Otherwise, the participant creation will fail.  |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULT``                | UDPv4 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4 |br| transport. Meaning that SHM will always be used          |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULTv6``              | UDPv6 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4 |br| transport. Meaning that SHM will always be used          |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``SHM``                    | Only a SHM transport will be instantiated.                                   |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv4``                  | Only a UDPv4 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv6``                  | Only a UDPv6 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``LARGE_DATA``             | UDPv4, TCPv4, and SHM transports will be instantiated. However, UDP will     |
|                            | only be used |br| for multicast announcements during the participant         |
|                            | discovery phase (see :ref:`disc_phases`) |br| while the participant          |
|                            | liveliness and the application data delivery occurs over TCP or SHM. |br|    |
|                            | This configuration is useful when working with large data.(See               |
|                            | :ref:`use-case-tcp`).                                                        |
+----------------------------+------------------------------------------------------------------------------+

.. literalinclude:: ../../../code/CodeTester.cpp
  :language: c++
  :start-after: //RTPS_SETUP_TRANSPORTS_EXAMPLE
  :end-before: //!--
  :dedent: 4

The same result can also be obtained using the |DomainParticipantQoS::setup_transports-api| wrapper
function of the :ref:`dds_layer_domainParticipantQos`, XML profiles (see :ref:`RTPS`) or the
``FASTDDS_BUILTIN_TRANSPORTS`` environment variable (see :ref:`env_vars_builtin_transports`).

.. note::
     TCPv4 transport is initialized with the following configuration:

     * |TCPTransportDescriptor::calculate_crc-api|, |TCPTransportDescriptor::check_crc-api| and
       |TCPTransportDescriptor::apply_security-api| are set to false.
     * |TCPTransportDescriptor::enable_tcp_nodelay-api| is set to true.
     * |TCPTransportDescriptor::keep_alive_thread-api| and
       |TCPTransportDescriptor::accept_thread-api| use the default configuration.

Configuring Readers and Writers
-------------------------------
One of the benefits of using the :ref:`rtps_layer` is that it provides new configuration possibilities while
maintaining the options from the DDS layer.
For example, you can set a Writer or a Reader as a Reliable or Best-Effort endpoint as previously:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_RELIABILITY
    :end-before: //!--
    :dedent: 4

.. _SettingDataDurability:

Setting the data durability kind
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Durability parameter defines the behavior of the Writer regarding samples already sent when a new Reader matches.
*eProsima Fast DDS* offers three Durability options:

* VOLATILE (default): Messages are discarded as they are sent.
  If a new Reader matches after message *n*, it will start received from message *n+1*.
* TRANSIENT_LOCAL: The Writer saves a record of the last *k* messages it has sent.
  If a new reader matches after message *n*, it will start receiving from message *n-k*
* TRANSIENT: As TRANSIENT_LOCAL, but the record of messages will be saved to persistent storage, so it will be available
  if the writer is destroyed and recreated, or in case of an application crash.

To choose your preferred option:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_DURABILITY
    :end-before: //!--
    :dedent: 4

Because in the :ref:`rtps_layer` you have control over the History, in TRANSIENT_LOCAL and TRANSIENT modes the Writer
sends all changes you have not explicitly released from the History.

Configuring the History
-----------------------

The History has its own configuration structure, the |HistoryAttributes-api|.

Changing the maximum size of the payload
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can choose the maximum size of the Payload that can go into a :class:`CacheChange_t`.
Be sure to choose a size that allows it to hold the biggest possible piece of data:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_PAYLOADMAXSIZE
    :end-before: //!--
    :dedent: 4

Changing the size of the History
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can specify a maximum amount of changes for the History to hold and an initial amount of allocated changes:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_RESOURCES
    :end-before: //!--
    :dedent: 4

When the initial amount of reserved changes is lower than the maximum, the History will allocate more changes as they
are needed until it reaches the maximum size.

.. _rtps_layer_custom_payload_pool:

Using a custom Payload Pool
---------------------------

A *Payload* is defined as the data the user wants to transmit between a Writer and a Reader.
RTPS needs to add some metadata to this Payload in order to manage the communication between the endpoints.
Therefore, this Payload is encapsulated inside the :cpp:member:`SerializedPayload_t` field
of the :class:`CacheChange_t`,
while the rest of the fields of the :class:`CacheChange_t` provide the required metadata.

|WriterHistory-api| and |ReaderHistory-api| provide an interface for the user to interact with these changes:
Changes to be transmitted by the Writer are added to its WriterHistory,
and changes already processed on the Reader can be removed from the ReaderHistory.
In this sense, the History acts as a buffer for changes that are not fully processed yet.

During a normal execution, new changes are added to the History and old ones are removed from it.
In order to manage the lifecycle of the Payloads contained in these changes,
Readers and Writers use a pool object,
an implementation of the |IPayloadPool-api| interface.
Different pool implementations allow for different optimizations.
For example, Payloads of different size could be retrieved from different preallocated memory chunks.

Writers and Readers can automatically select a default Payload pool implementation that best suits
the configuration given in |HistoryAttributes-api|.
However, a custom Payload pool can be given to |RTPSDomain::createRTPSWriter-api| and
|RTPSDomain::createRTPSReader-api| functions.
Writers and Readers will use the provided pool when a new :class:`CacheChange_t` is requested
or released.


IPayloadPool interface
^^^^^^^^^^^^^^^^^^^^^^

* |IPayloadPool::get_payload-api| overload with size parameter:

  Ties an empty Payload of the requested size to a :class:`CacheChange_t` instance.
  The Payload can then be filled with the required data.

* |IPayloadPool::get_payload-api| overload with SerializadPayload parameter:

  Copies the given Payload data to a new Payload from the pool and ties it to the :class:`CacheChange_t` instance.
  This overload also takes a pointer to the pool that owns the original Payload.
  This allows certain optimizations, like sharing the Payload if the original one comes form this same pool,
  therefore avoiding the copy operation.

* |IPayloadPool::release_payload-api|:

  Returns the Payload tied to a :class:`CacheChange_t` to the pool, and breaks the tie.

.. important::
  When implementing a custom Payload pool, make sure that the allocated Payloads
  fulfill the requirements of standard RTPS serialization.
  Specifically, the Payloads must be large enough to accommodate the serialized user data plus the 4 octets
  of the `SerializedPayloadHeader` as specified in section 10.2 of the `RTPS standard`_.

  For example, if we know the upper bound of the serialized user data,
  we may consider implementing a pool that always allocates Payloads of a fixed size,
  large enough to hold any of this data.
  If the serialized user data has at most N octets,
  then the allocated Payloads must have at least N+4 octets.

  Note that the size requested to |IPayloadPool::get_payload-api| already considers this 4 octet header.

Default Payload pool implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If no custom Payload pool is provided to the Writer or Reader, *Fast DDS* will automatically use the
default implementation that best matches the |HistoryAttributes::memoryPolicy-api| configuration of the History.

**PREALLOCATED_MEMORY_MODE**

All payloads will have a data buffer of fixed size,
equal to the value of |HistoryAttributes::payloadMaxSize-api|,
regardless of the size requested to |IPayloadPool::get_payload-api|.
Released Payloads can be reused for another :class:`CacheChange_t`.
This reduces memory allocation operations at the cost of higher memory usage.

During the initialization of the History, |HistoryAttributes::initialReservedCaches-api|
Payloads are preallocated for the initially allocated :class:`CacheChange_t`.

**PREALLOCATED_WITH_REALLOC_MEMORY_MODE**

Payloads are guaranteed to have a data buffer at least as large as the
maximum between the requested size and |HistoryAttributes::payloadMaxSize-api|.
Released Payloads can be reused for another :class:`CacheChange_t`.
If there is at least one free Payload with a buffer size equal or larger to the requested one,
no memory allocation is done.

During the initialization of the History, |HistoryAttributes::initialReservedCaches-api|
Payloads are preallocated for the initially allocated :class:`CacheChange_t`.

**DYNAMIC_RESERVE_MEMORY_MODE**

Every time a Payload is requested, a new one is allocated in memory with the appropriate size.
|HistoryAttributes::payloadMaxSize-api| is ignored.
The memory of released Payloads is always deallocated, so there are never free Payloads in the pool.
This reduces memory usage at the cost of frequent memory allocations.

No preallocation of Payloads is done in the initialization of the History,

**DYNAMIC_REUSABLE_MEMORY_MODE**

Payloads are guaranteed to have a data buffer at least as large as the requested size.
|HistoryAttributes::payloadMaxSize-api| is ignored.

Released Payloads can be reused for another :class:`CacheChange_t`.
If there is at least one free Payload with a buffer size equal or larger to the requested one,
no memory allocation is done.

.. _rtps_layer_payload_pool_example:

Example using a custom Payload pool
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_ENTITY_CREATE_WITH_PAYLOAD_POOL
    :end-before: //!--
    :dedent: 4
