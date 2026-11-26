<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="rtps-layer"></a>

<a id="id1"></a>

# RTPS Layer

The lower level RTPS Layer of *eprosima Fast DDS* serves an implementation of the protocol defined in the
[RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2).
This layer provides more control over the internals of the communication protocol than the [DDS Layer](../dds_layer/dds_layer.md#dds-layer), so advanced
users have finer control over the library’s functionalities.

<a id="relation-to-the-dds-layer"></a>

## Relation to the DDS Layer

Elements of this layer map one-to-one with elements from the [DDS Layer](../dds_layer/dds_layer.md#dds-layer), with a few additions.
This correspondence is shown in the following table:

| [DDS Layer](../dds_layer/dds_layer.md#dds-layer)                                                            | [RTPS Layer](#rtps-layer)   |
|-------------------------------------------------------------------------------------------------------------|-----------------------------|
| [Domain](../dds_layer/domain/domain.md#dds-layer-domain)                                                    | RTPSDomain                  |
| [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) | RTPSParticipant             |
| [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)                | RTPSWriter                  |
| [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader)              | RTPSReader                  |

<a id="how-to-use-the-rtps-layer"></a>

## How to use the RTPS Layer

We will now go over the use of the RTPS Layer like we did with the [DDS Layer](../dds_layer/dds_layer.md#dds-layer) one,
explaining the new features it presents.

We recommend you to look at the example describing how to use the RTPS layer that come with the distribution
while reading this section.
It is located in
[examples/cpp/rtps](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/rtps).

<a id="managing-the-participant"></a>

### Managing the Participant

Creating a [`RTPSParticipant`](../api_reference/rtps/participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE) is done with [`RTPSDomain::createParticipant()`](../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain17createParticipantENSt8uint32_tERK25RTPSParticipantAttributesP23RTPSParticipantListener).
[`RTPSParticipantAttributes`](../api_reference/rtps/attributes/RTPSParticipantAttributes.md#_CPPv4N8eprosima7fastdds4rtps25RTPSParticipantAttributesE) structure is used to configure the `RTPSParticipant` upon creation.

```c++
RTPSParticipantAttributes participant_attr;
participant_attr.setName("participant");
RTPSParticipant* participant = RTPSDomain::createParticipant(0, participant_attr);
```

<a id="managing-the-writers-and-readers"></a>

### Managing the Writers and Readers

As the RTPS standard specifies, [`RTPSWriters`](../api_reference/rtps/writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) and [`RTPSReaders`](../api_reference/rtps/reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE) are always associated
with a [`History`](../api_reference/rtps/history/History_class.md#_CPPv4N8eprosima7fastdds4rtps7HistoryE) element.
In the [DDS Layer](../dds_layer/dds_layer.md#dds-layer), its creation and management is hidden,
but in the [RTPS Layer](#rtps-layer), you have full control over its creation and configuration.

Writers are created with [`RTPSDomain::createRTPSWriter()`](../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain16createRTPSWriterEP15RTPSParticipantR16WriterAttributesP13WriterHistoryP14WriterListener) and configured with a [`WriterAttributes`](../api_reference/rtps/attributes/WriterAttributes.md#_CPPv4N8eprosima7fastdds4rtps16WriterAttributesE) structure.
They also need a [`WriterHistory`](../api_reference/rtps/history/WriterHistory.md#_CPPv4N8eprosima7fastdds4rtps13WriterHistoryE) which is configured with a [`HistoryAttributes`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) structure.

```c++
HistoryAttributes history_attr;
WriterHistory* history = new WriterHistory(history_attr);
WriterAttributes writer_attr;
RTPSWriter* writer = RTPSDomain::createRTPSWriter(participant, writer_attr, history);
```

Similar to the creation of Writers, Readers are created with [`RTPSDomain::createRTPSReader()`](../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain16createRTPSReaderEP15RTPSParticipantR16ReaderAttributesP13ReaderHistoryP14ReaderListener)
and configured with a [`ReaderAttributes`](../api_reference/rtps/attributes/ReaderAttributes.md#_CPPv4N8eprosima7fastdds4rtps16ReaderAttributesE) structure.
A [`HistoryAttributes`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE) structure is used to configure the required [`ReaderHistory`](../api_reference/rtps/history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE).
Note that in this case, you can provide a specialization of [`ReaderListener`](../api_reference/rtps/reader/ReaderListener.md#_CPPv4N8eprosima7fastdds4rtps14ReaderListenerE) class that implements your
callbacks:

```c++
class MyReaderListener : public ReaderListener
{
    // Callbacks override
};
MyReaderListener listener;
HistoryAttributes history_attr;
ReaderHistory* history = new ReaderHistory(history_attr);
ReaderAttributes reader_attr;
RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, history, &listener);
```

<a id="using-the-history-to-send-and-receive-data"></a>

### Using the History to Send and Receive Data

In the RTPS Protocol, Readers and Writers save the data about a topic in their associated Histories.
Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as [`CacheChange_t`](../api_reference/rtps/common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE).
Changes are always managed by the History.

You can add a new `CacheChange_t` to the History of the Writer to send data.
The procedure is as follows:

1. Request a `CacheChange_t` from the Writer History with `WriterHistory::create_change()`.
   In order to allocate enough memory,
   you need to provide as argument the maximum number of bytes in the payload.
2. Fill the `CacheChange_t` with the data.
3. Add it to the History with [`WriterHistory::add_change()`](../api_reference/rtps/history/WriterHistory.md#_CPPv4N8eprosima7fastdds4rtps13WriterHistory10add_changeEP13CacheChange_t).

The Writer will take care of everything to communicate the data to the Readers.

```c++
// Request a change from the history
CacheChange_t* change = history->create_change(255, ALIVE);
// Write serialized data into the change
change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2) + 1;
// Insert change into the history. The Writer takes care of the rest.
history->add_change(change);
```

If your topic data type has several fields, you will have to provide functions to serialize and deserialize
your data in and out of the `CacheChange_t`.
*Fast DDS-Gen* does this for you.

You can receive data from within the `ReaderListener::onNewCacheChangeAdded` callback,
as we did in the [DDS Layer](../dds_layer/dds_layer.md#dds-layer):

1. The callback receives a `CacheChange_t` parameter containing the received data.
2. Process the data within the received `CacheChange_t`.
3. Inform the Reader’s History that the change is not needed anymore.

```c++
class MyReaderListener : public ReaderListener
{
public:

    MyReaderListener()
    {
    }

    ~MyReaderListener()
    {
    }

    void onNewCacheChangeAdded(
            RTPSReader* reader,
            const CacheChange_t* const change)
    {
        // The incoming message is enclosed within the `change` in the function parameters
        printf("%s\n", change->serializedPayload.data);
        // Once done, remove the change
        reader->get_history()->remove_change((CacheChange_t*)change);
    }

};
```

<a id="rtps-layer-builtin-transports"></a>

<a id="managing-the-builtin-transports"></a>

### Managing the Builtin Transports

DDS uses the [Transport Layer](../transport/transport.md#comm-transports-configuration) to allow communication between DDS entities. *eProsima Fast DDS* comes
with five transports already implemented. However, these transports are not always exclusive between them
and in some cases they can be used simultaneously.

You can choose what transports you want to use by disabling the use of builtin transports and manually
adding them (see [TransportConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#transportconfigqos)) or using the default builtin transports behavior and selecting
one of the configuration options listed below. Each option modifies the kind of transports that will be
instantiated.

| Builtin Transports Options   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `NONE`                       | No transport will be instantiated. Hence, the user must manually add<br/>the desired  transports. Otherwise, the participant creation will fail.                                                                                                                                                                                                                                                                                                                                                  |
| `DEFAULT`                    | UDPv4 and SHM transports will be instantiated. SHM transport has priority<br/>over the UDPv4  transport. Meaning that SHM will always be used<br/>when possible.                                                                                                                                                                                                                                                                                                                                  |
| `DEFAULTv6`                  | UDPv6 and SHM transports will be instantiated. SHM transport has priority<br/>over the UDPv6 transport. Meaning that SHM will always be used<br/>when possible.                                                                                                                                                                                                                                                                                                                                   |
| `SHM`                        | Only a SHM transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `UDPv4`                      | Only a UDPv4 transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `UDPv6`                      | Only a UDPv6 transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `LARGE_DATA`                 | UDPv4, TCPv4, and SHM transports will be instantiated. However, UDP will<br/>only be used  for multicast announcements during the participant<br/>discovery phase (see [Discovery phases](../discovery/discovery.md#disc-phases))  while the participant<br/>liveliness and the application data delivery occurs over TCP or SHM.<br/>This configuration is useful when working with large data.(See<br/>[Large Data mode and Fast DDS over TCP](../use_cases/tcp/tcp_use_case.md#use-case-tcp)). |
```c++
RTPSParticipantAttributes participant_attr;
participant_attr.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);
RTPSParticipant* participant = RTPSDomain::createParticipant(0, participant_attr);
```

The same result can also be obtained using the [`setup_transports()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos16setup_transportsEN4rtps17BuiltinTransportsERKN4rtps24BuiltinTransportsOptionsE) wrapper
function of the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos), XML profiles (see [RTPS element type](../xml_configuration/domainparticipant.md#rtps)) or the
`FASTDDS_BUILTIN_TRANSPORTS` environment variable (see [FASTDDS_BUILTIN_TRANSPORTS](../env_vars/env_vars.md#env-vars-builtin-transports)).

#### NOTE
TCPv4 transport is initialized with the following configuration:

* [`calculate_crc`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13calculate_crcE), [`check_crc`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9check_crcE) and
  [`apply_security`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor14apply_securityE) are set to false.
* [`enable_tcp_nodelay`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor18enable_tcp_nodelayE) is set to true.
* [`keep_alive_thread`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17keep_alive_threadE) and
  [`accept_thread`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13accept_threadE) use the default configuration.

#### WARNING
To obtain a better performance when working with large data messages it is extremely recommended to use the
builtin transports configuration options to adjust the transport to the specific needs of the application.
Please refer to [Large Data with configuration options](../use_cases/tcp/tcp_large_data_with_options.md#use-case-large-data-options) for more information about how to configure it.

<a id="configuring-readers-and-writers"></a>

## Configuring Readers and Writers

One of the benefits of using the [RTPS Layer](#rtps-layer) is that it provides new configuration possibilities while
maintaining the options from the DDS layer.
For example, you can set a Writer or a Reader as a Reliable or Best-Effort endpoint as previously:

```c++
writer_attr.endpoint.reliabilityKind = BEST_EFFORT;
```

<a id="settingdatadurability"></a>

<a id="setting-the-data-durability-kind"></a>

### Setting the data durability kind

The Durability parameter defines the behavior of the Writer regarding samples already sent when a new Reader matches.
*eProsima Fast DDS* offers three Durability options:

* VOLATILE (default): Messages are discarded as they are sent.
  If a new Reader matches after message *n*, it will start received from message *n+1*.
* TRANSIENT_LOCAL: The Writer saves a record of the last *k* messages it has sent.
  If a new reader matches after message *n*, it will start receiving from message *n-k*
* TRANSIENT: As TRANSIENT_LOCAL, but the record of messages will be saved to persistent storage, so it will be available
  if the writer is destroyed and recreated, or in case of an application crash.

To choose your preferred option:

```c++
writer_attr.endpoint.durabilityKind = TRANSIENT_LOCAL;
```

Because in the [RTPS Layer](#rtps-layer) you have control over the History, in TRANSIENT_LOCAL and TRANSIENT modes the Writer
sends all changes you have not explicitly released from the History.

<a id="configuring-the-history"></a>

## Configuring the History

The History has its own configuration structure, the [`HistoryAttributes`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE).

<a id="changing-the-maximum-size-of-the-payload"></a>

### Changing the maximum size of the payload

You can choose the maximum size of the Payload that can go into a `CacheChange_t`.
Be sure to choose a size that allows it to hold the biggest possible piece of data:

```c++
history_attr.payloadMaxSize  = 250; // Defaults to 500 bytes
```

<a id="changing-the-size-of-the-history"></a>

### Changing the size of the History

You can specify a maximum amount of changes for the History to hold and an initial amount of allocated changes:

```c++
history_attr.initialReservedCaches = 250; // Defaults to 500
history_attr.maximumReservedCaches = 500; // Defaults to 0 = Unlimited Changes
```

When the initial amount of reserved changes is lower than the maximum, the History will allocate more changes as they
are needed until it reaches the maximum size.

<a id="rtps-layer-custom-payload-pool"></a>

<a id="using-a-custom-payload-pool"></a>

## Using a custom Payload Pool

A *Payload* is defined as the data the user wants to transmit between a Writer and a Reader.
RTPS needs to add some metadata to this Payload in order to manage the communication between the endpoints.
Therefore, this Payload is encapsulated inside the `SerializedPayload_t` field
of the `CacheChange_t`,
while the rest of the fields of the `CacheChange_t` provide the required metadata.

[`WriterHistory`](../api_reference/rtps/history/WriterHistory.md#_CPPv4N8eprosima7fastdds4rtps13WriterHistoryE) and [`ReaderHistory`](../api_reference/rtps/history/ReaderHistory.md#_CPPv4N8eprosima7fastdds4rtps13ReaderHistoryE) provide an interface for the user to interact with these changes:
Changes to be transmitted by the Writer are added to its WriterHistory,
and changes already processed on the Reader can be removed from the ReaderHistory.
In this sense, the History acts as a buffer for changes that are not fully processed yet.

During a normal execution, new changes are added to the History and old ones are removed from it.
In order to manage the lifecycle of the Payloads contained in these changes,
Readers and Writers use a pool object,
an implementation of the [`IPayloadPool`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPoolE) interface.
Different pool implementations allow for different optimizations.
For example, Payloads of different size could be retrieved from different preallocated memory chunks.

Writers and Readers can automatically select a default Payload pool implementation that best suits
the configuration given in [`HistoryAttributes`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributesE).
However, a custom Payload pool can be given to [`RTPSDomain::createRTPSWriter()`](../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain16createRTPSWriterEP15RTPSParticipantR16WriterAttributesP13WriterHistoryP14WriterListener) and
[`RTPSDomain::createRTPSReader()`](../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain16createRTPSReaderEP15RTPSParticipantR16ReaderAttributesP13ReaderHistoryP14ReaderListener) functions.
Writers and Readers will use the provided pool when a new `CacheChange_t` is requested
or released.

<a id="rtps-layer-ipayload-pool-interface"></a>

<a id="ipayloadpool-interface"></a>

### IPayloadPool interface

* [`IPayloadPool::get_payload`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPool11get_payloadENSt8uint32_tER19SerializedPayload_t) overload with size parameter:

  Ties an empty Payload of the requested size to a `CacheChange_t` instance.
  The Payload can then be filled with the required data.
* [`IPayloadPool::get_payload`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPool11get_payloadENSt8uint32_tER19SerializedPayload_t) overload with SerializadPayload parameter:

  Copies the given Payload data to a new Payload from the pool.
  Each `SerializedPayload_t` contains a pointer to the pool that allocated its data.
  This allows certain certain optimizations, like sharing the Payload if the original one comes from this same pool,
  therefore avoiding the copy operation.
* [`IPayloadPool::release_payload`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPool15release_payloadER19SerializedPayload_t):

  Returns the Payload tied to a `CacheChange_t` to the pool, and breaks the tie.

#### IMPORTANT
When implementing a custom Payload pool, make sure that the allocated Payloads
fulfill the requirements of standard RTPS serialization.
Specifically, the Payloads must be large enough to accommodate the serialized user data plus the 4 octets
of the SerializedPayloadHeader as specified in section 10.2 of the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2).

For example, if we know the upper bound of the serialized user data,
we may consider implementing a pool that always allocates Payloads of a fixed size,
large enough to hold any of this data.
If the serialized user data has at most N octets,
then the allocated Payloads must have at least N+4 octets.

Note that the size requested to [`IPayloadPool::get_payload`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPool11get_payloadENSt8uint32_tER19SerializedPayload_t) already considers this 4 octet header.

<a id="default-payload-pool-implementation"></a>

### Default Payload pool implementation

If no custom Payload pool is provided to the Writer or Reader, *Fast DDS* will automatically use the
default implementation that best matches the [`memoryPolicy`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes12memoryPolicyE) configuration of the History.

**PREALLOCATED_MEMORY_MODE**

All payloads will have a data buffer of fixed size,
equal to the value of [`payloadMaxSize`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes14payloadMaxSizeE),
regardless of the size requested to [`IPayloadPool::get_payload`](../api_reference/rtps/history/IPayloadPool.md#_CPPv4N8eprosima7fastdds4rtps12IPayloadPool11get_payloadENSt8uint32_tER19SerializedPayload_t).
Released Payloads can be reused for another `CacheChange_t`.
This reduces memory allocation operations at the cost of higher memory usage.

During the initialization of the History, [`initialReservedCaches`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes21initialReservedCachesE)
Payloads are preallocated for the initially allocated `CacheChange_t`.

**PREALLOCATED_WITH_REALLOC_MEMORY_MODE**

Payloads are guaranteed to have a data buffer at least as large as the
maximum between the requested size and [`payloadMaxSize`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes14payloadMaxSizeE).
Released Payloads can be reused for another `CacheChange_t`.
If there is at least one free Payload with a buffer size equal or larger to the requested one,
no memory allocation is done.

During the initialization of the History, [`initialReservedCaches`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes21initialReservedCachesE)
Payloads are preallocated for the initially allocated `CacheChange_t`.

**DYNAMIC_RESERVE_MEMORY_MODE**

Every time a Payload is requested, a new one is allocated in memory with the appropriate size.
[`payloadMaxSize`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes14payloadMaxSizeE) is ignored.
The memory of released Payloads is always deallocated, so there are never free Payloads in the pool.
This reduces memory usage at the cost of frequent memory allocations.

No preallocation of Payloads is done in the initialization of the History,

**DYNAMIC_REUSABLE_MEMORY_MODE**

Payloads are guaranteed to have a data buffer at least as large as the requested size.
[`payloadMaxSize`](../api_reference/rtps/attributes/HistoryAttributes.md#_CPPv4N8eprosima7fastdds4rtps17HistoryAttributes14payloadMaxSizeE) is ignored.

Released Payloads can be reused for another `CacheChange_t`.
If there is at least one free Payload with a buffer size equal or larger to the requested one,
no memory allocation is done.

<a id="rtps-layer-payload-pool-example"></a>

<a id="example-using-a-custom-payload-pool"></a>

### Example using a custom Payload pool

```c++
// A simple payload pool that reserves and frees memory each time
class CustomPayloadPool : public IPayloadPool
{
    bool get_payload(
            uint32_t size,
            SerializedPayload_t& payload) override
    {
        // Reserve new memory for the payload buffer
        octet* payload_buff = new octet[size];

        // Assign the payload buffer to the CacheChange and update sizes
        payload.data = payload_buff;
        payload.length = size;
        payload.max_size = size;

        // Tell the CacheChange who needs to release its payload
        payload.payload_owner = this;

        return true;
    }

    bool get_payload(
            const SerializedPayload_t& data,
            SerializedPayload_t& payload)
    {
        // Reserve new memory for the payload buffer
        octet* payload_buff = new octet[data.length];

        // Copy the data
        memcpy(payload_buff, data.data, data.length);

        // Tell the CacheChange who needs to release its payload
        payload.payload_owner = this;

        // Assign the payload buffer to the CacheChange and update sizes
        payload.data = payload_buff;
        payload.length = data.length;
        payload.max_size = data.length;

        return true;
    }

    bool release_payload(
            SerializedPayload_t& payload) override
    {
        // Ensure precondition
        if (this != payload.payload_owner)
        {
            std::cerr << "Trying to release a payload buffer allocated by a different PayloadPool." << std::endl;
            return false;
        }

        // Dealloc the buffer of the payload
        delete[] payload.data;

        // Reset sizes and pointers
        payload.data = nullptr;
        payload.length = 0;
        payload.max_size = 0;

        // Reset the owner of the payload
        payload.payload_owner = nullptr;

        return true;
    }

};

std::shared_ptr<CustomPayloadPool> payload_pool = std::make_shared<CustomPayloadPool>();

// A writer using the custom payload pool
HistoryAttributes writer_history_attr;
WriterHistory* writer_history = new WriterHistory(writer_history_attr, payload_pool);
WriterAttributes writer_attr;
RTPSWriter* writer = RTPSDomain::createRTPSWriter(participant, writer_attr, writer_history);

// A reader using the same instance of the custom payload pool
HistoryAttributes reader_history_attr;
ReaderHistory* reader_history = new ReaderHistory(reader_history_attr);
ReaderAttributes reader_attr;
RTPSReader* reader = RTPSDomain::createRTPSReader(participant, reader_attr, payload_pool, reader_history);

// Write and Read operations work as usual, but take the Payloads from the pool.
// Requesting a change to the Writer will provide one with an empty Payload taken from the pool
CacheChange_t* change = writer_history->create_change(255, ALIVE);

// Write serialized data into the change and add it to the history
change->serializedPayload.length = sprintf((char*) change->serializedPayload.data, "My example string %d", 2) + 1;
writer_history->add_change(change);
```
