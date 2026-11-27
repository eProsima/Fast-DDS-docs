<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-custom-buffers"></a>

<a id="custom-buffers-pro"></a>

# Custom buffers **[Pro]**

*Fast DDS Pro* maps the unbounded sequences of octets of a data type to [`Buffer`](../../api_reference/utils/buffer/buffer_class.md#_CPPv4I00EN8eprosima7fastdds6BufferE), a replacement for
`std::vector<uint8_t>` whose data are not necessarily in CPU memory.
By default, the contents of a buffer are serialized and sent as part of the sample.
This section explains how to change that for buffers that keep their data somewhere else (for example,
in the memory of a GPU, or in a file), so that what travels through the network is not the data, but
what the receiver needs to get access to them.

<a id="overview"></a>

## Overview

The support for custom buffers is made of the following pieces, all of them in the
`fastdds/utils/buffer` directory of the *Fast DDS* include directory:

* [`Buffer`](../../api_reference/utils/buffer/buffer_class.md#_CPPv4I00EN8eprosima7fastdds6BufferE) (`buffer.hpp`): the container used in the data types.
  It behaves as a `std::vector<uint8_t>` as long as its data are in CPU memory, and delegates everything else to a
  *buffer implementation*.
* [`BufferImplBase`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE) (`buffer_impl_base.hpp`): the interface to be implemented by a *buffer implementation*,
  that is, by each kind of buffer.
  The implementation of the buffers that keep their data in CPU memory, [`CpuBufferImpl`](../../api_reference/utils/buffer/cpubufferimpl.md#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)
  (`cpu_buffer_impl.hpp`), is provided by *Fast DDS*, and is the one used by default.
* [`BufferBackend`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4N8eprosima7fastdds13BufferBackendE) (`BufferBackend.hpp`): the interface to be implemented to serialize and deserialize the
  buffers of one kind.
  It is the *backend* of that kind of buffer.
* [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE) (`BufferAwareContext.hpp`): a
  [type support context](../../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-topic-type-support-context) that keeps the backends registered by the
  application, and uses them while serializing and deserializing the samples of a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or a [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).

Every kind of buffer is identified by a string, its *backend type*.
The CPU buffers have `"cpu"` as backend type, and the backend type of the custom buffers is chosen by their
implementation.

#### NOTE
Only `Buffer<uint8_t>` takes part in the custom serialization.
`Buffer` can be instantiated with other types, but the serialization of those instances cannot be customized.

<a id="buffers-in-the-generated-types"></a>

## Buffers in the generated types

[Fast DDS-Gen](../../../fastddsgen/introduction/introduction.md#fastddsgen-intro) uses `eprosima::fastdds::Buffer<uint8_t>` instead of `std::vector<uint8_t>` for the members of the
types that are unbounded sequences of `octet` or `uint8` (also when they are defined through a `typedef`).
For instance, the following IDL generates a class whose `data()` member functions work with `Buffer<uint8_t>`,
and the generated code includes `<fastdds/utils/buffer/buffer.hpp>`.

```omg-idl
@extensibility(APPENDABLE)
struct FileBuffer
{
    unsigned long index;
    sequence<uint8> data;
};
```

Bounded sequences are always mapped to `std::vector`.
The `-disable-buffers` option of [Fast DDS-Gen](../../../fastddsgen/introduction/introduction.md#fastddsgen-intro) (see [Usage](../../../fastddsgen/usage/usage.md#fastddsgen-usage)) avoids this mapping, and generates
`std::vector<T>` for every sequence.

Because a `Buffer` with CPU data converts implicitly to `std::vector<uint8_t>&`, and can be constructed from a
`std::vector<uint8_t>`, the applications written for a previous version of the generated code keep working
without changes.

#### NOTE
Code that serializes a `Buffer<uint8_t>` by hand using *Fast CDR* (for example, if the type is not generated with
[Fast DDS-Gen](../../../fastddsgen/introduction/introduction.md#fastddsgen-intro)) has to include `<fastdds/utils/buffer/buffer_serialization.hpp>`, which provides the
specializations of `calculate_serialized_size`, `serialize`, and `deserialize` for that type.

<a id="using-buffer"></a>

## Using Buffer

<a id="cpu-buffers"></a>

### CPU buffers

A default constructed `Buffer`, and a `Buffer` constructed from a size, a `std::vector`, or a
`std::initializer_list`, keep their data in CPU memory.
They offer the interface of `std::vector` (element access, iterators, `resize()`, `push_back()`, …),
and convert implicitly to `std::vector<uint8_t>&`.

```c++
// A default Buffer keeps its data in CPU memory, and can be used as a std::vector<uint8_t>
Buffer<uint8_t> buffer(4, 0);
buffer.push_back(1);
buffer[0] = 2;
std::vector<uint8_t>& vector_ref = buffer;
vector_ref.push_back(3);
std::vector<uint8_t> copy = buffer.to_vector();
```

<a id="buffers-with-other-backends"></a>

### Buffers with other backends

A `Buffer` with a custom implementation is created from a `std::unique_ptr<BufferImplBase<uint8_t>>`.
The implementation is set at construction, and there is no function to change it afterwards.
The only way to replace it is to assign another `Buffer` (by copy or by move), which replaces the implementation with
the one of the source.
As with any assignment, it must not happen while other threads are accessing the buffer.

The data of these buffers cannot be accessed through the `std::vector` interface.
Only [`Buffer::get_backend_type()`](../../api_reference/utils/buffer/buffer_class.md#_CPPv4NK8eprosima7fastdds6Buffer16get_backend_typeEv), `size()`, `empty()`, and the comparison operators work with any backend.
The rest of the `std::vector` interface, and the implicit conversion to `std::vector<uint8_t>&`, throw
`std::runtime_error` unless the buffer is a CPU buffer.
To get the data of a buffer with any backend, use [`Buffer::to_vector()`](../../api_reference/utils/buffer/buffer_class.md#_CPPv4NK8eprosima7fastdds6Buffer9to_vectorEv), which returns a copy of them in CPU memory.

```c++
// A Buffer with a custom implementation is created from it. There is no setter to change it afterwards,
// only assigning another Buffer replaces it.
Buffer<uint8_t> buffer(std::unique_ptr<BufferImplBase<uint8_t>>(new DemoBufferImpl(1, 1024)));

// Works for any backend
size_t size = buffer.size();
std::string backend_type = buffer.get_backend_type();    // "demo"

// Throws std::runtime_error when the backend is not the CPU one
try
{
    buffer.push_back(1);
}
catch (const std::runtime_error&)
{
    // Operation requires CPU backend. Current backend: demo. ...
}

// Explicit copy in CPU memory
std::vector<uint8_t> in_cpu = buffer.to_vector();
```

Copying a `Buffer` makes a deep copy of its implementation (see [`clone()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase5cloneEv)).
A `Buffer` that has been moved from is left empty, as a CPU buffer.

<a id="serialization-of-buffers"></a>

## Serialization of buffers

What is serialized for a `Buffer<uint8_t>` depends on the kind of buffer, and on the type support context of the
[DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and the [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) (see [Type support context](../../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-topic-type-support-context)).

| Buffer                                                                                    | Serialization                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|-------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CPU buffer                                                                                | Always as a standard CDR sequence of octets, with or without [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE).<br/>The receiver does not need to know anything about custom buffers to read it.                                                                                                                                                                                                                                                   |
| Custom buffer, and the context has a backend for its backend type                         | A marker, the backend type, and what the backend serializes.<br/>The receiver must have a [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE) with a backend registered for that backend type.                                                                                                                                                                                                                                                       |
| Custom buffer, and there is no context or the context has no backend for its backend type | The data are copied to CPU memory (see [`to_cpu()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase6to_cpuEv)), and serialized as a standard CDR sequence<br/>of octets, as a CPU buffer.<br/>When the CDR version is [`XCDR_DATA_REPRESENTATION`](../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE), the size of the serialized buffer is calculated without<br/>making that copy. |

On the reception side, a data reader with a [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE) detects the marker, and uses the backend
registered for the backend type that follows to create the buffer.
If the sample carries the standard sequence of octets instead, the result is a CPU buffer.
If no backend is registered for the received backend type, the deserialization fails with a
`eprosima::fastcdr::exception::BadParamException`.
A data reader without [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE) always expects a standard sequence of octets, so it cannot deserialize
the samples serialized by a backend.

#### WARNING
The marker and the backend data are only understood by *Fast DDS* applications that have the backend registered.
Use custom backends only between applications that are developed to work together, and make sure that every
data reader that may match with a data writer sending custom buffers has the same backends registered.
The [example](#use-case-custom-buffers-example) shows how to make sure of that by overriding the matching.

<a id="implementing-a-custom-buffer"></a>

## Implementing a custom buffer

Creating a new kind of buffer requires three steps.

<a id="implement-the-buffer"></a>

### 1. Implement the buffer

Derive from [`BufferImplBase`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE), and implement:

* [`get_backend_type()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase16get_backend_typeEv): the backend type of the buffer.
  Each implementation returns a fixed string, which is also the one used to register its backend.
* [`size()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase4sizeEv): the number of elements of the buffer.
  It is required by all backends.
* [`to_cpu()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase6to_cpuEv): a copy of the data in a [`CpuBufferImpl`](../../api_reference/utils/buffer/cpubufferimpl.md#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE).
  It is what [`Buffer::to_vector()`](../../api_reference/utils/buffer/buffer_class.md#_CPPv4NK8eprosima7fastdds6Buffer9to_vectorEv) uses, and what the serialization uses when there is no backend for the buffer.
* [`clone()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase5cloneEv): a deep copy of the buffer.
  It is what the copy of a `Buffer` uses.

Anything else (how the data are written or read, how the buffer is created from a resource, …) is up to the
implementation, since the data of a custom buffer cannot be accessed through the interface of `std::vector`.

```c++
// A buffer whose data are not in CPU memory, but in a resource identified by a handle (e.g. a block of device memory)
class DemoBufferImpl : public BufferImplBase<uint8_t>
{
public:

    DemoBufferImpl(
            uint64_t handle,
            uint64_t size)
        : handle_(handle)
        , size_(size)
    {
    }

    // Identifies this kind of buffer. The backend that serializes it is registered using this same string.
    std::string get_backend_type() const override
    {
        return "demo";
    }

    // Number of elements. Used by Buffer::size() and by the serialization.
    size_t size() const override
    {
        return size_;
    }

    // Copy of the data in CPU memory. Used by Buffer::to_vector() and when there is no backend for this buffer.
    std::unique_ptr<CpuBufferImpl<uint8_t>> to_cpu() const override
    {
        std::unique_ptr<CpuBufferImpl<uint8_t>> cpu(new CpuBufferImpl<uint8_t>());
        cpu->get_storage().resize(size_);
        // Copy here the data of the resource into cpu->get_storage()
        return cpu;
    }

    // Deep copy. Used by the copy constructor and the copy assignment of Buffer.
    std::unique_ptr<BufferImplBase<uint8_t>> clone() const override
    {
        // Duplicate here the resource, and use the handle of the new one
        return std::unique_ptr<BufferImplBase<uint8_t>>(new DemoBufferImpl(handle_, size_));
    }

    // Anything else is up to the implementation
    uint64_t handle() const
    {
        return handle_;
    }

private:

    uint64_t handle_;
    uint64_t size_;
};
```

<a id="implement-the-backend"></a>

### 2. Implement the backend

Derive from [`BufferBackend`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4N8eprosima7fastdds13BufferBackendE), and implement the part of the serialization that is specific to the buffer.
These functions are called by the [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE) only with the buffers whose backend type matches the one
the backend was registered for, so they can safely cast the [`BufferImplBase`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE) they receive to the type of the
implementation.

* [`calculate_serialized_size()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend25calculate_serialized_sizeERN8eprosima7fastcdr17CdrSizeCalculatorERK14BufferImplBaseINSt7uint8_tEERNSt6size_tE): the number of bytes that [`serialize()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend9serializeERN8eprosima7fastcdr3CdrERK14BufferImplBaseINSt7uint8_tEE) writes.
  The calculator advances `current_alignment` as a side effect, and the context ignores the returned value.
* [`serialize()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend9serializeERN8eprosima7fastcdr3CdrERK14BufferImplBaseINSt7uint8_tEE): writes the buffer to the CDR stream.
* [`deserialize()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend11deserializeERN8eprosima7fastcdr3CdrE): reads the CDR stream and returns a new buffer implementation.

The context writes the marker and the backend type before calling the backend, and reads them before calling
[`deserialize()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend11deserializeERN8eprosima7fastcdr3CdrE), so the backend does not have to take care of them.

```c++
// Sends the handle and the size of a DemoBufferImpl instead of its data
class DemoBufferBackend : public BufferBackend
{
public:

    size_t calculate_serialized_size(
            eprosima::fastcdr::CdrSizeCalculator& calculator,
            const BufferImplBase<uint8_t>& data,
            size_t& current_alignment) const override
    {
        // The calculator advances current_alignment as a side effect. The context ignores the returned value.
        const size_t initial_alignment = current_alignment;
        // Safe cast: the backend is registered under the backend type returned by DemoBufferImpl, "demo", and the
        // context only calls it with buffers that return that same backend type.
        const DemoBufferImpl& impl = static_cast<const DemoBufferImpl&>(data);
        calculator.calculate_serialized_size(impl.handle(), current_alignment);
        calculator.calculate_serialized_size(static_cast<uint64_t>(impl.size()), current_alignment);
        return current_alignment - initial_alignment;
    }

    void serialize(
            eprosima::fastcdr::Cdr& cdr,
            const BufferImplBase<uint8_t>& data) const override
    {
        // Safe cast: the backend is registered under the backend type returned by DemoBufferImpl, "demo", and the
        // context only calls it with buffers that return that same backend type.
        const DemoBufferImpl& impl = static_cast<const DemoBufferImpl&>(data);
        cdr << impl.handle();
        cdr << static_cast<uint64_t>(impl.size());
    }

    std::unique_ptr<BufferImplBase<uint8_t>> deserialize(
            eprosima::fastcdr::Cdr& cdr) const override
    {
        // What is read comes from a remote entity, so it must be validated before being used.
        // Errors must be reported with a Fast CDR exception.
        uint64_t handle = 0;
        uint64_t size = 0;
        cdr >> handle;
        cdr >> size;
        return std::unique_ptr<BufferImplBase<uint8_t>>(new DemoBufferImpl(handle, size));
    }

};
```

#### WARNING
What [`deserialize()`](../../api_reference/utils/buffer/bufferbackend.md#_CPPv4NK8eprosima7fastdds13BufferBackend11deserializeERN8eprosima7fastcdr3CdrE) reads comes from a remote entity, so it must be validated before using it.
For example, a backend that receives a file name must not accept it if it refers to anything outside the
directory it is supposed to use.
Errors must be reported throwing a *Fast CDR* exception (for example,
`eprosima::fastcdr::exception::BadParamException`), as it is the only kind handled by the generated code.

<a id="register-the-backend"></a>

### 3. Register the backend

Create a [`BufferAwareContext`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE), register in it the backend of each kind of custom buffer with
[`register_backend()`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContext16register_backendERKNSt6stringERKNSt10shared_ptrI13BufferBackendEE) (using the same string returned by [`get_backend_type()`](../../api_reference/utils/buffer/bufferimplbase.md#_CPPv4NK8eprosima7fastdds14BufferImplBase16get_backend_typeEv)),
and set the context in the data writers and data readers that use these buffers with
[`DataWriter::set_type_support_context()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE) and [`DataReader::set_type_support_context()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE).
As explained in [Type support context](../../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-topic-type-support-context), the context must be set before the entity is enabled.
To achieve this, create the publisher and the subscriber with
`entity_factory().autoenable_created_entities` set to `false`, and enable the data writer or data reader
once the context is set.

<a id="datawriter"></a>

#### DataWriter

```c++
// Create a Publisher that does not enable the DataWriters it creates
PublisherQos publisher_qos = PUBLISHER_QOS_DEFAULT;
publisher_qos.entity_factory().autoenable_created_entities = false;
Publisher* publisher = participant->create_publisher(publisher_qos);

// Create the context, and register in it the backend of each kind of custom buffer
auto context = std::make_shared<BufferAwareContext>();
context->register_backend("demo", std::make_shared<DemoBufferBackend>());

// Create the DataWriter, set the context, and enable it
DataWriter* writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (RETCODE_OK != writer->set_type_support_context(context) ||
        RETCODE_OK != writer->enable())
{
    // Error
    return;
}
```

<a id="datareader"></a>

#### DataReader

```c++
// Create a Subscriber that does not enable the DataReaders it creates
SubscriberQos subscriber_qos = SUBSCRIBER_QOS_DEFAULT;
subscriber_qos.entity_factory().autoenable_created_entities = false;
Subscriber* subscriber = participant->create_subscriber(subscriber_qos);

// Create the context, and register in it the backend of each kind of custom buffer
auto context = std::make_shared<BufferAwareContext>();
context->register_backend("demo", std::make_shared<DemoBufferBackend>());

// Create the DataReader, set the context, and enable it
DataReader* reader = subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (RETCODE_OK != reader->set_type_support_context(context) ||
        RETCODE_OK != reader->enable())
{
    // Error
    return;
}
```

Both sides need the backend: a data reader that does not have it cannot make sense of the samples.
The backend registered for a backend type can be retrieved with [`get_backend()`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4NK8eprosima7fastdds18BufferAwareContext11get_backendERKNSt6stringE), and
[`for_each_backend()`](../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4I0ENK8eprosima7fastdds18BufferAwareContext16for_each_backendEvRR5_Func) calls a function with each of the registered backends.
The latter holds the internal lock of the context while iterating, so the function must not call any method of the
context.

```c++
BufferAwareContext context;
context.for_each_backend(
    [](const std::string& backend_type, const std::shared_ptr<BufferBackend>& backend)
    {
        // Do not call any method of the context from here
        static_cast<void>(backend_type);
        static_cast<void>(backend);
    });
```

<a id="use-case-custom-buffers-example"></a>

<a id="example"></a>

## Example

The `file_buffer` example, which is included as part of the *Fast DDS Pro* installation, implements a buffer whose
data are kept in a file, and a backend that sends only the name of the file.
The subscriber receives the name, looks for the file in its own directory, and gets a buffer that reads its data
from it, so the data never travel through *Fast DDS*, no matter how big they are.

Since a file name is meaningful only for the applications that can access the file, the example also overrides
[`should_endpoints_match()`](../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener22should_endpoints_matchEPK17DomainParticipantRK28SubscriptionBuiltinTopicDataRK27PublicationBuiltinTopicData) in a participant listener so that a data writer and a data
reader only match when both can access the same directory.
Please refer to the `README.md` file of the example for the details and for the instructions to run it.
