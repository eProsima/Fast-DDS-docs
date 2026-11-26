<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-request-reply"></a>

<a id="request-reply-communication"></a>

# Request-Reply communication

This section explains how to configure a Request-Reply communication in *Fast DDS* between two applications.
A *Client application* will send a *Request* to the *Server application* and this one, after processing the request,
will send a *Reply* to the *Client application*.

```plantuml
 node "Client application" as client {
 }

 node "Server application" as server {
 }

 client --> server : Request
 server --> client : Reply
```

<a id="overview"></a>

## Overview

This kind of communication involves in the DDS paradigm the usage of two *Topics*: one for sending requests
(Request Topic) and the other one for sending replies (Reply Topic).
For managing these *Topics* four DDS entities are involved: a DataReader and a DataWriter for each *Topic*.
The DDS communication schema will be:

```plantuml
 node "Server application" {
     cloud "Processing" as Processing
     [Request DataReader] -right-> Processing
     Processing -right-> [Reply DataWriter]
 }

 node "Client application" {
     [Reply DataReader]
     [Request DataWriter]
 }

 [Request DataWriter] -down-> [Request DataReader] : Request Topic
 [Reply DataWriter] --> [Reply DataReader] : Reply Topic
```

The key for making *Request-Reply* work is relating the Request with the Reply in the client’s side.
*Fast DDS* API provides [`SampleIdentity`](../../api_reference/rtps/common/SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) to achieve this.

A full example can be found in the [Fast DDS repository](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/request_reply).

<a id="getting-started"></a>

## Getting started

For *Request-Reply* communication perform the following steps:

1. Define two structures in an IDL file.
   One structure will be used as Request Topic’s data type and the other one as Reply Topic’s data type.
   ```omg-idl
   enum OperationType
   {
       ADDITION,
       SUBSTRACTION,
       MULTIPLICATION,
       DIVISION
   };

   struct RequestType
   {
       OperationType operation;
       long x;
       long y;
   };

   struct ReplyType
   {
       long long z;
   };
   ```
2. In the client application, create a DataWriter for the request and a DataReader for the reply.
   ```c++
   participant->register_type(request_type);
   participant->register_type(reply_type);

   Topic* request_topic = participant->create_topic("CalculatorRequest",
                   request_type.get_type_name(), TOPIC_QOS_DEFAULT);

   Topic* reply_topic =
           participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

   DataWriter* request_writer = publisher->create_datawriter(request_topic, DATAWRITER_QOS_DEFAULT);

   DataReader* reply_reader = subscriber->create_datareader(reply_topic, DATAREADER_QOS_DEFAULT, &listener);
   ```
3. In the server application, create a DataWriter for the reply and a DataReader for the request.
   ```c++
   participant->register_type(request_type);
   participant->register_type(reply_type);

   Topic* request_topic = participant->create_topic("CalculatorRequest",
                   request_type.get_type_name(), TOPIC_QOS_DEFAULT);

   Topic* reply_topic =
           participant->create_topic("CalculatorReply", reply_type.get_type_name(), TOPIC_QOS_DEFAULT);

   DataWriter* reply_writer = publisher->create_datawriter(reply_topic, DATAWRITER_QOS_DEFAULT);

   DataReader* request_reader = subscriber->create_datareader(request_topic, DATAREADER_QOS_DEFAULT, &listener);
   ```

<a id="sending-the-request-and-storing-the-assigned-identifier"></a>

## Sending the request and storing the assigned identifier

For sending the request, the client application should retrieve and store the internal identifier assigned to the
published sample.
Therefore the sample should be published using the overloaded [`write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) function which second argument is
a reference to a [`WriteParams`](../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) object.
The assigned identifier will be stored in the [`WriteParams`](../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE)’s attribute [`sample_identity()`](../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParams15sample_identityERK14SampleIdentity).

```c++
eprosima::fastdds::rtps::SampleIdentity my_request_sample_identity;
RequestType request;

// Fill the request

// Publish request
eprosima::fastdds::rtps::WriteParams write_params;
request_writer->write(static_cast<void*>(&request), write_params);

// Store sample identity
my_request_sample_identity = write_params.sample_identity();
```

<a id="receiving-the-request-and-sending-a-reply-associated-with-it"></a>

## Receiving the request and sending a reply associated with it

When the server application receives the request (for example through [`on_data_available()`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader)),
it has to retrieve the request’s identifier using [`sample_identity`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo15sample_identityE).

```c++
void on_data_available(
        eprosima::fastdds::dds::DataReader* reader) override
{
    RequestType request;
    eprosima::fastdds::dds::SampleInfo sample_info;

    reader->take_next_sample(&request, &sample_info);

    if (eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == sample_info.instance_state)
    {
        // Store the request identity.
        eprosima::fastdds::rtps::SampleIdentity client_request_identity = sample_info.sample_identity;
    }
}

```

After processing the request, the server should send the reply to the client with the related request attached.
This is done assigning the stored identifier in [`related_sample_identity()`](../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParams23related_sample_identityERK14SampleIdentity).

```c++
ReplyType reply;

// Fill reply

// Send reply associating it with the client request.
eprosima::fastdds::rtps::WriteParams write_params;
write_params.related_sample_identity() = client_request_identity;
reply_writer->write(reinterpret_cast<void*>(&reply), write_params);
```

<a id="identifying-the-reply-for-the-client"></a>

## Identifying the reply for the client

When the client application receives a reply (for example through [`on_data_available()`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader)),
the client application should identify if the received reply is the one expected for its request.
For this the client application has to compare the stored [`SampleIdentity`](../../api_reference/rtps/common/SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) with the incoming
[`related_sample_identity`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo23related_sample_identityE).

```c++
void on_data_available(
        eprosima::fastdds::dds::DataReader* reader) override
{
    ReplyType reply;
    eprosima::fastdds::dds::SampleInfo sample_info;

    reader->take_next_sample(&reply, &sample_info);

    if (eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == sample_info.instance_state)
    {
        if (sample_info.related_sample_identity == my_request_sample_identity)
        {
            // Work to do
        }
    }
}

```
