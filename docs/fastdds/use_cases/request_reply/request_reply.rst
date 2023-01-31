.. include:: ../../../03-exports/aliases-api.include

.. _use-case-request-reply:

Request-Reply communication
===========================

This section explains how to configure a Request-Reply communication in *Fast DDS* between two applications.
A *Client application* will send a *Request* to the *Server application* and this one, after processing the request,
will send a *Reply* to the *Client application*.

.. uml::
   :align: center

    node "Client application" as client {
    }

    node "Server application" as server {
    }

    client --> server : Request
    server --> client : Reply

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Overview
--------

This kind of communication involves in the DDS paradigm the usage of two *Topics*: one for sending requests
(Request Topic) and the other one for sending replies (Reply Topic).
For managing these *Topics* four DDS entities are involved: a DataReader and a DataWriter for each *Topic*.
The DDS communication schema will be:

.. uml::
   :align: center

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

The key for making *Request-Reply* work is relating the Request with the Reply in the client's side.
*Fast DDS* API provides |SampleIdentity-api| to achive this.

A full example can be found in `Fast DDS repository`_.

Getting started
---------------

For *Request-Reply* communication perform the following steps:

1. Define two structures in an IDL file.
   One structure will be used as Request Topic's data type and the other one as Reply Topic's data type.

   .. code-block:: idl

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

2. In the client application, create a DataWriter for the request and a DataReader for the reply.

    .. literalinclude:: ../../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 4
       :start-after: //REQUEST_REPLY_EXAMPLE_CLIENT_CREATE_ENTITIES
       :end-before: //!

3. In the server application, create a DataWriter for the reply and a DataReader for the request.

    .. literalinclude:: ../../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 4
       :start-after: //REQUEST_REPLY_EXAMPLE_SERVER_CREATE_ENTITIES
       :end-before: //!

Sending the request and storing the assigned identifier
-------------------------------------------------------

For sending the request, the client application should retrieve and store the internal identifier assigned to the
published sample.
Therefore the sample should be published using the overloaded |DataWriter::write-api| function which second argument is
a reference to a |WriteParams-api| object.
The assigned identifier will be stored in the |WriteParams-api|'s attribute |WriteParams-sample_identity-api|.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 4
   :start-after: //REQUEST_REPLY_EXAMPLE_CLIENT_RETRIEVE_ID
   :end-before: //!

Receiving the request and sending a reply associated with it
------------------------------------------------------------

When the server application receives the request (for example through |DataReaderListener::on_data_available-api|),
it has to retrieve the request's identifier using |SampleInfo::sample_identity-api|.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 4
   :start-after: //REQUEST_REPLY_EXAMPLE_SERVER_GET_ID
   :end-before: //!

After processing the request, the server should send the reply to the client with the related request attached.
This is done assigning the stored identifier in |WriteParams-related_sample_identity-api|.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 4
   :start-after: //REQUEST_REPLY_EXAMPLE_SERVER_SEND_REPLY
   :end-before: //!

Identifying the reply for the client
------------------------------------

When the client application receives a reply (for example through |DataReaderListener::on_data_available-api|),
the client application should identify if the received reply is the one expected for its request.
For this the client application has to compare the stored |SampleIdentity-api| with the incoming
|SampleInfo::related_sample_identity-api|.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 4
   :start-after: //REQUEST_REPLY_EXAMPLE_CLIENT_RECEIVE_REPLY
   :end-before: //!

.. _Fast DDS repository: https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/RequestReplyExample
