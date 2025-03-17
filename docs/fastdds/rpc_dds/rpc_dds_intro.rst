.. _rpc_dds_intro:

RPC over DDS
============

.. _RPC over DDS specification: https://www.omg.org/spec/DDS-RPC/1.0/PDF

*Remote Procedure Calls* (see `RPC over DDS specification`_), also known as RPC,
is a type of bidirectional communication used in a request-reply pattern.
The RPC architecture is based on the client-server model: the client sends a request to the server,
and the server sends one or more responses (replies) back to the client.
In the context of Data Distribution Service (DDS), the client and server are modeled using DomainParticipants
and they contain all the necessary objects to communicate with each other.
Each client (internally represented by a *Requester* entity) sends a request sample
to the server through a request topic;
the server (internally represented by a *Replier* entity) processes it and sends a reply sample back to the client
through a different topic (reply topic).
All the Requesters and Repliers internally have a DataWriter and DataReader to send and receive samples.

.. toctree::
    :maxdepth: 2

    /fastdds/rpc_dds/request_reply_api/request_reply_api_intro
