.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _rpc_dds_intro:

RPC over DDS
============

.. _RPC over DDS specification: https://www.omg.org/spec/DDS-RPC/1.0/PDF

*Remote Procedure Calls* (see `RPC over DDS specification`_), also known as RPC,
is a type of bidirectional communication, used in a request-reply pattern, that can be implemented
using DDS entities such as DomainParticipants, Topics, DataWriters and DataReaders.

The RPC architecture is based on the client-server model: the client sends a request to the server,
and the server sends one response (reply) back to the client.

The RPC specification provides two different APIs to build RPC over DDS applications:

* *A low-level request/reply API* (see :ref:`request_reply_api_intro`), which is based on the DDS API.
  Clients and Servers are modeled by a *Requester* or *Replier* entities, respectively,
  and created from a DomainParticipant.

  Each *Requester* and *Replier* contains a DataWriter and DataReader to send and receive samples.
  Requester sends request samples to the the Replier through a request topic,
  and the Replier, after processing the data and computing the result of the operation,
  sends reply samples back to the Requester through a different topic (reply topic).

  This API is intended to be used internally by the *Fast DDS-Gen* tool, so it is not recommended
  to be used directly by the user.

* *A high-level function-call style based API*, built on top of the request/reply API, and generated automatically
  by the *Fast DDS-Gen* tool from an IDL file, containing an interface with the operations to be used by both client
  and server (see :ref:`fastddsgen_interfaces_introduction`).

  Once the user declares the methods in the IDL file, *Fast DDS-Gen* generates the source code required
  to make remote invokations in a function-call, user-friendly style, and provides a source file to allow the user to
  implement the operations in the server side:

.. image:: ../../_static/rpc_diagrams.png
   :width: 100%
   :align: center
   :alt: RPC over DDS API comparison

.. include:: includes/request_reply_api_intro.rst
.. include:: includes/rpc_service.rst
.. include:: includes/rpc_requester.rst
.. include:: includes/rpc_replier.rst
.. include:: includes/rpc_exceptions.rst
.. include:: includes/rpc_data_streaming_intro.rst
.. include:: includes/rpc_data_streaming_interfaces.rst
