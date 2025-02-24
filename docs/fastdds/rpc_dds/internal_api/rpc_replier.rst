.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_replier:

RPC Replier
===========

Each Replier is formed by a |DataWriter|, which is used to publish Reply samples,
and a |DataReader|, which is used to receive Request samples.
Since each Replier contains a subset of DDS entities that participate in the same RPC Request/Reply communication,
each Replier is associated with an active |Service|.
The name of the service associated with a Replier can be accessed using |Replier::get_service_name-api| method.

Creating a Replier
------------------

A new Replier instance can be created at an active Service using
|DomainParticipant::create_service_replier-api| method.
Each Service can contain multiple Replier instances.

When a Replier is created, a new DataWriter on the Reply topic and a new DataReader on Request topic are created.
DataWriter and DataReader QoS policies can be set during the Replier creation using a |ReplierQos-api| instance.

User can access to the DataWriter and DataReader instances using |Replier::get_replier_writer-api| and
|Replier::get_replier_reader-api| methods, respectively.

.. note::
  Before creating a new Replier, user must create its associated |Service-api| instance in the DomainParticipant.
  If a null pointer or a Service associated with a different participant are provided,
  Replier is not created and |DomainParticipant::create_service_replier-api| method returns a null pointer.

.. note::
  Before creating a new Replier, Service validates the provided |ReplierQos-api| instance.
  If some field is not valid, Service will notify it to the user using a log message error and Replier is not created.
  User must ensure that Service configuration in |ReplierQos-api| match
  with the |Service-api| instance configuration and that |ReliabilityQosPolicyKind-api|
  of both DataWriter and DataReader are set to |RELIABLE_RELIABILITY_QOS-api|.

.. note::
  When a Replier is created, it could happen that DataWriter and DataReader
  are not created correctly for external reasons.
  In this case, the Replier instance is deleted automatically and
  |DomainParticipant::create_service_replier-api| method returns a null pointer.

Deleting a Replier
------------------

A Replier instance can be deleted using |DomainParticipant::delete_service_replier-api| method.
When a Replier is deleted, all its internal DDS entities (i.e: DataWriter and DataReader)
are deleted and it is unregistered from its associated Service.

.. note::
  If there is no |Service-api| with the provided :code:`service_name`,
  Replier is not valid or it is associated with a different Service,
  |DomainParticipant::delete_service_replier-api| method will return a :code:`ReturnCode_t` error.

Sending and receiving samples
-----------------------------

Request samples are taken using the |Replier::take_request-api| method.
When this method is called, Replier takes received Request samples from the history of the DataReader
and fills the :code:`related_sample_identity` member of the provided |RequestInfo-api| struct
with the :code:`related_sample_identity`` of the received Request sample.
After processing the Request, Replier sends a new Reply sample using the |Replier::send_reply-api| method
and passing the previously created |RequestInfo-api| as an input parameter.
When |Replier::send_reply-api| method is called,
the created DataWriter sends a new Reply sample with the provided data and
the :code:`related_sample_identity` of the Request sample that originated the Reply,
to allow Request and Reply samples correlation at the Requester side.

.. warning::
  If a Reply sample is sent before discovering the Requester Reply topic DataReader,
  the middleware will not be able to deliver the Reply sample and will discard it.
  The endpoint matching algorithm described in RPC over DDS Standard will be implemented in future releases.

.. warning::
  RPC over DDS implementation in *Fast DDS* is designed to be incompatible with |Listeners|.
  If user needs to process status changes, it can be done creating a |WaitSet| on a different thread.

Example
-------

The following code snippet shows how to use a Replier instance:

.. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--RPC_REPLIER_EXAMPLE
      :end-before: //!--
