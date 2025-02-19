.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_requester:

RPC Requester
=============

Each Requester is formed by a |DataWriter|, which is used to publish Request samples,
and a |DataReader|, which is used to receive Reply samples.
Since each Requester contains a subset of DDS entities that participate in the same RPC Request/Reply communication,
each Requester is associated with an active |Service|.
The name of the service associated with a Requester can be accessed using |Requester::get_service_name-api| method.

Creating a Requester
--------------------

A new Requester instance can be created at an active |Service|
using |DomainParticipant::create_service_requester-api| method.
Each Service can contain multiple Requester instances.

When a Requester is created,
a new DataWriter on the Request topic and a new DataReader on the filtered Reply topic are created.
DataWriter and DataReader QoS policies can be set during the Requester creation using a |RequesterQos-api| instance.

User can access to the DataWriter and DataReader instances using |Requester::get_requester_writer-api| and
|Requester::get_requester_reader-api| methods, respectively.

.. note::
  Before creating a new Requester, user must create its associated |Service-api| instance in the DomainParticipant.
  If a null pointer or a Service associated with a different participant are provided,
  Requester is not created and |DomainParticipant::create_service_requester-api| method returns a null pointer.

.. note::
  Before creating a new Requester, Service validates the provided |RequesterQos-api| instance.
  If some field is not valid, Service will notify it to the user using a log message error and Requester is not created.
  User must ensure that Service configuration in |RequesterQos-api|
  match with the |Service-api| instance configuration and that |ReliabilityQosPolicyKind-api|
  of both DataWriter and DataReader are set to |RELIABLE_RELIABILITY_QOS-api|.
.. note::
  When a Requester is created, it could happen that DataWriter and DataReader
  are not created correctly for external reasons.
  In this case, the Requester instance is deleted automatically and
  |DomainParticipant::create_service_requester-api| method returns a null pointer.

Deleting a Requester
--------------------

A Requester instance can be deleted using |DomainParticipant::delete_service_requester-api| method.
When a Requester is deleted, all its internal DDS entities (i.e: DataWriter and DataReader)
are deleted and it is unregistered from its associated Service.

.. note::
  If there is no Service with the provided :code:`service_name`,
  Requester is not valid or it is associated with a different Service,
  |DomainParticipant::delete_service_requester-api| method will return a :code:`ReturnCode_t`` error.

Sending and receiving samples
-----------------------------

Request samples can be sent using the |Requester::send_request-api| method. When this method is called,
the created DataWriter sends a new Request sample with the provided data and
fills the :code:`related_sample_identity` field in the input |RequestInfo-api| struct
with the |GUID_t-api| of the DataWriter and a sequence number.
Similarly, received Reply samples are taken using |Requester::take_reply-api| method.
When this method is called, Requester takes received Reply samples from the history of the DataReader
and fills the provided |RequestInfo-api| struct with the :code:`related_sample_identity` of the received Reply sample.
This way, user can match Request and Reply samples by checking if the :code:`related_sample_identity`
attributes of its |RequestInfo-api| instances are equal.

.. warning::
  If a Request sample is sent before discovering a |Replier|,
  the middleware will not be able to deliver the Request sample and will discard it.
  The endpoint matching algorithm described in RPC over DDS Standard 7.6.2.2 will be implemented in future releases.

.. warning::
  RPC over DDS implementation in *Fast DDS* is designed to be incompatible with |Listeners|.
  If user needs to process status changes, it can be done creating a |WaitSet| on a different thread.

Example
-------

The following code snippet shows how to use a Requester instance:

.. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--RPC_REQUESTER_EXAMPLE
      :end-before: //!--
