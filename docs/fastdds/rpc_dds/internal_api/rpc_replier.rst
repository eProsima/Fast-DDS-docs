.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_replier:

RPC Replier
===========

A |Replier| is the RPC Entity used in the communication at the server side, processing the received Request samples
and sending Reply samples back to the Requester when the result of the operation is ready.

Since each Replier is used in only one Request/Reply communication, represented by a Service, all of them are registered
in a |Service| instance when created.
The name of the service associated with a Replier can be accessed using |Replier::get_service_name-api| method.

Similarly to the rest of the RPC entities, |Replier-api| instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Replier contains two DDS entities: a |DataWriter|, which is used to publish Reply samples,
  and a |DataReader|, which is used to receive Request samples.

* Each disabled Replier does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot take requests nor send replies.

For consistency, the states of each Replier instance and its associated Service
must follow the compatibility rule below:

.. math::
  replier\_state \leq service\_state

where :code:`replier_state` is the state of the Replier instance and :code:`service_state`
is the state of the associated Service, and the order of the states is defined as :math:`disabled < enabled`.

Creating a Replier
------------------

A new Replier instance can be created in an enabled or disabled Service using
|DomainParticipant::create_service_replier-api| method.
When a new Replier instance is created, it is registered internally in the Service.
Each Service can contain multiple Replier instances.

.. note::
  Following the state compatibility rule, calling |DomainParticipant::create_service_replier-api| using a
  disabled |Service-api| instance as input parameter creates a disabled |Replier-api|.

  Reciprocally, calling |DomainParticipant::create_service_replier-api| with an enabled |Service-api| instance tries
  to return an enabled |Replier-api|.
  If the process of enabling the Replier fails (i.e., the creation of the internal DDS entities),
  |DomainParticipant::create_service_replier-api| returns :code:`nullptr`.

All Replier's DDS entities Qos can be configured manually when
creating the |Replier-api| instance using |ReplierQos-api|.
Before creating the Replier, Service validates the provided |ReplierQos-api| instance:
if some field is not valid, Service will notify it to the user using a log message error and Replier is not created.

User must ensure that Service configuration in |ReplierQos-api| match
with the |Service-api| instance configuration and that |ReliabilityQosPolicyKind-api|
of both DataWriter and DataReader are set to |RELIABLE_RELIABILITY_QOS-api|.

.. note::
  Before creating a new Replier, user must create its associated |Service-api| instance in the DomainParticipant.
  If a null pointer or a Service associated with a different participant are provided,
  Replier is not created and |DomainParticipant::create_service_replier-api| method returns a null pointer.

Enabling and disabling a Replier
--------------------------------
|Replier-api| instances can be enabled or disabled using
|RPCEntity::enable-api| and |RPCEntity::close-api| methods, respectively.

When a disabled |Replier-api| instance is enabled, a new DataWriter on the Reply topic and
a new DataReader on Request topic are created using the Qos configured at creation through |ReplierQos-api|.

User can access to the DataWriter and DataReader instances using |Replier::get_replier_writer-api| and
|Replier::get_replier_reader-api| methods, respectively.

Reciprocally, when an enabled Replier is disabled, its respective DataWriter and DataReader are destroyed,
making the Replier not participate in the communication through the Service.

.. warning::
  A disabled Replier does not contain DDS entities, so |Replier::get_replier_writer-api| and
  |Replier::get_replier_reader-api| return a null pointer in this case.
  The user must be responsible for checking that the Replier is enabled
  using the |RPCEntity::is_enabled-api| method before accessing the Replier's internal DDS entities.

Deleting a Replier
------------------

A |Replier-api| instance can be unregistered from a Service and deleted using
|DomainParticipant::delete_service_replier-api| method.

This method can be called on Repliers in any state: if enabled, it will try to disable the instance,
returning a :code:`ReturnCode_t` error if it was not possible to close the Replier.

.. note::
  If there is no |Service-api| with the provided :code:`service_name`
  or the Replier is associated with a different Service,
  |DomainParticipant::delete_service_replier-api| method will return a :code:`ReturnCode_t` error.

Sending and receiving samples
-----------------------------

Request samples are taken using the |Replier::take_request-api| method.
When this method is called, Replier takes received Request samples from the history of the internal DataReader
and fills the :code:`related_sample_identity` member of the provided |RequestInfo-api| struct
with the :code:`related_sample_identity`` of the received Request sample.

The Replier public API provides two overloads for |Replier::take_request-api|,
depending on whether you want to take only the next sample or all samples from the DataReader history.

.. note::
  In case of using the overload that takes all samples, the loan must be returned to avoid memory problems.
  The user must therefore be responsible for calling the |Replier::return_loan-api| method after
  finishing processing the samples.

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
