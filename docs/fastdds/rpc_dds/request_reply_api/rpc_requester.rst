.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_requester:

RPC Requester
=============

A |Requester| is the RPC Entity used in the communication at the client side,
sending Request samples and processing the received Reply samples.

Since each Requester is used in only one Request/Reply communication, represented by a Service,
all of them are registered in a |Service| instance when created.
The name of the service associated with a Requester can be accessed using |Requester::get_service_name-api| method.

Similarly to the rest of the RPC entities, |Requester-api| instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Requester contains two DDS entities: a |DataWriter|, which is used to publish Request samples,
  and a |DataReader|, which is used to receive Reply samples.

* Each disabled Requester does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot send requests nor take replies.

For consistency, the states of each Requester instance and its associated Service
must follow the compatibility rule below:

.. math::
  requester\_state \leq service\_state

where :code:`requester_state` is the state of the Requester instance and :code:`service_state`
is the state of the associated Service, and the order of the states is defined as :math:`disabled < enabled`.

Creating a Requester
--------------------

A new Requester instance can be created in an enabled or disabled Service using
|DomainParticipant::create_service_requester-api| method.
When a new Requester instance is created, it is registered internally in the Service.
Each Service can contain multiple Requester instances.

.. note::
  Following the state compatibility rule, calling |DomainParticipant::create_service_requester-api| using a
  disabled |Service-api| instance as input parameter creates a disabled |Requester-api|.

  Reciprocally, calling |DomainParticipant::create_service_requester-api| with an enabled |Service-api| instance tries
  to return an enabled |Requester-api|.
  If the process of enabling the Requester fails (i.e., the creation of the internal DDS entities),
  |DomainParticipant::create_service_requester-api| returns :code:`nullptr`.

All Requester's DDS entities Qos can be configured manually when
creating the |Requester-api| instance using |RequesterQos-api|.
Before creating the Requester, Service validates |DataWriterQos-api| and |DataReaderQos-api| provided
in the |RequesterQos-api| instance:
if some field is not valid, Service will notify it to the user using a log message error and Requester is not created.

User must ensure that |ReliabilityQosPolicyKind-api|
of both DataWriter and DataReader are set to |RELIABLE_RELIABILITY_QOS-api|.
This is configured automatically when a new |RequesterQos-api| instance is created.

.. note::
  When a new |RequesterQos-api| instance is created, |HistoryQosPolicyKind-api| and
  |DurabilityQosPolicyKind-api| are by default |KEEP_ALL_HISTORY_QOS-api| and |VOLATILE_DURABILITY_QOS-api|
  in both DataWriter and DataReader QoS, respectively.

.. note::
  Before creating a new Requester, user must create its associated |Service-api| instance in the DomainParticipant.
  If a null pointer or a Service associated with a different participant are provided,
  Requester is not created and |DomainParticipant::create_service_requester-api| method returns a null pointer.

Enabling and disabling a Requester
----------------------------------
|Requester-api| instances can be enabled or disabled using
|RPCEntity::enable-api| and |RPCEntity::close-api| methods, respectively.

When a disabled |Requester-api| instance is enabled, a new DataReader on the content filtered Reply topic and
a new DataWriter on Request topic are created using the Qos configured at creation through |RequesterQos-api|.

User can access to the DataWriter and DataReader instances using |Requester::get_requester_writer-api| and
|Requester::get_requester_reader-api| methods, respectively.

Reciprocally, when an enabled Requester is disabled, its respective DataWriter and DataReader are destroyed,
making the Requester not participate in the communication through the Service.

.. warning::
  A disabled Requester does not contain DDS entities, so |Requester::get_requester_writer-api| and
  |Requester::get_requester_reader-api| return a null pointer in this case.
  The user must be responsible for checking that the Requester is enabled
  using the |RPCEntity::is_enabled-api| method before accessing the Requester's internal DDS entities.

Deleting a Requester
--------------------

A |Requester-api| instance can be unregistered from a Service and deleted using
|DomainParticipant::delete_service_requester-api| method.

This method can be called on Requesters in any state: if enabled, it will try to disable the instance,
returning a :code:`ReturnCode_t` error if it was not possible to close the Requester.

.. note::
  If there is no |Service-api| with the provided :code:`service_name`
  or the Requester is associated with a different Service,
  |DomainParticipant::delete_service_requester-api| method will return a :code:`ReturnCode_t` error.

Sending and receiving samples
-----------------------------

Request samples can be sent using the |Requester::send_request-api| method.
When this method is called, the created DataWriter sends a new Request sample with the provided data and
fills the :code:`related_sample_identity` field in the input |RequestInfo-api| struct
with the |GUID_t-api| of the DataWriter and a sequence number.

Similarly, received Reply samples are taken using |Requester::take_reply-api| method.
When this method is called, Requester takes received Reply samples from the history of the DataReader
and fills the provided |RequestInfo-api| struct with the :code:`related_sample_identity` of the received Reply sample.
This way, user can match Request and Reply samples by checking if the :code:`related_sample_identity`
attributes of their |RequestInfo-api| instances are equal.

The Requester public API provides two overloads for |Requester::take_reply-api|,
depending on whether you want to take only the next sample or all samples from the DataReader history.

.. note::
  In case of using the overload that takes all samples, the loan must be returned to avoid memory problems.
  The user must therefore be responsible for calling the |Requester::return_loan-api| method after
  finishing processing the samples.

.. warning::
  If a Request sample is sent before matching a |Replier| completely, |Requester::send_request-api| method
  will fail, returning ``RETCODE_PRECONDITION_NOT_MET``.

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
      :dedent: 8
