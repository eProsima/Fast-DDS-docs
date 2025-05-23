.. _rpc_service:

RPC Service
^^^^^^^^^^^

A Service is the RPC entity used to register all RPC entities involved in an RPC communication,
both on the client side (|Requesters|) and on the server side (|Repliers|).
Two or more participants can communicate with each other via RPC if they register the same service
and create Requesters and Repliers in it.

A service is represented by its name and a Service type,
so that two |Service-api| instances represent the same RPC service if they have the same name and type.
The name of the Service and the name of its type can be obtained through the methods |Service::get_service_name-api|
and |Service::get_service_type_name-api| respectively.

Similarly to the rest of the RPC entities, |Service-api| instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Service contains at least two DDS entities: a Request |Topic|,
  which is used to publish and receive Request samples,
  and a Reply |Topic|, which is used to publish and receive Reply samples.
  If it also contains enabled Requesters and Repliers,
  it also includes their respective internal |DataWriters| and |DataReaders|.
  Therefore, an enabled |Service-api| instance can also be understood as the subset of DDS entities
  associated with a |DomainParticipant| that participate in the same communication through RPC.

* Each disabled Replier does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot take requests nor send replies.

Service types
"""""""""""""

Due to the fact that a Service represents a Request/Reply communication,
a Service type is represented by a pair of topic types: the Request topic type and the Reply topic type.
A new Service type can be defined creating an instance of |ServiceTypeSupport-api| class,
which is constructed using |TypeSupport-api| instances of both Request and Reply types.

|DomainParticipant-api| class have |DomainParticipant::register_service_type-api| and
|DomainParticipant::unregister_service_type-api| methods
to register and unregister a |ServiceTypeSupport-api| instance, respectively.

.. note::
  Request and Reply types associated with a service type named :code:`<service_type_name>` are registered
  in the participant with the names :code:`<service_type_name>_Request`` and
  :code:`<service_type_name>_Reply`, respectively.

A registered |ServiceTypeSupport-api| can be found by name using |DomainParticipant::find_service_type-api| method;
if a Service type with the provided name is not found, an empty |ServiceTypeSupport-api| is returned.

.. note::
  Trying to register a different |ServiceTypeSupport-api| instance with a
  :code:`service_type_name` already registered will return a :code:`ReturnCode_t` error.

Registering and unregistering a |ServiceTypeSupport-api| instance involves
registering and unregistering the Request and Reply types in the participant, respectively.
If a |ServiceTypeSupport-api| contains a Request or Reply type with the same name
as a previously registered type but with a different |TypeSupport-api|,
the |DomainParticipant::register_service_type-api| method will fail.
Similarly, attempting to unregister a |ServiceTypeSupport-api| instance with a Request or Reply type
that is used by a DDS endpoint on a topic not created by the Service will fail.
To avoid these issues, it is recommended to make sure that the Request and Reply types
are reserved for RPC communication though a unique service and
not used by external DDS entities which are not part of the Service.

Creating and deleting a Service
"""""""""""""""""""""""""""""""

A DomainParticipant can create and register a
|Service-api| instance using |DomainParticipant::create_service-api| method.
Respectively, a |Service-api| can be unregistered and deleted using |DomainParticipant::delete_service-api| method.
Each registered Service can be found by name using |DomainParticipant::find_service-api| method.

.. note::
  Before creating a new Service,
  user must register the |ServiceTypeSupport-api| instance associated with the Service type.
  If there is no Service type registered with the :code:`service_type_name` provided,
  Service is not created and |DomainParticipant::create_service-api| method will return a null pointer.

When a new Service is created, it is enabled by default. It means that DomainParticipant creates the topics
used for publishing and receiving Request and Reply samples.

.. note::
  If the DomainParticipant is unable to enable the Service when it is created, it is considered an error.
  The instance is destroyed, and |DomainParticipant::create_service-api| returns :code:`nullptr`.

Only the DomainParticipant that created the |Service-api| instance can delete it.
Attempting to delete a |Service-api| instance from a different DomainParticipant
will result in a :code:`ReturnCode_t` error.

Enabling and disabling a Service
""""""""""""""""""""""""""""""""

|Service-api| instances can be enabled or disabled using
|RPCEntity::enable-api| and |RPCEntity::close-api| methods, respectively.

When a disabled |Service-api| instance is enabled, new Request and Reply topics are created in the DomainParticipant
using the registered Service type. It also creates the content filtered topic used for Reply samples.
Additionally, if it contains Requesters and Repliers, an attempt will be made to enable all possible entities.

.. note::
  Request and Reply topics are created with names
  :code:`<service_name>_Request` and :code:`<service_name>_Reply`, respectively.
  The |ContentFilteredTopic-api| instance is also created
  from the Reply topic instance with name :code:`<service_name>_ReplyFiltered`.

.. note::
  If some of the RPCEntities within the Service cannot be enabled, an error log is displayed.
  However, |RPCEntity::enable-api| ignores this failure and returns :code:`RETCODE_OK`` as long as
  the topics have been created successfully.

Reciprocally, when an enabled Service is disabled, all the DDS entities that the Service contains are destroyed,
making the Service not participate in the communication through RPC.
It implies disabling all containing Requesters and Repliers and deleting the Request and Reply topics.

.. note::
  If some of the RPCEntities within the Service cannot be disabled,
  |RPCEntity::close-api| returns an error code.

Example
"""""""

The following code snippet shows how to create and delete a Service instance:

.. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--CREATE_DELETE_SERVICE_EXAMPLE
      :end-before: //!--
      :dedent: 8
