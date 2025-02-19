.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_service:

RPC Service
===========

Service types
-------------

Each Service is represented by a :code:`service_name`` and a :code:`service_type_name`.
Due to the fact that a Service represents a Request/Reply communication,
a Service type is represented by a pair of topic types: the Request topic type and the Reply topic type.
A new Service type can be defined creating an instance of |ServiceTypeSupport-api| class,
which is constructed using |TypeSupport-api| instances of both Request and Reply types.

.. note::
  |DomainParticipant-api| class have |DomainParticipant::register_service_type-api| and
  |DomainParticipant::unregister_service_type-api| methods
  to register and unregister a |ServiceTypeSupport-api| instance, respectively.
  Trying to register a different |ServiceTypeSupport-api| instance with a
  :code:`service_type_name` already registered will return a :code:`ReturnCode_t` error.

.. note::
  A registered |ServiceTypeSupport-api| can be found by name using |DomainParticipant::find_service_type-api| method.
  If a Service type with the provided name is not found, an empty |ServiceTypeSupport-api| is returned.

.. warning::
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
-------------------------------

A DomainParticipant can create or delete a Service instance using |DomainParticipant::create_service-api| and
|DomainParticipant::delete_service-api| methods, respectively.
An active Service can be found by name using |DomainParticipant::find_service-api| method.

When a new Service is created, DomainParticipant creates the topics
used for publishing and receiving Request and Reply samples.
When an active Service is deleted, all its internal DDS entities
(i.e: Request and Reply topics, as well as all the DDS entities that publish or are subscribed to them) are deleted.

.. note::
  Before creating a new Service,
  user must register the |ServiceTypeSupport-api| instance associated with the Service type.
  If there is no Service type registered with the :code:`service_type_name` provided,
  Service is not created and |DomainParticipant::create_service-api| method will return a null pointer.

.. note::
  When a Service is created, Request and Reply topics are created with names
  :code:`<service_name>_Request` and :code:`<service_name>_Reply`, respectively.
  The |ContentFilteredTopic-api| instance is also created
  from the Reply topic instance with name :code:`<service_name>_ReplyFiltered`.

.. note::
  During the creation of a Service, it could happen that
  Request and Reply topics are not created correctly for external reasons
  (for example, a Topic with the same name already exists).
  In this case, |Service-api| instance is deleted automatically and
  |DomainParticipant::create_service-api| method returns a null pointer.

.. warning::
  Since a Service represents a subset of DDS entities associated with a DomainParticipant,
  only the DomainParticipant that created the |Service-api| instance can delete it.
  Attempting to delete a |Service-api| instance from a different DomainParticipant
  will result in a :code:`ReturnCode_t` error.

Example
-------

The following code snippet shows how to create and delete a Service instance:

.. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--CREATE_DELETE_SERVICE_EXAMPLE
      :end-before: //!--
