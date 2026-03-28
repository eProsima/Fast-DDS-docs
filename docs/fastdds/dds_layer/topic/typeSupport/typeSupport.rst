.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_definition_data_types:

Definition of data types
========================

The definition of the data type exchanged in a :ref:`dds_layer_topic_topic` is divided in
two classes: the |TypeSupport-api| and the |TopicDataType-api|.

TopicDataType describes the data type exchanged between a publication and a subscription, i.e.,
the data corresponding to a Topic.
The user has to create a specialized class for each specific type that will be used by the application.

Any specialization of TopicDataType must be registered in the :ref:`dds_layer_domainParticipant`
before it can be used to create Topic objects.
A TypeSupport object encapsulates an instance of TopicDataType, providing the functions needed to
register the type and interact with the publication and subscription.
To register the data type, create a new TypeSupport with a TopicDataType instance
and use the |TypeSupport::register_type-api| member function on the TypeSupport.
Then the Topic can be created with the registered type name.

.. note::

   Registering two different data types on the same DomainParticipant with identical names is not
   allowed and will issue an error.
   However, it is allowed to register the same data type within the same DomainParticipant,
   with the same or different names.
   If the same data type is registered twice on the same DomainParticipant with the same
   name, the second registering will have no effect, but will not issue any error.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_TYPE_REGISTER
   :end-before: //!
   :dedent: 8


.. _dds_layer_topic_dynamic_data_types:

Dynamic data types
------------------

Instead of directly writing the specialized |TopicDataType-api| class, it is possible to dynamically define
data types following the OMG Extensible and Dynamic Topic Types for DDS interface.
Data types can also be described on an XML file that is dynamically loaded.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DYNAMIC_TYPES
   :end-before: //!
   :dedent: 8

A complete description of the dynamic definition of types can be found on the :ref:`dynamic-types` section.


.. _dds_layer_topic_keyed_data_types:

Data types with a key
---------------------

Data types that define a set of fields to form a unique key can distinguish different data sets
within the same data type.

To define a keyed Topic, the |TopicDataType::getKey-api| member function on the |TopicDataType-api|
has to be overridden to return the appropriate key value according to the data fields.
Additionally, the |TopicDataType::m_isGetKeyDefined-api| data member needs to be set to ``true`` to let the entities
know that this is a keyed Topic and that |TopicDataType::getKey-api| should be used.
Types that do not define a key will have |TopicDataType::m_isGetKeyDefined-api| set to false.

There are three ways to implement keys on the TopicDataType:

* Adding a ``@Key`` annotation to the members that form the key in the IDL file when using |Fast DDS-Gen|.
* Adding the attribute ``Key`` to the member and its parents when using :ref:`dynamic-types`.
* Manually implementing the |TopicDataType::getKey-api| member function on the TopicDataType and setting
  the |TopicDataType::m_isGetKeyDefined-api| data member value to ``true``.

Data types with key are used to define data sub flows on a single Topic.
Data values with the same key on the same Topic represent data from the same sub-flow,
while data values with different keys on the same Topic represent data
from different sub-flows.
The middleware keeps these sub-flows separated, but all will be restricted to the same QoS values of
the Topic.
If no key is provided, the data set associated with the Topic is restricted to a single flow.


.. _dds_layer_topic_type_support_context:

Type support context
--------------------

Fast DDS allows passing a user-defined *context* object to the type support callbacks, enabling
type-specific information (e.g., upper bounds for strings and sequences) to be available during
serialization, deserialization, and related operations without needing global state.

The context is represented by the |TopicDataType::Context-api| interface.
Users subclass it to carry whatever per-endpoint information their type requires:

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_TYPE_SUPPORT_CONTEXT_DEF
   :end-before: //!
   :dedent: 8

The context-aware virtual methods of |TopicDataType-api| all follow the same pattern: they receive a
``const std::shared_ptr<TopicDataType::Context>&`` as their first argument, and their default
implementation ignores the context and delegates to the corresponding context-free method.
Users may override any subset of the following methods:

- |TopicDataType::serialize_ctx-api| — serialize a sample using the context.
- |TopicDataType::deserialize_ctx-api| — deserialize a payload using the context.
- |TopicDataType::calculate_serialized_size_ctx-api| — compute the serialized size with the context.
- |TopicDataType::create_data_ctx-api| — allocate a new sample using the context.
- |TopicDataType::delete_data_ctx-api| — deallocate a sample using the context.
- |TopicDataType::compute_key_ctx-api| — extract the key using the context.
- |TopicDataType::is_bounded_ctx-api| — check whether the type is bounded with this context.
- |TopicDataType::is_plain_ctx-api| — check whether the type is plain with this context.
- |TopicDataType::construct_sample_ctx-api| — in-place construct a sample using the context.
- |TopicDataType::get_max_serialized_size_ctx-api| — return the maximum serialized size with this context.

The context is attached to a :ref:`dds_layer_publisher_dataWriter` or
:ref:`dds_layer_subscriber_dataReader` **before enabling** the entity, using
|DataWriter::set_type_support_context-api| or |DataReader::set_type_support_context-api|
respectively.
Because the context is set before the entity is enabled, it is guaranteed to be available for every
write or read operation performed during the entity's lifetime.
