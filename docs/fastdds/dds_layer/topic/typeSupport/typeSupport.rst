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
and use the :func:`register_type` member function on the TypeSupport.
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

* Adding a ``@Key`` annotation to the members that form the key in the IDL file when using *Fast DDS-Gen*.
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
