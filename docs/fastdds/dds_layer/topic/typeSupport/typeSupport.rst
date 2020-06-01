.. include:: ../includes/aliases.rst

.. _dds_layer_definition_data_types:

Definition of data types
========================

The definition of the data type exchanged in a :ref:`dds_layer_topic_topic` is divided in
two classes: the :class:`TypeSupport` and the :class:`TopicDataType`.

:class:`TopicDataType` describes the data type exchanged between a publication and a subscription, i.e.,
the data corresponding to a :ref:`dds_layer_topic_topic`.
The user has to create a specialized class for each specific type that will be used by the application.

Any specialization of :class:`TopicDataType` must be registered in the :ref:`dds_layer_domainParticipant`
before it can be used to create :ref:`dds_layer_topic_topic` objects.
A :class:`TypeSupport` object encapsulates an instance of :class:`TopicDataType`, providing the functions needed to
register the type and interact with the publication and subscription.
To register the data type, create a new :class:`TypeSupport` with a :class:`TopicDataType` instance
and use the :func:`register_type` member function on the :class:`TypeSupport`.
Then the :ref:`dds_layer_topic_topic` can be created with the registered type name.

.. note::

   Registering two different data types on the same :ref:`dds_layer_domainParticipant` with identical names is not
   allowed and will issue an error.
   However, it is allowed to register the same data type within the same :ref:`dds_layer_domainParticipant`,
   with the same or different names.
   If the same data type is registered twice on the same :ref:`dds_layer_domainParticipant` with the same
   name, the second registering will have no effect, but will not issue any error.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_TYPE_REGISTER
   :end-before: //!
   :dedent: 8


.. _dds_layer_topic_dynamic_data_types:

Dynamic data types
------------------

Instead of directly writing the specialized :class:`TopicDataType` class, it is possible to dynamically define
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
This mechanism is implemented overriding the :func:`getKey` member function on the TopicDataType to return
the appropriate key value according to the field data.
Types that do not define a key will not override this member function.

Data types with key are used to define data sub flows on a single :ref:`dds_layer_topic_topic`.
Data values with the same key on the same :ref:`dds_layer_topic_topic` represent data from the same sub-flow,
while data values with different keys on the same :ref:`dds_layer_topic_topic` represent data
from different sub-flows.
The middleware keeps these sub-flows separated, but all will be restricted to the same QoS values of
the :ref:`dds_layer_topic_topic`.
If no key is provided, the data set associated with the :ref:`dds_layer_topic_topic` is restricted to a single flow.
