.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_contentFilteredTopic_custom_filters:

Using custom filters
====================

Fast DDS API supports the creation and later registration of user's custom filters to be used in the creation of a
|ContentFilteredTopic-api|.
Required steps for using a Custom Filter are:

- :ref:`dds_layer_topic_customfilter_creation`
- :ref:`dds_layer_topic_customfilter_factory`
- :ref:`dds_layer_topic_customfilter_register`
- :ref:`dds_layer_topic_customfilter_create_topic`

.. _dds_layer_topic_customfilter_creation:

Creating the Custom Filter
--------------------------

A custom filter must be implemented by a class which inherits from |IContentFilter-api|.
Only one function must be implemented, overriding |IContentFilter::evaluate-api|.
Each time a sample is received by a |DataReader-api|, this function is called with next arguments.

- ``payload`` - The serialized payload of the sample which the custom filter has to evaluate.
- ``sample_info`` - The extra information which accompanies the sample.
- ``reader_guid`` - The GUID of the reader for which the filter is being evaluated.

The function returns a boolean where ``true`` implies the sample is accepted and ``false`` rejects the sample.

Next snippet code shows an example of Custom Filter which deserialize the ``index`` field from a serialized sample and
rejects samples where ``index`` > ``low_mark_`` and ``index`` < ``high_mark_``.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CUSTOM_FILTER_CLASS
   :end-before: //!
   :dedent: 4

.. _dds_layer_topic_customfilter_factory:

Creating the Factory for the Custom Filter
------------------------------------------

Fast DDS creates filters through a factory.
Therefore a factory which provides instantiating of a Custom Filter must be implemented.

A Custom Filter's factory has to inherit from |IContentFilterFactory-api|.
This interface requires two function to be implemented.

Each time a Custom Filter has to be created or updated, |DomainParticipant::create_contentfilteredtopic-api| calls
internally |IContentFilterFactory::create_content_filter-api| with these arguments:

- ``filter_class_name`` - Filter class name for which the factory is being called.
  It allows using the same factory for different filter classes.
- ``type_name`` - Type name of the topic being filtered.
- ``data_type`` - Type support object of the topic being filtered.
- ``filter_expression`` - Custom filter expression.
- ``filter_parameters`` - Values to set for the filter parameters (where custom filter expression has its pattern to
  substitute them).
- ``filter_instance`` - When a filter is being created, it will be ``nullptr`` on input, and will have the pointer to
  the created filter instance on output.
  When a filter is being updated, it will have a previously returned pointer on input.

This function should return the result of the operation.

When a Custom Filter should be removed, |DomainParticipant::delete_contentfilteredtopic-api| calls internally
|IContentFilterFactory::delete_content_filter-api|.
The factory must remove the provided Custom Filter's instance.

Next snippet code shows an example of Custom Filter's factory which manages instances of the Custom Filter implemented
in the previous section.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CUSTOM_FILTER_FACTORY_CLASS
   :end-before: //!
   :dedent: 4

.. _dds_layer_topic_customfilter_register:

Registering the Factory
-----------------------

To be able to use the Custom Filter in an application, the Custom Filter's factory must be registered in the
|DomainParticipant-api|.
Next snippet code shows how to register a factory through API function
|DomainParticipant::register_content_filter_factory-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CUSTOM_FILTER_REGISTER_FACTORY
   :end-before: //!
   :dedent: 8

.. _dds_layer_topic_customfilter_create_topic:

Creating a ContentFilteredTopic using the Custom Filter
-------------------------------------------------------

:ref:`dds_layer_topic_contentFilteredTopic_creation` explains how to create a |ContentFilteredTopic-api|.
In the case of using a Custom Filter, |DomainParticipant::create_contentfilteredtopic-api| has an overload adding an
argument to select the Custom Filter.

Next snippet code shows how to create a |ContentFilteredTopic-api| using the Custom Filter.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CUSTOM_FILTER_CREATE_TOPIC
   :end-before: //!
   :dedent: 8
