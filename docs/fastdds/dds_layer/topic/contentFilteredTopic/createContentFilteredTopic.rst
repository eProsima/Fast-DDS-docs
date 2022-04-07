.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_filtering_data_on_topic:

Filtering data on a Topic
=========================

.. _dds_layer_topic_contentFilteredTopic_creation:

Creating a ContentFilteredTopic
-------------------------------

A :ref:`dds_layer_topic_contentFilteredTopic` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a ContentFilteredTopic is done with the |DomainParticipant::create_contentfilteredtopic-api| member
function on the |DomainParticipant-api| instance, that acts as a factory for the |ContentFilteredTopic-api|.

Mandatory arguments are:

 * A string with the name that identifies the ContentFilteredTopic.

 * The related |Topic-api| being filtered.

 * A string with the filter expression indicating the conditions for a sample to be returned.

 * A list of strings with the value of the parameters present on the filter expression.

Optional arguments are:

 * A string with the name of the filter class to use for the filter creation.
   This allows the user to create filters different from the standard SQL like one
   (please refer to :ref:`dds_layer_topic_contentFilteredTopic_custom_filters`).
   Defaults to |FASTDDS_SQLFILTER_NAME-api| (``DDSSQL``).

.. important::
    Setting an empty string as filter expression results in the disabling of the filtering.
    This can be used to enable/disable the DataReader filtering capabilities at any given time by simply
    :ref:`updating the filter expression <dds_layer_topic_contentFilteredTopic_update>`.

|DomainParticipant::create_contentfilteredtopic-api| will return a null pointer if there was an error during the
operation, e.g. if the related Topic belongs to a different DomainParticipant, a Topic with the same name already
exists, syntax errors on the filter expression, or missing parameter values.
It is advisable to check that the returned value is a valid pointer.

.. note::
    Different filter classes may impose different requirements on the related Topic, the expression, or the parameters.
    The default filter class, in particular, requires that a TypeObject for the related Topic's type has been
    registered.
    When using :ref:`fastddsgen <fastddsgen_supported_options>` to generate your type support code, remember to include
    the ``-typeobject`` option so the TypeObject registration code is generated.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_CONTENT_FILTERED_TOPIC
   :end-before: //!
   :dedent: 8


.. _dds_layer_topic_contentFilteredTopic_update:

Updating the filter expression and parameters
---------------------------------------------

A ContentFilteredTopic provides several member functions for the management of the filter expression and
the expression parameters:

 * The filter expression can be retrieved with the |ContentFilteredTopic::get_filter_expression-api| member function.

 * The expression parameters can be retrieved with the |ContentFilteredTopic::get_expression_parameters-api| member
   function.

 * The expression parameters can be modified using the |ContentFilteredTopic::set_expression_parameters-api| member
   function.

 * The filter expression can be modified along with the expression parameters using the
   |ContentFilteredTopic::set_filter_expression-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_UPDATE_CONTENT_FILTERED_TOPIC
   :end-before: //!
   :dedent: 8

.. _dds_layer_topic_contentFilteredTopic_deletion:

Deleting a ContentFilteredTopic
-------------------------------

A ContentFilteredTopic can be deleted with the |DomainParticipant::delete_contentfilteredtopic-api| member function
on the DomainParticipant instance where the ContentFilteredTopic was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_CONTENT_FILTERED_TOPIC
   :end-before: //!
   :dedent: 8

