.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

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
   This allows the user to create filters different from the standard SQL like one.
   Defaults to ``FASTDDS_SQLFILTER_NAME``.

|DomainParticipant::create_contentfilteredtopic-api| will return a null pointer if there was an error during the
operation, e.g. if the related topic belongs to a different participant, a topic with the same name already exists,
syntax errors on the filter expression, or missing parameter values.
It is advisable to check that the returned value is a valid pointer.

.. note::
    Different filter classes may impose different requirements on the related topic, the expression, or the parameters.
    The default filter class, in particular, requires that a TypeObject for the related topic's type has been registered.
    When using fastddsgen to generate your type support code, remember to include the ``-typeobject`` option so the
    TypeObject registration code is generated.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_CONTENT_FILTERED_TOPIC
   :end-before: //!
   :dedent: 8
