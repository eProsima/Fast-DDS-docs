.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_contentFilteredTopic:

ContentFilteredTopic
====================

A |ContentFilteredTopic-api| is a specialization of the broader concept of :ref:`dds_layer_topic_topicDescription`.
A ContentFilteredTopic is a Topic with filtering properties.
It makes it possible to subscribe to a Topic while at the same time specify interest on a subset of the Topic's data.

.. important::
  Note that a ContentFilteredTopic can only be used to create a DataReader, not a DataWriter.

A ContentFilteredTopic provides a relationship between a :ref:`dds_layer_topic_topic`, called the related topic, and
some user-defined filtering properties:

* A **filter expression**, which establishes a logical expression on the content of the related topic.
  It is similar to the WHERE clause in a SQL statement.
* A list of **expression parameters**, which give values to the parameters present in the filter expression.
  There must be one parameter string for each parameter in the filter expression.

Note that a ContentFilteredTopic is *not* an Entity, and thus it has neither QoS nor listener.
A DataReader created with a ContentFilteredTopic will use the QoS from the related topic.
Multiple DataReaders can be created for the same ContentFilteredTopic, and changing the filter properties of a
ContentFilteredTopic will affect all DataReaders using it.

Please refer to :ref:`dds_layer_topic_filtering_data_on_topic` and
:ref:`dds_layer_topic_contentFilteredTopic_writer_side` for more information about how to use
|ContentFilteredTopic-api|.
