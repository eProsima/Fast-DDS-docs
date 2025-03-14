.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_topicListener:

TopicListener
=============

|TopicListener-api| is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_topic_topic`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

TopicListener has the following callback:

* |TopicListener::on_inconsistent_topic-api|: A remote Topic is discovered with the same name
  but different characteristics as another locally created Topic.

.. warning::
    Currently |TopicListener::on_inconsistent_topic-api| is not implemented (it will never be called), and will be
    implemented on a future release of *Fast DDS*.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_TOPIC_LISTENER_SPECIALIZATION
    :end-before: //!
