.. _dds_layer_topic_topicListener:

TopicListener
=============

:class:`TopicListener` is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_topic_topic`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:class:`TopicListener` has the following callbacks:

 * **on_inconsistent_topic**: A remote :ref:`dds_layer_topic_topic` is discovered with the same name
   but different characteristics as another locally created :ref:`dds_layer_topic_topic`.

