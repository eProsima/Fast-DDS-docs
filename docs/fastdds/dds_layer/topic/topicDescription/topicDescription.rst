.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_topicDescription:

TopicDescription
================

|TopicDescription-api| is an abstract class that serves as the base for all classes describing a data flow.
Applications will not create instances of |TopicDescription-api| directly, they must create instances of one
of its specializations instead.
At the moment, the only specialization implemented is :ref:`dds_layer_topic_topic`.
