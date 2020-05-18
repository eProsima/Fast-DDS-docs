.. _dds_layer_topic:

Topic
=====

A :ref:`dds_layer_topic_topic` conceptually fits between publications and subscriptions.
Each publication channel must be unambiguously identified by the subscriptions in order to receive only the data flow
they are interested in, and not data from other publications.
A :ref:`dds_layer_topic_topic` serves this purpose, allowing publications and subscriptions that share the same
:ref:`dds_layer_topic_topic` to match and start communicating.
In that sense, the :ref:`dds_layer_topic_topic` acts as a description for a data flow.

Publications are always linked to a single :ref:`dds_layer_topic_topic`, while subscriptions are linked to a
broader concept of :ref:`dds_layer_topic_topicDescription`.

.. figure:: /01-figures/topic_class_diagram.svg
    :align: center

    Topic class diagram

.. toctree::

    /fastdds/dds_layer/topic/topic/topic
    /fastdds/dds_layer/topic/topicDescription/topicDescription
    /fastdds/dds_layer/topic/topicListener/topicListener
    /fastdds/dds_layer/topic/typeSupport/typeSupport
    /fastdds/dds_layer/topic/topic/createTopic

