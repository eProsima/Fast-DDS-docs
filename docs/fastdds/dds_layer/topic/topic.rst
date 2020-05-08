.. _dds_layer_topic:

Topic
=====

.. image:: /01-figures/topic_class_diagram.svg

A topic conceptually fits between publications and subscriptions.
Each publication channel must be unambiguously identified by the subscriptions in order to receive only the data flow
they are interested in, and not data from other publications.
A topic serves this purpose, allowing publications and subscriptions to match and start communicating.

In that sense, the topic acts as a description for a data flow.
In its most basic form, the topic provides:
 * A name to identify the data flow.
 * A :ref:`dds_layer_topic_typeSupport` object describing the data type that is transmitted on the flow.

By definition, each Topic corresponds to a single data type.
But several Topics may refer to the same data type, allowing independent data flows even if they are transmitting the same data type.
However, if the data type is keyed, a single Topic can have data subflows.
Different data values with the same key value on the same Topic represent successive values for the same subflow,
while different data values with different key values on the same Topic represent different subflow.
The middleware keeps these subflows separated, but all will be restricted to the same QoS values of the Topic.
If no key is provided, the data set associated with the Topic is restricted to a single flow.

.. toctree::

    /fastdds/dds_layer/topic/topic/topic
    /fastdds/dds_layer/topic/topicDescription/topicDescription
    /fastdds/dds_layer/topic/topicListener/topicListener
    /fastdds/dds_layer/topic/typeSupport/typeSupport


