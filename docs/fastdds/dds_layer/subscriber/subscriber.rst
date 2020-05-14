.. _dds_layer_subscriber:

Subscriber
==========

A subscription is defined by the association of a :ref:`dds_layer_subscriber_dataReader`
to a :ref:`dds_layer_subscriber_subscriber`.
To start receiving updates of a publication, the application creates a new
:ref:`dds_layer_subscriber_dataReader` in a :ref:`dds_layer_subscriber_subscriber`.
This :ref:`dds_layer_subscriber_dataReader` will be bound to the :ref:`dds_layer_topic_topic`
that describes the data type that is going to be received.
The :ref:`dds_layer_subscriber_dataReader` will then start receiving data value updates from
remote publications that match this :ref:`dds_layer_topic_topic`.

When the :ref:`dds_layer_subscriber_subscriber` receives data, it informs the application that new data is available.
Then, the application can use the :ref:`dds_layer_subscriber_dataReader` to get the received data.

.. figure:: /01-figures/subscriber_class_diagram.svg
    :align: center

    Subscriber class diagram

.. toctree::

    /fastdds/dds_layer/subscriber/subscriber/subscriber
    /fastdds/dds_layer/subscriber/subscriberListener/subscriberListener
    /fastdds/dds_layer/subscriber/subscriber/createSubscriber
    /fastdds/dds_layer/subscriber/dataReader/dataReader
    /fastdds/dds_layer/subscriber/dataReaderListener/dataReaderListener
    /fastdds/dds_layer/subscriber/dataReader/createDataReader
    /fastdds/dds_layer/subscriber/sampleInfo/sampleInfo
