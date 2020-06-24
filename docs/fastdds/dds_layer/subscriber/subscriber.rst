.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _dds_layer_subscriber:

Subscriber
==========

A subscription is defined by the association of a :ref:`dds_layer_subscriber_dataReader`
to a :ref:`dds_layer_subscriber_subscriber`.
To start receiving updates of a publication, the application creates a new
DataReader in a Subscriber.
This DataReader will be bound to the :ref:`dds_layer_topic_topic`
that describes the data type that is going to be received.
The DataReader will then start receiving data value updates from
remote publications that match this Topic.

When the Subscriber receives data, it informs the application that new data is available.
Then, the application can use the DataReader to get the received data.

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
    /fastdds/dds_layer/subscriber/dataReader/readingData
