.. _dds_layer_publisher:

Publisher
=========

A publication is defined by the association of a :ref:`dds_layer_publisher_dataWriter`
to a :ref:`dds_layer_publisher_publisher`.
To start publishing the values of a data instance, the application creates a new
:ref:`dds_layer_publisher_dataWriter` in a :ref:`dds_layer_publisher_publisher`.
This :ref:`dds_layer_publisher_dataWriter` will be bound to the :ref:`dds_layer_topic_topic`
that describes the data type that is being transmitted. Remote subscriptions that match with this
:ref:`dds_layer_topic_topic` will be able to receive the data value updates from the
:ref:`dds_layer_publisher_dataWriter`.


.. image:: /01-figures/publisher_class_diagram.svg

.. toctree::

    /fastdds/dds_layer/publisher/publisher/publisher
    /fastdds/dds_layer/publisher/publisherListener/publisherListener
    /fastdds/dds_layer/publisher/publisher/createPublisher
    /fastdds/dds_layer/publisher/dataWriter/dataWriter
    /fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener
    /fastdds/dds_layer/publisher/dataWriter/createDataWriter
    /fastdds/dds_layer/publisher/dataWriter/publishingData

