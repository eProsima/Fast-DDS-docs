.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _dds_layer_publisher:

Publisher
=========

A publication is defined by the association of a |DataWriter|
to a |Publisher|.
To start publishing the values of a data instance, the application creates a new
DataWriter in a Publisher.
This DataWriter will be bound to the |Topic|
that describes the data type that is being transmitted.
Remote subscriptions that match with this
Topic will be able to receive the data value updates from the DataWriter.


.. image:: /01-figures/publisher_class_diagram.svg

.. toctree::

    /fastdds/dds_layer/publisher/publisher/publisher
    /fastdds/dds_layer/publisher/publisherListener/publisherListener
    /fastdds/dds_layer/publisher/publisher/createPublisher
    /fastdds/dds_layer/publisher/dataWriter/dataWriter
    /fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener
    /fastdds/dds_layer/publisher/dataWriter/createDataWriter
    /fastdds/dds_layer/publisher/dataWriter/publishingData

