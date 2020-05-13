.. _dds_layer_publisher:

Publisher
=========

A publication is defined by the association of a :ref:`dds_layer_publisher_dataWriter`
to a :ref:`dds_layer_publisher_publisher`:

 * The :ref:`dds_layer_publisher_publisher` is the object responsible for data distribution.
   It defines how and when the data should be transmitted.

 * The :ref:`dds_layer_publisher_dataWriter` is the object that informs a :ref:`dds_layer_publisher_publisher`
   about the existence and value of data instances of a given type.

This association expresses the intent of the application to publish the data described by the
:ref:`dds_layer_publisher_dataWriter` in the context provided by the :ref:`dds_layer_publisher_publisher`.

When data-object values have been communicated to the :ref:`dds_layer_publisher_publisher`
through the appropriate :ref:`dds_layer_publisher_dataWriter`,
it is the :ref:`dds_layer_publisher_publisher`'s responsibility to perform the distribution according to its own QoS,
or the QoS attached to the corresponding :ref:`dds_layer_publisher_dataWriter`.


.. image:: /01-figures/publisher_class_diagram.svg

.. toctree::

    /fastdds/dds_layer/publisher/publisher/publisher
    /fastdds/dds_layer/publisher/publisherListener/publisherListener
    /fastdds/dds_layer/publisher/dataWriter/dataWriter
    /fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener
