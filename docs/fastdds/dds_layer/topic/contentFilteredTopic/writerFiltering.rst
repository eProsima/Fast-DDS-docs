.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_contentFilteredTopic_writer_side:

Where is filtering applied: writer vs reader side
=================================================

Content filters may be evaluated on either side, as the DataWriter obtains the filter expression from the
DataReader during discovery.

Filtering on the writer side can save network bandwidth at the cost of increasing CPU usage on the writer.

.. _dds_layer_topic_contentFilteredTopic_writer_restrictions:

Conditions for writer side filtering
------------------------------------

A DataWriter will perform filter evaluation for a DataReader when all of the following conditions are met.
Filtering will otherwise be performed by the DataReader.

- The DataWriter has infinite liveliness. See |LivelinessQosPolicy|.
- Communication with the DataReader is neither intra-process nor data-sharing.
- The DataReader is not using multicast.
- The DataWriter is filtering for no more DataReaders than the maximum value set on
  |WriterResourceLimitsQos::reader_filters_allocation-api|.
  - There is a resource-limit policy on |DataWriterQos| that controls the allocation behavior of
    writer-side filtering resources.
    Setting a maximum value of 0 disables filter evaluation on the writer side.
    A maximum value of 32 (the default value) means the writer will perform filter evaluation for
    up to 32 readers.
  - If the DataWriter is evaluating filters for *writer_resource_limits.reader_filters_allocation.maximum*
    DataReaders, and a new filtered DataReader is created, then the filter for the newly created DataReader
    will be evaluated on the reader side.
