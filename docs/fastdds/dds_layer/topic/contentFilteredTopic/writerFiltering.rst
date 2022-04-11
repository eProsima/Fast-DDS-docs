.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_contentFilteredTopic_writer_side:

Where is filtering applied: writer vs reader side
=================================================

:ref:`Content filters <dds_layer_topic_contentFilteredTopic>` may be evaluated on either side, as the DataWriter obtains
the filter expression from the DataReader during discovery.
Filtering on the writer side can save network bandwidth at the cost of increasing CPU usage on the writer.

.. _dds_layer_topic_contentFilteredTopic_writer_restrictions:

Conditions for writer side filtering
------------------------------------

A DataWriter will perform filter evaluation in the DataReader stead whenever all of the following conditions are met;
filtering will otherwise be performed by the DataReader.

- The DataWriter has infinite liveliness.
  See |LivelinessQosPolicy|.
- Communication with the DataReader is neither :ref:`intra-process <intraprocess-delivery>` nor
  :ref:`data-sharing <datasharing-delivery>`.
- The DataReader is not using multicast.
- The DataWriter is filtering for no more DataReaders than the maximum value set on
  |WriterResourceLimitsQos::reader_filters_allocation-api|.

  - There is a :ref:`resource-limit policy <writerresourcelimitsqos>` on |DataWriterQos| that controls the allocation
    behavior of writer-side filtering resources.
    Setting a maximum value of 0 disables filter evaluation on the writer side.
    A maximum value of 32 (the default value) means the writer will perform filter evaluation for up to 32 readers.
  - If the DataWriter is evaluating filters for ``writer_resource_limits.reader_filters_allocation.maximum``
    DataReaders, and a new filtered DataReader is created, then the filter for the newly created DataReader
    will be evaluated on the reader side.

.. _dds_layer_topic_contentFilteredTopic_writer_race_condition:

Discovery race condition
------------------------

On applications where the filter expression and/or the expression parameters are updated, there may be a
situation where the DataWriter will apply the old version of the filter until it receives updated information
through discovery.
This may imply that a publication made a short time after the DataReader updated the filter, but before the
updated discovery information is received by the DataWriter, may not be sent to the DataReader, even if the
new filter would have told otherwise.
Publications made after the updated discovery information is received will use the updated filter.

If some critical application considers this race condition issue unbearable, filtering on the writer side
can be disabled by setting the maximum value on |WriterResourceLimitsQos::reader_filters_allocation-api| to 0.
