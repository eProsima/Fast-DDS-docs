.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_write:

Publishing data
===============

The user informs of a change in the value of a data instance with the |DataWriter::write-api| member function on the
:ref:`dds_layer_publisher_dataWriter`. This change will then be communicated to every
:ref:`dds_layer_subscriber_dataReader` matched with the DataWriter.
As a side effect, this operation asserts liveliness on the DataWriter itself,
the :ref:`dds_layer_publisher_publisher` and the :ref:`dds_layer_domainParticipant`.

The function takes two arguments:

 * A pointer to the data instance with the new values.
 * The handler to the instance.

An empty (i.e., default constructed |InstanceHandle_t-api|) instance handler can be used for the argument handle.
This indicates that the identity of the instance should be automatically deduced from the key of the
instance data.
Alternatively, the member function |DataWriter::write-api| is overloaded to take only the pointer to the data instance,
which will always deduced the identity from the key of the instance data.

If the handle is not empty, then it must correspond to the value obtained with the |TopicDataType::getKey-api| of the
|TypeSupport-api| instance.
Otherwise the write function will fail with ``RETCODE_PRECONDITION_NOT_MET``.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_WRITE
   :end-before: //!
   :dedent: 8


.. _dds_layer_publisher_write_blocking:

Blocking of the write operation
-------------------------------

If the reliability kind is set to ``RELIABLE`` on the :ref:`dds_layer_publisher_dataWriterQos`,
the |DataWriter::write-api| operation may block.
Specifically, if the limits specified in the configured resource limits have been reached, the
|DataWriter::write-api| operation will block waiting for space to become available.
Under these circumstances, the reliability ``max_blocking_time`` configures the maximum time
the write operation may block waiting.
If ``max_blocking_time`` elapses before the DataWriter is able to store
the modification without exceeding the limits, the write operation will fail and return ``TIMEOUT``.


.. _dds_layer_publisher_write_loans:

Borrowing a data buffer
-----------------------

When the user calls |DataWriter::write-api| with a new sample value,
the data is copied from the given sample to the DataWriter's memory.
For large data types this copy can consume significant time and memory resources.
Instead, the DataWriter can loan a sample from its memory to the user,
and the user can fill this sample with the required values.
When |DataWriter::write-api| is called with such a loaned sample,
the DataWriter does not copy its contents, as it already owns the buffer.

To use loaned data samples in publications, perform the following steps:

1. Get a reference to a loaned sample using |DataWriter::loan_sample-api|.
2. Use the reference to build the data sample.
3. Write the sample using |DataWriter::write-api|.


Once |DataWriter::write-api| has been called with a loaned sample,
the loan is considered returned, and it is not safe to make any
changes on the contents of the sample.

If function |DataWriter::loan_sample-api| is called but the sample is never written,
the loan must be returned to the DataWriter using |DataWriter::discard_loan-api|.
Otherwise the DataWriter may run out of samples.


.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_LOAN_SAMPLES
   :end-before: //!
   :dedent: 8
