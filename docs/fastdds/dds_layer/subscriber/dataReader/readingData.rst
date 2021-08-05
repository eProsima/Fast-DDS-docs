.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_accessreceived:

Accessing received data
=======================

The application can access and consume the data values received on the :ref:`dds_layer_subscriber_dataReader`
by *reading* or *taking*.

 * **Reading** is done with any of the following member functions:

   * |DataReader::read_next_sample-api| reads the next, non-previously accessed data value available
     on the DataReader, and stores it in the provided data buffer.
   * |DataReader::read-api|, |DataReader::read_instance-api|, and |DataReader::read_next_instance-api|
     provide mechanisms to get a collection of samples matching certain conditions.

 * **Taking** is done with any of the following member functions:

   * |DataReader::take_next_sample-api| reads the next, non-previously accessed data value available on the DataReader,
     and stores it in the provided data buffer.
   * |DataReader::take-api|, |DataReader::take_instance-api|, and |DataReader::take_next_instance-api|
     provide mechanisms to get a collection of samples matching certain conditions.

   When taking data, the returned samples are also removed from the DataReader, so they are no longer accessible.

When there is no data in the DataReader matching the required conditions, all the operations will return
``NO_DATA`` and output parameter will remain unchanged.

In addition to the data values, the data access operations also provide SampleInfo instances with additional
information that help interpreting the returned data values, like the originating
:ref:`dds_layer_publisher_dataWriter` or the publication time stamp.
Please, refer to the :ref:`dds_layer_subscriber_sampleInfo` section for an extensive description of its contents.

.. _dds_layer_subscriber_accessreceived_loans:

Loaning and Returning Data and SampleInfo Sequences
---------------------------------------------------

The |DataReader::read-api| and |DataReader::take-api| operations (and their variants) return information to the
application in two sequences:

 * Received DDS data samples in a sequence of the data type
 * Corresponding information about each DDS sample in a SampleInfo sequence

These sequences are parameters that are passed by the application code into the
|DataReader::read-api| and |DataReader::take-api| operations.
When the passed sequences are empty (they are initialized but have a maximum length of 0), the middleware will
fill those sequences with memory directly loaned from the receive queue itself.
There is no copying of the data or SampleInfo when the contents of the sequences are loaned.
This is certainly the most efficient way for the application code to retrieve the data.

When doing so, however, the code must return the loaned sequences back to the middleware, so that they can be reused
by the receive queue.
If the application does not return the loan by calling the |DataReader::return_loan-api| operation, then Fast DDS
will eventually run out of memory to store DDS data samples received from the network for that DataReader.
See the code below for an example of borrowing and returning loaned sequences.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_LOAN_SEQUENCES
   :end-before: //!
   :dedent: 8

.. _dds_layer_subscriber_accessreceived_data:

Processing returned data
------------------------

After calling the |DataReader::read-api| or |DataReader::take-api| operations, accessing the data on the returned
sequences is quite easy.
The sequences API provides a **length()** operation returning the number of elements in the collections.
The application code just needs to check this value and use the **[]** operator to access the corresponding elements.
Elements on the DDS data sequence should only be accessed when the corresponding element on the SampleInfo sequence
indicate that valid data is present.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_PROCESS_DATA
   :end-before: //!
   :dedent: 8

.. _dds_layer_subscriber_accessreceived_listener:

Accessing data on callbacks
---------------------------

When the DataReader receives new data values from any matching DataWriter, it informs the application through
two Listener callbacks:

* |DataReaderListener::on_data_available-api|.

* |SubscriberListener::on_data_on_readers-api|.

These callbacks can be used to retrieve the newly arrived data, as in the following example.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_READ_LISTENER
   :end-before: //!

.. note::
   If several new data changes are received at once, the callbacks may be triggered just once,
   instead of once per change.
   The application must keep *reading* or *taking* until no new changes are available.


.. _dds_layer_subscriber_accessreceived_wait:

Accessing data with a waiting thread
------------------------------------

Instead of relying on the Listener to try and get new data values,
the application can also dedicate a thread to wait until any new data is available on the
DataReader.
This can be done using a waitset to wait for a change on the `DataAvailable` status.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_READ_WAITSET
   :end-before: //!
   :dedent: 8

The same could be achieved using the :func:`wait_for_unread_message` member function,
that blocks until a new data sample is available or the given timeout expires.
If no new data was available after the timeout expired, it will return with value ``false``.
This function returning with value ``true`` means there is new data available on the
:ref:`dds_layer_subscriber_dataReader` ready for the application to retrieve.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAREADER_READ_WAIT
   :end-before: //!
   :dedent: 8
