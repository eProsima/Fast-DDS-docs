.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _transport_zero_copy:

Zero-Copy
=========

This section explains the Zero-Copy transfer mode implemented in *Fast DDS*.
The Zero-Copy transfer allows the transmission of data between DDS |DataWriter| and |DataReader| entities without
making copies of the data being transmitted.
These samples are stored in a shared memory region, which is accessible by multiple processes implementing an
application using Fast DDS. In other words, the DataWriter has its History in a memory mapped file accessible to
the DataReader.

The figure below shows a comparison between the different transports available in *Fast DDS*.

.. figure:: /01-figures/fast_dds/transport/transport_comparison.png
    :align: center

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Overview
---------

When creating a DataWriter that supports Zero-Copy transfers, samples must be created with a *Fast DDS* function
that extends the DDS API (|DataWriter::loan_sample-api|).
The return of this function is a reference A* to the sample being sent, that is, a reference to the sample stored in the
memory mapped file.
The reference to this sample is sent to the DataReader which supports Zero-Copy and which is attached to the memory
mapped file.
Thus, the user has access to a reference B* to the sample.
Since both processes do not share process memory, i.e., each has an assigned virtual process memory,
the references A* and B* will be different but both point to the same RAM region.

It is worth mentioning that one of the main advantages of this transfer solution is to avoid encapsulation of
RTPS packets for DDS entities running on the same machine.

This feature requires the usage of new *Fast DDS* API which extends the standard DDS API.

* |DataWriter::loan_sample-api|
* |DataWriter::discard_loan-api|

Getting started
---------------

To enable Zero-Copy perform the following steps:

1.  Define a plain and bounded type in an IDL file and generate the corresponding source code for further processing
    with the |Fast DDS-Gen| tool.

    .. code-block:: idl

        struct LoanableHelloWorld
        {
            unsigned long index;
            char message[256];
        };

2.  On the DataWriter side:

    a)  Create a DataWriter for the previous type.
    b)  Get a loan on a sample using |DataWriter::loan_sample-api|.
    c)  Write the sample using |DataWriter::write-api|.

3.  On the DataReader side:

    a)  Create a DataReader for the previous type.
    b)  Take/read the sample using the available functions in the DataReader.
        Please refer to section :ref:`dds_layer_subscriber_accessreceived` for further detail on how to access
        received data.
    c)  Return the loaned sample using :func:`return_loan`.

Writing and reading in Zero-Copy transfers
------------------------------------------

The following is an example of how to publish and receive samples with DataWriters and DataReaders respectively
that implement Zero-Copy.

DataWriter
^^^^^^^^^^

When the DataWriter is created, *Fast DDS* will pre-allocate a pool of
|ResourceLimitsQosPolicy::max_samples-api| + |ResourceLimitsQosPolicy::extra_samples-api| samples that reside
in a shared memory mapped file.
This poll will be used to loan samples when the |DataWriter::loan_sample-api| function is called.

An application example of a DataWriter that supports Zero-Copy using the *Fast DDS* library is presented below.
There are several points to note in the following code:

*   Enabling the ``DataSharingQosPolicy``.
*   The use of the :func:`return_loan` function to access and modify data samples.
*   The writing of data samples.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_WRITER
   :end-before: //!


DataReader
^^^^^^^^^^

The following is an application example of a DataWriter that supports Zero-Copy using the *Fast DDS* library.
As shown in this code snippet, the only difference in the DataReader declaration is enabling the
|DataReaderQoS| ``DataSharingQosPolicy``.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_READER
   :end-before: //!

Finally, the code snippet below implements the |DataReaderListener::on_data_available-api| |DataReaderListener|
callback.
The key points to be noted in this function are:

*   The declaration and handling of ``LoanableSequences``.
*   The use of the :func:`return_loan` function to indicate to the DataReader that the application has finished
    accessing the sequence.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 12
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_LISTENER_READER
   :end-before: //!

Caveats
-------

*   After calling |DataWriter::loan_sample-api|, *Fast DDS* takes ownership of the sample and therefore it is no longer
    safe to make changes to that sample.
*   If function |DataWriter::loan_sample-api| is called first and the sample is never written, it is necessary to use
    function |DataWriter::discard_loan-api| to return the sample to the DataWriter.
    If this is not done, the subsequent calls to |DataWriter::loan_sample-api| may fail if DataWriter has no more
    |ResourceLimitsQosPolicy::extra_samples-api| to loan.
*   The current maximum supported sample size is the maximum value of an ``uint32_t``.

Constraints
-----------

Although Zero-Copy can be used for one or several *Fast DDS* application processes running on the same machine,
it has some constraints:

*   Complete support for plain types.
*   Unbounded types are not supported.
*   Suitable for |PREALLOCATED_MEMORY_MODE-api| memory configurations only.

.. note::
    Zero-Copy transfer support for non-plain types will be implemented in future releases of *Fast DDS*.

Next steps
----------

In the *eProsima Fast DDS* Github repository contains the complete example discussed in this section, as well as
multiple other examples for different use cases. The example implementing Zero-Copy transfers can be found
`here <https://github.com/eProsima/Fast-DDS/tree/feature/zero-copy-preview/examples/C%2B%2B/DDS/ZeroCopyExample>`_..
