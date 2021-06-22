.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _use-case-zero-copy:

Zero-Copy communication
=======================

This section explains how to configure a Zero-Copy communication in *Fast DDS*.
The Zero-Copy communication allows the transmission of data between applications
without copying data in memory, saving time and resources.
In order to achieve this, it uses Data-sharing delivery between the |DataWriter|
and the |DataReader|, and data buffer loans between the application and *Fast DDS*.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Overview
---------

:ref:`datasharing-delivery` provides a communication channel between a DataWriter and a DataReader
using shared memory. Therefore, it does not require copying the sample data to transmit it.

:ref:`DataWriter sample loaning<dds_layer_publisher_write_loans>`
is a *Fast DDS* extension that allows the application to borrow
a buffer for a sample in the publishing DataWriter.
The sample can be constructed directly on this buffer,
eliminating the need to copy it to the DataWriter afterwards.
This prevents the copying of the data between the publishing application and the DataWriter.
If Data-sharing delivery is used, the loaned data buffer will be in the shared memory itself.

Reading the data on the subscriber side can also be done
with :ref:`loans from the DataReader<dds_layer_subscriber_accessreceived_loans>`.
The application gets the received samples as a reference to the receive queue itself.
This prevents the copying of the data from the DataReader to the receiving application.
Again, if Data-sharing delivery is used, the loaned data will be in the shared memory,
and will indeed be the same memory buffer used in the DataWriter history.

Combining these three features, we can achieve Zero-Copy communication between the
publishing application and the subscribing application.


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

    a)  Create a DataWriter for the previous type. Make sure that the DataWriter does not have DataSharing disabled.
    b)  Get a loan on a sample using |DataWriter::loan_sample-api|.
    c)  Write the sample using |DataWriter::write-api|.

3.  On the DataReader side:

    a)  Create a DataReader for the previous type. Make sure that the DataReader does not have DataSharing disabled.
    b)  Take/read samples using the available functions in the DataReader.
        Please refer to section :ref:`dds_layer_subscriber_accessreceived_loans` for further detail on how to access
        to loans of the received data.
    c)  Return the loaned samples using |DataReader::return_loan-api|.

Writing and reading in Zero-Copy transfers
------------------------------------------

The following is an example of how to publish and receive samples with DataWriters and DataReaders respectively
that implement Zero-Copy.

DataWriter
^^^^^^^^^^

When the DataWriter is created, *Fast DDS* will pre-allocate a pool of
|ResourceLimitsQosPolicy::max_samples-api| + |ResourceLimitsQosPolicy::extra_samples-api| samples that reside
in a shared memory mapped file.
This pool will be used to loan samples when the |DataWriter::loan_sample-api| function is called.

An application example of a DataWriter that supports Zero-Copy using the *Fast DDS* library is presented below.
There are several points to note in the following code:

*   Not disabling the :ref:`datasharingqospolicy`.
    |DATASHARING_AUTO-api| kind automatically enables Zero-Copy when possible.
*   The use of the |DataWriter::loan_sample-api| function to access and modify data samples.
*   The writing of data samples.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_WRITER
   :end-before: //!


DataReader
^^^^^^^^^^

The following is an application example of a DataReader that supports Zero-Copy using the *Fast DDS* library.
As shown in this code snippet, the configuration in the DataReader is similar to the DataWriter.
Be sure not to disable the :ref:`datasharingqospolicy`.
|DATASHARING_AUTO-api| kind automatically enables Zero-Copy when possible.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_READER
   :end-before: //!

Finally, the code snippet below implements the |DataReaderListener::on_data_available-api| |DataReaderListener|
callback.
The key points to be noted in this function are:

*   The declaration and handling of |LoanableSequence-api|.
*   The use of the |DataReader::return_loan-api| function to indicate to the DataReader that the application has
    finished accessing the sequence.

.. literalinclude:: ../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 12
   :start-after: //LOANABLE_HELLOWORLD_EXAMPLE_LISTENER_READER
   :end-before: //!

Caveats
-------

*   After calling |DataWriter::write-api|, *Fast DDS* takes ownership of the sample and therefore it is no longer
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

*   Only plain types are supported.
*   Constraints for :ref:`datasharing delivery<datasharing-delivery-constraints>` also apply.

.. note::
    Zero-Copy transfer support for non-plain types may be implemented in future releases of *Fast DDS*.

Next steps
----------

The *eProsima Fast DDS* Github repository contains the complete example discussed in this section, as well as
multiple other examples for different use cases. The example implementing Zero-Copy transfers can be found
`here <https://github.com/eProsima/Fast-DDS/tree/master/examples/C%2B%2B/DDS/ZeroCopyExample>`_.
