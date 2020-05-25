.. _rtps_layer:

RTPS Layer
==========

The lower level RTPS Layer of *eprosima Fast DDS* serves an implementation of the protocol defined in the
`RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_.
This layer provides more control over the internals of the communication protocol than the :ref:`dds_layer`, so advanced
users have finer control over the library's functionalities.


Relation to the DDS Layer
-------------------------

Elements of this layer map one-to-one with elements from the :ref:`dds_layer`, with a few additions.
This correspondence is shown in the following table:

+----------------------------------------+-------------------+
| :ref:`dds_layer`                       | :ref:`rtps_layer` |
+========================================+===================+
| :ref:`dds_layer_domain`                | RTPSDomain        |
+----------------------------------------+-------------------+
| :ref:`dds_layer_domainParticipant`     | RTPSParticipant   |
+----------------------------------------+-------------------+
| :ref:`dds_layer_publisher_dataWriter`  | RTPSWriter        |
+----------------------------------------+-------------------+
| :ref:`dds_layer_subscriber_dataReader` | RTPSReader        |
+----------------------------------------+-------------------+

How to use the RTPS Layer
-------------------------

We will now go over the use of the RTPS Layer like we did with the :ref:`dds_layer` one,
explaining the new features it presents.

We recommend you to look at the two examples of how to use this layer the distribution comes with while reading
this section. They are located in `examples/RTPSTest_as_socket` and in `examples/RTPSTest_registered`

Managing the Participant
^^^^^^^^^^^^^^^^^^^^^^^^

Creating a :class:`RTPSParticipant` is done with :func:`RTPSDomain::createParticipant`.
:class:`RTPSParticipantAttributes` structure is used to configure the :class:`RTPSParticipant` upon creation.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_CREATE_PARTICIPANT
    :end-before: //!--
    :dedent: 4

Managing the Writers and Readers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the RTPS standard specifies, Writers and Readers are always associated with a History element.
In the :ref:`dds_layer`, its creation and management is hidden,
but in the :ref:`rtps_layer`, you have full control over its creation and configuration.

Writers are created with :func:`RTPSDomain::createRTPSWriter` and configured with a :class:`WriterAttributes` structure.
They also need a :class:`WriterHistory` which is configured with a :class:`HistoryAttributes` structure.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_HISTORY
    :end-before: //!--
    :dedent: 4

The creation of a Reader is similar to that of the Writers.
Note that in this case, you can provide a specialization of :class:`ReaderListener` class that implements your
callbacks:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_CONF_HISTORY
    :end-before: //!--
    :dedent: 8

Using the History to Send and Receive Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the RTPS Protocol, Readers and Writers save the data about a topic in their associated History.
Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as :class:`CacheChange_t`.
Changes are always managed by the History. As a user, the procedure for interacting with the History is always the same:

1. Request a :class:`CacheChange_t` from the History
2. Use it
3. Release it

You can interact with the History of the Writer to send data.
A callback that returns the maximum number of payload bytes is required:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITE_SAMPLE
    :end-before: //!--
    :dedent: 4

If your topic data type has several fields, you will have to provide functions to serialize and deserialize
your data in and out of the :class:`CacheChange_t`.
*Fast DDS-Gen* does this for you.

You can receive data from within a :class:`ReaderListener` callback method as we did in the :ref:`dds_layer`:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_LISTENER
    :end-before: //!--

Configuring Readers and Writers
-------------------------------
One of the benefits of using the :ref:`rtps_layer` is that it provides new configuration possibilities while
maintaining the options from the DDS layer.
For example, you can set a Writer or a Reader as a Reliable or Best-Effort endpoint as previously:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_RELIABILITY
    :end-before: //!--
    :dedent: 4

.. _SettingDataDurability:

Setting the data durability kind
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Durability parameter defines the behavior of the Writer regarding samples already sent when a new Reader matches.
*eProsima Fast DDS* offers three Durability options:

* VOLATILE (default): Messages are discarded as they are sent.
  If a new Reader matches after message *n*, it will start received from message *n+1*.
* TRANSIENT_LOCAL: The Writer saves a record of the last *k* messages it has sent.
  If a new reader matches after message *n*, it will start receiving from message *n-k*
* TRANSIENT: As TRANSIENT_LOCAL, but the record of messages will be saved to persistent storage, so it will be available
  if the writer is destroyed and recreated, or in case of an application crash (see :ref:`persistence`)

To choose your preferred option:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_DURABILITY
    :end-before: //!--
    :dedent: 4

Because in the :ref:`rtps_layer` you have control over the History, in TRANSIENT_LOCAL and TRANSIENT modes the Writer
sends all changes you have not explicitly released from the History.

Configuring the History
-----------------------

The History has its own configuration structure, the :class:`HistoryAttributes`.

Changing the maximum size of the payload
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can choose the maximum size of the Payload that can go into a :class:`CacheChange_t`.
Be sure to choose a size that allows it to hold the biggest possible piece of data:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_PAYLOADMAXSIZE
    :end-before: //!--
    :dedent: 4

Changing the size of the History
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can specify a maximum amount of changes for the History to hold and an initial amount of allocated changes:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_RESOURCES
    :end-before: //!--
    :dedent: 4

When the initial amount of reserved changes is lower than the maximum, the History will allocate more changes as they
are needed until it reaches the maximum size.

