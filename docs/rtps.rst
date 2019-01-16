Writer-Reader Layer
===================

The lower level Writer-Reader Layer of *eprosima Fast RTPS* provides a raw implementation of the RTPS protocol.
It provides more control over the internals of the protocol than the Publisher-Subscriber layer.
Advanced users can make use of this layer directly to gain more control over the functionality of the library.


Relation to the Publisher-Subscriber Layer
------------------------------------------

Elements of this layer map one-to-one with elements from the Publisher-Subscriber Layer, with a few additions.
The following table shows the name correspondence between layers:

        +----------------------------+---------------------+
        | Publisher-Subscriber Layer | Writer-Reader Layer |
        +============================+=====================+
        |          Domain            |     RTPSDomain      |
        +----------------------------+---------------------+
        |        Participant         |   RTPSParticipant   |
        +----------------------------+---------------------+
        |         Publisher          |     RTPSWriter      |
        +----------------------------+---------------------+
        |         Subscriber         |     RTPSReader      |
        +----------------------------+---------------------+

How to use the Writer-Reader Layer
----------------------------------

We will now go over the use of the Writer-Reader Layer like we did with the Publish-Subscriber one,
explaining the new features it presents.

We recommend you to look at the two examples of how to use this layer the distribution comes with while reading
this section. They are located in `examples/RTPSTest_as_socket` and in `examples/RTPSTest_registered`

Managing the Participant
^^^^^^^^^^^^^^^^^^^^^^^^

To create a :class:`RTPSParticipant`, the process is very similar to the one shown in the Publisher-Subscriber layer.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_CREATE_PARTICIPANT
    :end-before: //!--

The :class:`RTPSParticipantAttributes` structure is equivalent to the `rtps` member of :class:`ParticipantAttributes`
field in the Publisher-Subscriber Layer, so you can configure your :class:`RTPSParticipant` the same way as before:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_CONF_PARTICIPANT
    :end-before: //!--

Managing the Writers and Readers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the RTPS standard specifies, Writers and Readers are always associated with a History element.
In the Publisher-Subscriber Layer, its creation and management is hidden,
but in the Writer-Reader Layer, you have full control over its creation and configuration.

Writers are configured with a :class:`WriterAttributes` structure.
They also need a :class:`WriterHistory` which is configured with a :class:`HistoryAttributes` structure.

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_HISTORY
    :end-before: //!--

The creation of a Reader is similar.
Note that in this case, you can provide a :class:`ReaderListener` instance that implements your callbacks:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_CONF_HISTORY
    :end-before: //!--

Using the History to Send and Receive Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the RTPS Protocol, Readers and Writers save the data about a topic in their associated History.
Each piece of data is represented by a Change, which *eprosima Fast RTPS* implements as :class:`CacheChange_t`.
Changes are always managed by the History. As a user, the procedure for interacting with the History is always the same:

1. Request a :class:`CacheChange_t` from the History
2. Use it
3. Release it

You can interact with the History of the Writer to send data.
A callback that returns the maximum number of payload bytes is required:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITE_SAMPLE
    :end-before: //!--

If your topic data type has several fields, you will have to provide functions to serialize and deserialize
your data in and out of the :class:`CacheChange_t`.
*FastRTPSGen* does this for you.

You can receive data from within a :class:`ReaderListener` callback method as we did in the Publisher-Subscriber Layer:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_LISTENER
    :end-before: //!--

Configuring Readers and Writers
-------------------------------
One of the benefits of using the Writer-Reader layer is that it provides new configuration possibilities while
maintaining the options from the Publisher-Subscriber layer (see :ref:`configuration`).
For example, you can set a Writer or a Reader as a Reliable or Best-Effort endpoint as previously:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_RELIABILITY
    :end-before: //!--

.. _SettingDataDurability:

Setting the data durability kind
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Durability parameter defines the behavior of the Writer regarding samples already sent when a new Reader matches.
*eProsima Fast RTPS* offers three Durability options:

* VOLATILE (default): Messages are discarded as they are sent.
  If a new Reader matches after message *n*, it will start received from message *n+1*.
* TRANSIENT_LOCAL: The Writer saves a record of the last *k* messages it has sent.
  If a new reader matches after message *n*, it will start receiving from message *n-k*
* TRANSIENT: As TRANSIENT_LOCAL, but the record of messages will be saved to persistent storage, so it will be available
  if the writer is destroyed and recreated, or in case of an application crash (see :ref:`persistence`)

To choose your preferred option:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_DURABILITY
    :end-before: //!--

Because in the Writer-Reader layer you have control over the History, in TRANSIENT_LOCAL and TRANSIENT modes the Writer
sends all changes you have not explicitly released from the History.

Configuring the History
-----------------------

The History has its own configuration structure, the :class:`HistoryAttributes`.

Changing the maximum size of the payload
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can choose the maximum size of the Payload that can go into a :class:`CacheChange_t`.
Be sure to choose a size that allows it to hold the biggest possible piece of data:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_PAYLOADMAXSIZE
    :end-before: //!--

Changing the size of the History
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can specify a maximum amount of changes for the History to hold and an initial amount of allocated changes:

.. literalinclude:: ../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_HISTORY_CONF_RESOURCES
    :end-before: //!--

When the initial amount of reserved changes is lower than the maximum, the History will allocate more changes as they
are needed until it reaches the maximum size.
