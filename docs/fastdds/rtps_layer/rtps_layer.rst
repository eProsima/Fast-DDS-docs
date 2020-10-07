.. include:: ../../03-exports/aliases-api.include

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

We recommend you to look at the two examples describing how to use the RTPS layer that come with the distribution
while reading this section.
They are located in `examples/RTPSTest_as_socket` and in `examples/RTPSTest_registered`

Managing the Participant
^^^^^^^^^^^^^^^^^^^^^^^^

Creating a |RTPSParticipant-api| is done with |RTPSDomain::createParticipant-api|.
|RTPSParticipantAttributes-api| structure is used to configure the :class:`RTPSParticipant` upon creation.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_CREATE_PARTICIPANT
    :end-before: //!--
    :dedent: 4

Managing the Writers and Readers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the RTPS standard specifies, |RTPSWriters-api| and |RTPSReaders-api| are always associated with a |History-api| element.
In the :ref:`dds_layer`, its creation and management is hidden,
but in the :ref:`rtps_layer`, you have full control over its creation and configuration.

Writers are created with |RTPSDomain::createRTPSWriter-api| and configured with a |WriterAttributes-api| structure.
They also need a |WriterHistory-api| which is configured with a |HistoryAttributes-api| structure.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITER_CONF_HISTORY
    :end-before: //!--
    :dedent: 4

Similar to the creation of Writers, Readers are created with |RTPSDomain::createRTPSReader-api|
and configured with a |ReaderAttributes-api| structure.
A |HistoryAttributes-api| structure is used to configure the required |ReaderHistory-api|.
Note that in this case, you can provide a specialization of |ReaderListener-api| class that implements your
callbacks:

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_READER_CONF_HISTORY
    :end-before: //!--
    :dedent: 8

Using the History to Send and Receive Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the RTPS Protocol, Readers and Writers save the data about a topic in their associated History.
Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as |CacheChange_t-api|.
Changes are always managed by the History.

You can add a new :class:`CacheChange_t` to the History of the Writer to send data.
The procedure is as follows:

1. Request a :class:`CacheChange_t` from the Writer with |RTPSWriters::new_change-api|.
   In order to allocate enough memory,
   you need to provide a callback that returns the maximum number bytes in the payload.
2. Fill the :class:`CacheChange_t` with the data.
3. Add it to the History with |WriterHistory::add_change-api|.

The Writer will take care of everything to communicate the data to the Readers.

.. literalinclude:: ../../../code/CodeTester.cpp
    :language: c++
    :start-after: //RTPS_API_WRITE_SAMPLE
    :end-before: //!--
    :dedent: 4

If your topic data type has several fields, you will have to provide functions to serialize and deserialize
your data in and out of the :class:`CacheChange_t`.
*Fast DDS-Gen* does this for you.

You can receive data from within the |ReaderListener::onNewCacheChangeAdded-api| callback,
as we did in the :ref:`dds_layer`:

1. The callback receives a :class:`CacheChange_t` parameter containing the received data.
2. Process the data within the received :class:`CacheChange_t`.
3. Inform the Reader's History that the change is not needed anymore.


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
  if the writer is destroyed and recreated, or in case of an application crash.

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

The History has its own configuration structure, the |HistoryAttributes-api|.

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

