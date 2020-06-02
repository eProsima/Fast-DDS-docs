.. _realtime-allocations:

Tuning allocations
==================

Allocating and deallocating memory implies some non-deterministic time consuming operations.
Therefore, most real-time systems need to operate in a way that all dynamic memory is allocated during the
application initialization, avoiding memory management operations in the main loop.

If users provide maximum sizes for the data and collections that Fast DDS keeps internally,
memory for these data and collections can be preallocated during entity initialization.
In order to choose the correct size values, users must be aware of the topology of the whole domain.
Specifically, the number of :ref:`DomainParticipants<dds_layer_domainParticipant>`,
:ref:`DataWriters<dds_layer_publisher_dataWriter>`, and :ref:`DataReaders<dds_layer_subscriber_dataReader>`
must be known when setting their configuration.

The following sections describe how to configure allocations to be done during the initialization of the
entities.
Although some examples are provided on each section as reference, there is also a
:ref:`complete example use case<tuning_allocations_full_example>`.

Parameters on the participant
-----------------------------

Every :ref:`dds_layer_domainParticipant` holds an internal collection with information about every local and remote
peer :ref:`DomainParticipants<dds_layer_domainParticipant>` that has been discovered.
This information includes, among other things:

* A nested collection with information of every :ref:`dds_layer_publisher_dataWriter` announced on the
  peer :ref:`dds_layer_domainParticipant`.
* A nested collection with information of every :ref:`dds_layer_subscriber_dataReader` announced on the
  peer :ref:`dds_layer_domainParticipant`.
* Custom data configured by the user on the peer :ref:`dds_layer_domainParticipant`, namely,
  :ref:`User Data<userdataqospolicy>`, :ref:`Partitions<partitionqospolicy>`,
  and :ref:`Properties<propertypolicyqos>`.

By default, these collections are fully dynamic, meaning that new memory is allocated when a new
:ref:`dds_layer_domainParticipant`, :ref:`dds_layer_publisher_dataWriter`, or :ref:`dds_layer_subscriber_dataReader`
is discovered.
Likewise, the mentioned custom configuration data parameters have an arbitrary size.
By default, the memory for these parameters is allocated when the peer :ref:`dds_layer_domainParticipant` announces
their value.

However, :ref:`dds_layer_domainParticipantQos` has a data member ``allocation``,
of type :ref:`participantresourcelimitsqos`, that allows configuring
maximum sizes for these collections and parameters, so that all the required memory can be preallocated during
the initialization of the :ref:`dds_layer_domainParticipant`.


Limiting the number of discovered entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:ref:`participantresourcelimitsqos` provides three data members to configure the allocation behavior of
discovered entities:

* ``participants`` configures the allocation of the collection of discovered
  :ref:`DomainParticipants<dds_layer_domainParticipant>`.
* ``writers`` configures the allocation of the collection of :ref:`DataWriters<dds_layer_publisher_dataWriter>`
  within each discovered :ref:`dds_layer_domainParticipant`.
* ``readers`` configures the allocation of the collection of :ref:`DataReaders<dds_layer_subscriber_dataReader>`
  within each discovered :ref:`dds_layer_domainParticipant`.

By default, a full dynamic behavior is used.
Using these members, however, it is easy to configure the collections to be preallocated during initialization,
setting them to a static maximum expected value, as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by these data members.

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-ALLOCATION-QOS-PARTICIPANTS    |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-ALLOCATION-QOS-PARTICIPANTS |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

.. warning::

   Configuring a collection as fixed in size effectively limits the number of peer entities
   that can be discovered.
   Once the configured limit is reached, any new entity will be ignored.
   In the given example, if a fourth peer :ref:`dds_layer_domainParticipant` appears, it will
   not be discovered, as the collection of discovered :ref:`DomainParticipants<dds_layer_domainParticipant>`
   is already full.


Limiting the size of custom parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``data_limits`` inside :ref:`participantresourcelimitsqos` provides three data members to configure the allocation
behavior of custom parameters:

* ``max_user_data`` limits the size of :ref:`User Data<userdataqospolicy>` to the given number of octets.
* ``max_partitions`` limits the size of :ref:`Partitions<partitionqospolicy>` to the given number of octets.
* ``max_properties`` limits the size of :ref:`Properties<propertypolicyqos>` to the given number of octets.

If these sizes are configured to something different than zero, enough memory will be allocated for them
for each participant and endpoint.
A value of zero implies no size limitation, and memory will be dynamically allocated as needed.
By default, a full dynamic behavior is used.

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-ALLOCATION-QOS-PARAMETERS      |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-ALLOCATION-QOS-PARAMETERS   |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

.. warning::

   If the data fields announced by the remote peer do not fit on the preallocated memory,
   an error will be triggered during the processing of the announcement message.
   This usually means that the discovery messages of a remote peer with too large data fields
   will be discarded, i.e., peers with too large data fields will not be discovered.


Parameters on the DataWriter
----------------------------

Every :ref:`dds_layer_publisher_dataWriter` holds an internal collection with information about every
:ref:`dds_layer_subscriber_dataReader` to which it matches.
By default, this collection is fully dynamic, meaning that new memory is allocated when a new
:ref:`dds_layer_subscriber_dataReader` is matched.

However, :ref:`dds_layer_publisher_dataWriterQos` has a data member ``writer_resource_limits``,
of type :ref:`writerresourcelimitsqos`, that allows configuring
the memory allocation behavior on the :ref:`dds_layer_publisher_dataWriter`.

:ref:`writerresourcelimitsqos` provides a data member ``matched_subscriber_allocation``
of type :ref:`resourcelimitedcontainerconfig` that allows configuring
the maximum expected size of the collection of matched :ref:`DataReaders<dds_layer_subscriber_dataReader>`,
so that it can be preallocated during the initialization of the :ref:`dds_layer_publisher_dataWriter`,
as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by this data member.


+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-ALLOCATION-QOS-WRITER          |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-ALLOCATION-QOS-WRITER       |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

.. warning::

   Configuring the collection of matched :ref:`DataReaders<dds_layer_subscriber_dataReader>` as fixed in size
   effectively limits the number of :ref:`DataReaders<dds_layer_subscriber_dataReader>` to be matched.
   Once the configured limit is reached, any new :ref:`dds_layer_subscriber_dataReader` will be ignored.
   In the given example, if a fourth (potentially matching) :ref:`dds_layer_subscriber_dataReader`
   appears, it will not be matched, as the collection is already full.


Parameters on the DataReader
----------------------------

Every :ref:`dds_layer_subscriber_dataReader` holds an internal collection with information about every
:ref:`readerresourcelimitsqos` to which it matches.
By default, this collection is fully dynamic, meaning that new memory is allocated when a new
:ref:`dds_layer_publisher_dataWriter` is matched.

However, :ref:`dds_layer_subscriber_dataReaderQos` has a data member ``reader_resource_limits``,
of type :ref:`readerresourcelimitsqos`, that allows configuring
the memory allocation behavior on the :ref:`dds_layer_subscriber_dataReader`.

:ref:`readerresourcelimitsqos` provides a data member ``matched_publisher_allocation``
of type :ref:`resourcelimitedcontainerconfig` that allows configuring
the maximum expected size of the collection of matched :ref:`DataWriters<dds_layer_publisher_dataWriter>`,
so that it can be preallocated during the initialization of the :ref:`dds_layer_subscriber_dataReader`,
as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by this data member.

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-ALLOCATION-QOS-READER          |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-ALLOCATION-QOS-READER       |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

.. warning::

   Configuring the collection of matched :ref:`DataWriters<dds_layer_publisher_dataWriter>` as fixed in size
   effectively limits the number of :ref:`DataWriters<dds_layer_publisher_dataWriter>` to be matched.
   Once the configured limit is reached, any new :ref:`dds_layer_publisher_dataWriter` will be ignored.
   In the given example, if a fourth (potentially matching) :ref:`dds_layer_publisher_dataWriter`
   appears, it will not be matched, as the collection is already full.

.. _tuning_allocations_full_example:

Full example
------------

Given a system with the following topology:

.. list-table:: **Allocation tuning example topology**
   :header-rows: 1
   :align: left

   * - Participant P1
     - Participant P2
     - Participant P3
   * - Topic 1 publisher
     - Topic 1 subscriber
     - Topic 2 subscriber
   * - Topic 1 subscriber
     -
     - Topic 2 publisher
   * - Topic 1 subscriber
     -
     - Topic 2 subscriber

* The total number of :ref:`DomainParticipants<dds_layer_domainParticipant>` is 3.
* The maximum number of :ref:`DataWriters<dds_layer_publisher_dataWriter>` per :ref:`dds_layer_domainParticipant` is 1
* The maximum number of :ref:`DataReaders<dds_layer_subscriber_dataReader>` per :ref:`dds_layer_domainParticipant` is 2.
* The :ref:`DataWriter<dds_layer_publisher_dataWriter>` for topic 1
  matches with 3 :ref:`DataReaders<dds_layer_subscriber_dataReader>`.
* The :ref:`DataWriter<dds_layer_publisher_dataWriter>` for topic 2
  matches with 2 :ref:`DataReaders<dds_layer_subscriber_dataReader>`.
* All the :ref:`DataReaders<dds_layer_subscriber_dataReader>`
  match exactly with 1 :ref:`DataWriter<dds_layer_publisher_dataWriter>`.

We will also limit the size of the parameters:

* Maximum :ref:`Partition Data<partitionqospolicy>` size: 256
* Maximum :ref:`User Data<userdataqospolicy>` size: 256
* Maximum :ref:`Properties Data <propertypolicyqos>` size: 512

The following piece of code shows the set of parameters needed for the use case depicted in this example.

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp      |
|    :language: c++                                   |
|    :start-after: //CONF-ALLOCATION-QOS-EXAMPLE      |
|    :end-before: //!--                               |
|    :dedent: 8                                       |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-ALLOCATION-QOS-EXAMPLE   |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+


