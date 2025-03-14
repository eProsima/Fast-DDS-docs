.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _realtime-allocations:

Tuning allocations
==================

Allocating and deallocating memory implies some non-deterministic time consuming operations.
Therefore, most real-time systems need to operate in a way that all dynamic memory is allocated during the
application initialization, avoiding memory management operations in the main loop.

If users provide maximum sizes for the data and collections that *Fast DDS* keeps internally,
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

Every DomainParticipant holds an internal collection with information about every local and remote
peer DomainParticipants that has been discovered.
This information includes, among other things:

* A nested collection with information of every DataWriter announced on the
  peer DomainParticipant.
* A nested collection with information of every DataReader announced on the
  peer DomainParticipant.
* Custom data configured by the user on the peer DomainParticipant, namely,
  :ref:`userdataqospolicy`, :ref:`partitionqospolicy`,
  and :ref:`propertypolicyqos`.

By default, these collections are fully dynamic, meaning that new memory is allocated when a new
DomainParticipant, DataWriter, or DataReader
is discovered.
Likewise, the mentioned custom configuration data parameters have an arbitrary size.
By default, the memory for these parameters is allocated when the peer DomainParticipant announces
their value.

However, :ref:`dds_layer_domainParticipantQos` has a member function |DomainParticipantQos::allocation-api|,
of type :ref:`participantresourcelimitsqos`, that allows configuring
maximum sizes for these collections and parameters, so that all the required memory can be preallocated during
the initialization of the DomainParticipant.


Limiting the number of discovered entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:ref:`participantresourcelimitsqos` provides three data members to configure the allocation behavior of
discovered entities:

* |ParticipantResourceLimitsQos::participants-api| configures the allocation of the collection of discovered
  DomainParticipants.
* |ParticipantResourceLimitsQos::readers-api| configures the allocation of the collection of DataWriters
  within each discovered DomainParticipant.
* |ParticipantResourceLimitsQos::writers-api| configures the allocation of the collection of DataReaders
  within each discovered DomainParticipant.

By default, a full dynamic behavior is used.
Using these members, however, it is easy to configure the collections to be preallocated during initialization,
setting them to a static maximum expected value, as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by these data members.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-ALLOCATION-QOS-PARTICIPANTS
        :end-before: //!--
        :dedent: 8
   
    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-ALLOCATION-QOS-PARTICIPANTS
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

.. warning::

   Configuring a collection as fixed in size effectively limits the number of peer entities
   that can be discovered.
   Once the configured limit is reached, any new entity will be ignored.
   In the given example, if a fourth peer DomainParticipant appears, it will
   not be discovered, as the collection of discovered DomainParticipants
   is already full.


Limiting the size of custom parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

|ParticipantResourceLimitsQos::data_limits-api| inside :ref:`participantresourcelimitsqos` provides three data members
to configure the allocation behavior of custom parameters:

* |VariableLengthDataLimits::max_user_data-api| limits the size of :ref:`userdataqospolicy` to the given number
  of octets.
* |VariableLengthDataLimits::max_properties-api| limits the size of :ref:`partitionqospolicy` to the given number
  of octets.
* |VariableLengthDataLimits::max_partitions-api| limits the size of :ref:`propertypolicyqos` to the given number
  of octets.

If these sizes are configured to something different than zero, enough memory will be allocated for them
for each participant and endpoint.
A value of zero implies no size limitation, and memory will be dynamically allocated as needed.
By default, a full dynamic behavior is used.

|ParticipantResourceLimitsQos::content_filter-api| inside :ref:`participantresourcelimitsqos` provides members
to configure the allocation behavior of content filter discovery information:

* |ContentFilterProperty::AllocationConfiguration::expression_initial_size-api| sets the preallocated size of
  the filter expression.
* |ContentFilterProperty::AllocationConfiguration::expression_parameters-api| controls the allocation behavior
  for the list of expression parameters.
  Refer to :ref:`resourcelimitedcontainerconfig` for a complete description of the alternatives.
  Receiving information about a content filter with more parameters than the maximum configured here, will make
  the filtering happen on the reader side.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF-ALLOCATION-QOS-PARAMETERS
       :end-before: //!--
       :dedent: 8
   
    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF-ALLOCATION-QOS-PARAMETERS
       :end-before: <!--><-->
       :lines: 2-3,5-
       :append: </profiles>

.. warning::

   If the data fields announced by the remote peer do not fit on the preallocated memory,
   an error will be triggered during the processing of the announcement message.
   This usually means that the discovery messages of a remote peer with too large data fields
   will be discarded, i.e., peers with too large data fields will not be discovered.


Parameters on the DataWriter
----------------------------

Every DataWriter holds internal collections with information about every
DataReader to which it matches.
By default, these collections are fully dynamic, meaning that new memory is allocated when a new
DataReader is matched.
However, :ref:`dds_layer_publisher_dataWriterQos` has a data member |DataWriterQos::writer_resource_limits-api|,
of type :ref:`writerresourcelimitsqos`, that allows configuring
the memory allocation behavior on the DataWriter.

:ref:`writerresourcelimitsqos` provides data members |WriterResourceLimitsQos::matched_subscriber_allocation-api|
and |WriterResourceLimitsQos::reader_filters_allocation-api|
of type :ref:`resourcelimitedcontainerconfig` that allow configuring
the maximum expected size of the collection of matched DataReader,
and the collection of writer side content filters,
so they can be preallocated during the initialization of the DataWriter,
as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by these data members.


.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-ALLOCATION-QOS-WRITER
        :end-before: //!--
        :dedent: 8
   
    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-ALLOCATION-QOS-WRITER
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

.. warning::

   Configuring the collection of matched DataReaders as fixed in size
   effectively limits the number of DataReaders to be matched.
   Once the configured limit is reached, any new DataReader will be ignored.
   In the given example, if a fourth (potentially matching) DataReader
   appears, it will not be matched, as the collection is already full.


Parameters on the DataReader
----------------------------

Every DataReader holds an internal collection with information about every
:ref:`readerresourcelimitsqos` to which it matches.
By default, this collection is fully dynamic, meaning that new memory is allocated when a new
DataWriter is matched.
However, :ref:`dds_layer_subscriber_dataReaderQos` has a data member |DataReaderQos::reader_resource_limits-api|,
of type :ref:`readerresourcelimitsqos`, that allows configuring
the memory allocation behavior on the DataReader.

:ref:`readerresourcelimitsqos` provides a data member |ReaderResourceLimitsQos::matched_publisher_allocation-api|
of type :ref:`resourcelimitedcontainerconfig` that allows configuring
the maximum expected size of the collection of matched DataWriters,
so that it can be preallocated during the initialization of the DataReader,
as shown in the example below.
Please, refer to :ref:`resourcelimitedcontainerconfig` for a complete description of additional configuration
alternatives given by this data member.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-ALLOCATION-QOS-READER
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-ALLOCATION-QOS-READER
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

.. warning::

   Configuring the collection of matched DataWriters as fixed in size
   effectively limits the number of DataWriters to be matched.
   Once the configured limit is reached, any new DataWriter will be ignored.
   In the given example, if a fourth (potentially matching) DataWriter
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

* The total number of DomainParticipants is 3.
* The maximum number of DataWriters per DomainParticipant is 1
* The maximum number of DataReaders per DomainParticipant is 2.
* The DataWriter for topic 1
  matches with 3 DataReaders.
* The DataWriter for topic 2
  matches with 2 DataReaders.
* All the DataReaders
  match exactly with 1 DataWriter.

We will assume that content filtering is not being used, and will also limit the size of the parameters:

* Maximum :ref:`partitionqospolicy` size: 256
* Maximum :ref:`userdataqospolicy` size: 256
* Maximum :ref:`propertypolicyqos` size: 512

The following piece of code shows the set of parameters needed for the use case depicted in this example.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF-ALLOCATION-QOS-EXAMPLE
       :end-before: //!--
       :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF-ALLOCATION-QOS-EXAMPLE
       :end-before: <!--><-->
       :lines: 2-3,5-
       :append: </profiles>


