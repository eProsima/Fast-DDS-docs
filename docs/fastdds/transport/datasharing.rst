.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _datasharing-delivery:

Data-sharing delivery
=====================

*Fast DDS* allows to speed up communications between entities within the same machine
by sharing the history of the |DataWriter| with the |DataReader| through shared memory.
This prevents any of the overhead involved in the transport layer,
effectively avoiding any data copy between DataWriter and DataReader.

.. note::

    Fast DDS utilizes the |DomainParticipant|'s |GuidPrefix_t-api| to identify peers running in the same host.
    Two participants with identical 4 first bytes on the |GuidPrefix_t-api| are considered to be running in the same
    host.
    |is_on_same_host_as-api| API is provided to check this condition.
    Please, take also into account the caveats included in :ref:`intraprocess_delivery_guids`.

Use of Data-sharing delivery does not prevent data copies between the application
and the DataReader and DataWriter.
These can be avoided in some cases using :ref:`use-case-zero-copy`.

.. note::
    Although Data-sharing delivery uses shared memory,
    it differs from :ref:`transport_sharedMemory_sharedMemory`
    in that Shared Memory is a full-compliant transport.
    That means that with Shared Memory Transport
    the data being transmitted must be copied from the DataWriter history to the transport
    and from the transport to the DataReader.
    With Data-sharing these copies can be avoided.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Overview
---------

When the DataWriter is created, *Fast DDS* will pre-allocate a pool of
|ResourceLimitsQosPolicy::max_samples-api| + |ResourceLimitsQosPolicy::extra_samples-api| samples that reside
in a shared memory mapped file.
When publishing new data, the DataWriter will take a sample from this pool and add it to its history,
and notify the DataReader which sample from the pool has the new data.

The DataReader will have access to the same shared memory mapped file,
and will be able to access the data published by the DataWriter.

.. _datasharing-delivery-constraints:

Constraints
-----------

This feature is available only if the following requirements are met:

* The |DataWriter| and |DataReader| have access to the same shared memory.
* The |Topic| has a bounded |TopicDataType|,
  i.e., its |TopicDataType::is_bounded-api| member function returns true.
* The Topic :ref:`is not keyed<dds_layer_topic_keyed_data_types>`.
* The DataWriter is configured with |PREALLOCATED_MEMORY_MODE-api| or |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|.
* No :ref:`security <security>` plugins are used.

There is also a limitation with the DataReader's HistoryQos.
Using Data-sharing mechanism, the DataWriter's history is shared with the DataReaders.
This means that the effective HistoryQos depth on the DataReader is, at most,
the Datawriter's HistoryQos depth.
To avoid confusions, set the DataReaders' history depth to a value equal or less than the DataWriter's.

Data-sharing delivery configuration
-----------------------------------

Data-sharing delivery can be configured in the |DataWriter| and the |DataReader|
using :ref:`datasharingqospolicy`.
Four attributes can be configured:

* The data-sharing delivery kind
* The shared memory directory
* The data-sharing domain identifiers.
* The maximum number of data-sharing domain identifiers.

Data-Sharing delivery kind
""""""""""""""""""""""""""

Can be set to one of three modes:

* **AUTO**: If both a DataWriter and DataReader meet the requirements,
  data-sharing delivery will be used between them.
  This is the default value.
* **ON**: Like **AUTO**, but the creation of the entity will fail if the requirements are not met.
* **OFF**: No data-sharing delivery will be used on this entity.

The following matrix shows when two entities are data-sharing compatible according to their configuration
(given that the entity creation does not fail and that both entities have access to a shared memory):

.. |common_ids| replace:: Only if they have common domain IDs
.. |bounded_and_common_ids| replace:: Only if the TopicDataType is bounded :raw-html:`<br />`
   and they have common domain IDs

+------------+----------+------------------+-------------------+--------------------------+
|                       |                       **Reader**                                |
+                       +------------------+-------------------+--------------------------+
|                       | **ON**           | **OFF**           | **AUTO**                 |
+------------+----------+------------------+-------------------+--------------------------+
| **Writer** | **ON**   | |common_ids|     | No                | |common_ids|             |
+            +----------+------------------+-------------------+--------------------------+
|            | **OFF**  | No               | No                | No                       |
+            +----------+------------------+-------------------+--------------------------+
|            | **AUTO** | |common_ids|     | No                | |bounded_and_common_ids| |
+------------+----------+------------------+-------------------+--------------------------+


Data-sharing domain identifiers
"""""""""""""""""""""""""""""""

Each entity defines a set of identifiers that represent a *domain* to which the entity belongs.
Two entities will be able to use data-sharing delivery between them only if both have at least a common domain.

Users can define the domains of a |DataWriter| or |DataReader| with the :ref:`datasharingqospolicy`.
If no domain identifier is provided by the user, the system will create one automatically.
This automatic data-sharing domain will be unique for the machine where the entity is running.
That is, all entities running on the same machine, and for which the user has configured no user-specific domains,
will be able to use data-sharing delivery (given that the rest of requirements are met).

During the discovery phase, entities will exchange their domain identifiers and check if they can
use Data-sharing to communicate.

.. note::
    Even though a data-sharing domain identifier is a 64 bit integer,
    user-defined identifiers are restricted to 16 bit integers.


Maximum number of Data-sharing domain identifiers
"""""""""""""""""""""""""""""""""""""""""""""""""

The maximum number of domain identifiers that are expected to be received
from a remote entity during discovery.
If the remote entity defines (and sends) more than this number of domain identifiers,
the discovery will fail.

By default there is no limit to the number of identifiers.
The default value can be changed with the |DataSharingQosPolicy::max_domains-api| function.
Defining a finite number allows to preallocate the required memory
to receive the list of identifiers during the entity creation,
avoiding dynamic memory allocations afterwards.
Note that a value of ``0`` means no limit.


Shared memory directory
"""""""""""""""""""""""

If a user-defined directory is given for the shared memory files,
this directory will be used for the memory-mapped files used for data-sharing delivery.
If none is given, the default directory configured for the current system is used.

Configuring a user-defined directory may be useful in some scenarios:

* To select a file system with Huge TLB enabled for the memory-mapped files.
* To allow data-sharing delivery between containers that mount the same container.

.. _datareader-datawriter-history-coupling:

DataReader and DataWriter history coupling
------------------------------------------

With traditional :ref:`comm-transports-configuration` delivery,
the DataReader and DataWriter keep separate and independent histories,
each one with their own copy of the sample.
Once the sample is sent through the transport and received by the DataReader,
the DataWriter is free to remove the sample from its history
without affecting the DataReader.

With data-sharing delivery,
the DataReader directly accesses the data instance created by the DataWriter.
This means that the samples in both the history of the DataReader and the DataWriter
refer to the same object in the shared memory.
Therefore, there is a strong coupling in the behavior of the DataReader and DataWriter histories.

.. important::
    If the DataWriter reuses the same sample to publish new data,
    the DataReader loses access to the old data sample.

.. note::
    The DataWriter can remove the sample from its history,
    and it will still be available on the DataReader,
    unless the same sample from the pool is reused to publish a new one.


Data acknowledgement
""""""""""""""""""""

With data-sharing delivery, sample acknowledgment from the DataReader occurs the first time
a sample is retrieved by the application (using |DataReader::read_next_sample-api|,
|DataReader::take_next_sample-api|, or any of their variations).
Once the data has been accessed by the application,
the DataWriter is free to reuse that sample to publish new data.
The DataReader detects when a sample has been reused
and automatically removes it from its history.

This means that subsequent attempts to access the same sample
from the DataReader may return no sample at all.


Blocking reuse of samples until acknowledged
""""""""""""""""""""""""""""""""""""""""""""

With |KEEP_LAST_HISTORY_QOS-api| or |BEST_EFFORT_RELIABILITY_QOS-api| configurations,
the DataWriter can remove samples from its history to add new ones,
even if they were not acknowledged by the DataReader.
In situations where the publishing rate is consistently faster
than the rate at which the DataReader can process the samples,
this can lead to every sample being reused before the application
has a chance to process it, thus blocking the communication at application level.

In order to avoid this situation,
the samples in the preallocated pool are never reused unless they have been acknowledged,
i.e., they have been processed by the application at least once.
If there is no reusable sample in the pool,
the writing operation in the DataWriter will be blocked until one is available
or until |ReliabilityQosPolicy::max_blocking_time-api| is reached.

Note that the DataWriter history is not affected by this behavior,
samples will be removed from the history by standard rules.
Only the reuse of pool samples is affected.
This means that the DataWriter history can be empty and the write operation
be still blocked because all samples in the pool are unacknowledged.

The chance of the DataWriter blocking on a write operation can be reduced
using |ResourceLimitsQosPolicy::extra_samples-api|.
This will make the pool to allocate more samples than the history size,
so that the DataWriter has more chances to get a free sample,
while the DataReader can still access samples that have been removed from the
DataWriter history.
