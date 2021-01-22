.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _datasharing-delivery:

Data-sharing delivery
=====================

*Fast DDS* allows to speed up communications between entities within the same machine
by sharing the history of the |DataWriter| with the |DataReader| through shared memory,
thus avoiding any of the overhead involved in the transport layer,
and effectively avoiding any data copy between DataWriter and DataReader.

.. note::
    Although Data-sharing delivery uses sahred memory,
    it differs from :ref:`transport_sharedMemory_sharedMemory`
    in that :ref:`transport_sharedMemory_sharedMemory` is a full-compliant transport.
    That means that with :ref:`transport_sharedMemory_sharedMemory`
    the data being transmitted must be copied from the DataWriter history to the transport
    and from the transport to the DataReader.
    With Data-sharing these copies can be avoided

The figure below shows a comparison between the different transports available in *Fast DDS*.

.. figure:: /01-figures/fast_dds/transport/transport_comparison.png
    :align: center

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


Constraints
-----------

This feature is available only if the following requirements are met:

* The |DataWriter| and |DataReader| have access to the same shared memory.
* The |Topic| has a bounded |TopicDataType|,
  i.e., its |TopicDataType::is_bounded-api| member function returns true.
* The DataWriter is configured with |PREALLOCATED_MEMORY_MODE-api| or |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|.


Data-sharing delivery configuration
-----------------------------------

Data-sharing delivery can be configured in the |DataWriter| and the |DataReader|
using :ref:`datasharingqospolicy`.
Three attributes can be configured: the data-sharing delivery kind, the shared memory directory,
and the data-sharing domain identifiers.

Data-Sharing delivery kind
""""""""""""""""""""""""""

Can be set to one of three modes:

* **AUTO**: If the requisites are met,
  data-sharing delivery will be used with entities that are data-sharing compatible.
  If the requisites are not met, no data-sharing delivery will be used.
* **ON**: Like **AUTO**, but the creation of the entity will fail if the requisites are not met.
* **OFF**: No data-sharing delivery will be used on this entity.

The following matrix shows when two entities are data-sharing compatible according to their configuration
(given that the entity creation does not fail):

.. |common_ids| replace:: Only if they have common domain IDs
.. |bounded_and_common_ids| replace:: Only if the TopicDataType is bounded and they have common domain IDs

+------+------------------+-------------------+--------------------------+
|      | ON               | OFF               | AUTO                     |
+======+==================+===================+==========================+
| ON   | |common_ids|     | No                | |common_ids|             |
+------+------------------+-------------------+--------------------------+
| OFF  | No               | No                | No                       |
+------+------------------+-------------------+--------------------------+
| AUTO | |common_ids|     | No                | |bounded_and_common_ids| |
+------+------------------+-------------------+--------------------------+

Data-sharing domain identifiers
"""""""""""""""""""""""""""""""

Each entity defines a set of identifiers that represent a *domain* to which the entity belong.
Two entities will be able to use data-sharing delivery between them only if both have at least a common domain.

Users can define the domains of a |DataWriter| or |DataReader| with the :ref:`datasharingqospolicy`.
If no domain identifier is provided by the user, the system will create one automatically.
This automatic data-sharing domain will be unique for the machine where the entity is running.
That is, all entities running on the same machine, and for which the user has configured no user-specific domains,
will be able to use data-sharing delivery (given that the rest of requisites are met).

Even though a data-sharing domain identifier is a 64 bit integer,
user-defined identifiers are restricted to 16 bit integers.


Shared memory directory
"""""""""""""""""""""""

If a user-defined directory is given for the shared memory files,
this directory will be used for the memory-mapped files used for data-sharing delivery.
If none is given, the default directory configured for the current system is used.

Configuring a user-defined directory may be useful in some scenarios:

* To select a file system with Huge TLB enabled for the memory-mapped files.
* To allow data-sharing delivery between containers that mount the same container.

.. warning::

    Currently the configuration of shared memory directory is not supported.
    As a result, any directory set by the user will be discarded,
    and the default directory configured for the current system is used.


Effects of the writer history configuration
-------------------------------------------

With traditional :ref:`comm-transports-configuration` delivery,
the DataReader and DataWriter keep separate and independent histories,
each one with their own copy of the sample.
Once the samples is sent through the transport and received by the DataReader,
the DataWriter is free to remove the sample from its history
without affecting the DataReader.

With data-sharing delivery, DataReader directly accesses the shared history of the DataWriter.
This means that the samples in both the history of the DataReader and the DataWriter
are the same object in the shared memory.
If the DataWriter removes the sample from its history, the DataReader loses access to it.
Therefore, there is a strong coupling in the behavior of the DataReader and DataWriter histories.

Reliable - keep all
"""""""""""""""""""

With data-sharing delivery, sample acknowledgment from the DataReader occurs the first time
a sample is retrieved by the application (using |DataReader::read_next_sample-api|,
|DataReader::take_next_sample-api|, or any of their variations).
Therefore, this configuration ensures that the DataWriter will not remove a samples that has not been seen
by the reader end application at least once.

Reliable - keep last
""""""""""""""""""""

With |KEEP_LAST_HISTORY_QOS-api| configuration, the DataWriter can reuse the oldest sample
in its history to add a new one, even if it was not acknowledged by the DataReader.
This is specially harmful with a shallow history depth, but even with deep history depths,
if the publishing rate is consistently higher than the rate at which the DataReader can process the samples,
the DataWriter history will become full and samples will be reused before being acknowledged.

If the publications come in bursts, the problem can be mitigated configuring
|ResourceLimitsQosPolicy::extra_samples-api|.
Instead of increasing the size of the DataWriter history,
this parameter defines the number of samples that will be allocated to act as an extended buffer
before an unacknowledged samples is reused.
The DataWriter will behave normally, according to its history configuration,
but the DataReader will have a total of
|ResourceLimitsQosPolicy::max_samples-api| plus |ResourceLimitsQosPolicy::extra_samples-api|
samples available for the application.

The value of |ResourceLimitsQosPolicy::extra_samples-api| will strongly depend on the
publication and reading rates, and in the duration of the publication bursts.

Best effort
"""""""""""

With |BEST_EFFORT_RELIABILITY_QOS-api| the behavior is similar to that of
|KEEP_LAST_HISTORY_QOS-api|, even worse,
no sample waits for an acknowledge on the DataWriter.
The solution is to use |ResourceLimitsQosPolicy::extra_samples-api| again.
