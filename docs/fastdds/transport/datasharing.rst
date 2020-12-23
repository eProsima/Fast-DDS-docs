.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _datasharing-delivery:

Data-sharing delivery
=====================

eProsima Fast DDS allows to speed up communications between entities within the same machine
by sharing the history of the |DataWriter| with the |DataReader| through shared memory,
thus avoiding any of the overhead involved in the transport layer,
and effectively achieving zero-copy between DataWriter and DataReader.

This feature is available only if the following requisites are met:

* The |Topic| has a bounded |TopicDataType|,
  i.e., its |TopicDataType::is_bounded-api| member function returns true.
* The |DataWriter| is configured with |PREALLOCATED_MEMORY_MODE-api| or |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|.
* The |DataWriter| is configured with |KEEP_ALL_HISTORY_QOS-api| and |RELIABLE_RELIABILITY_QOS-api|.
* The |DataReader| is configured with |RELIABLE_RELIABILITY_QOS-api|.


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


