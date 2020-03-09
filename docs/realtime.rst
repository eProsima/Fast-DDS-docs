.. |br| raw:: html

   <br />

.. _realtime:

Real-time behavior
##################

Fast RTPS can be configured to offer real-time features.
These features will guarantee Fast RTPS responses within specified time constrains.
To maintain this compromise Fast RTPS is able to have the following behavior:

- Not allocate memory after the initialization of Fast RTPS entities.
- Several methods are blocked for a maximum period of time.

This section explains how to configure Fast RTPS to achieve this behavior.
For easier understanding it was divided in two subsections:

- :ref:`realtime-allocations`: configuration to avoid memory allocation after initialization.
- :ref:`non-blocking-calls`: usage of non-blocking methods for real-time behavior.

.. _realtime-allocations:

Tuning allocations
******************

Some important non-deterministic operating system calls are the ones for allocating and deallocating memory.
Most real-time systems have the need to operate in a way that all dynamic memory is allocated on the application
startup, and avoid calls to memory management APIs on the main loop.

Fast-RTPS provides some configuration parameters to meet these requirements, allowing the items of internal
data collections to be preallocated.
In order to choose the correct values for these parameters, the user should be aware of the topology of the whole
domain, so the number of participants and endpoints should be known when setting them.

Parameters on the participant
=============================

All the allocation related parameters on the participant are grouped into the :class:`rtps.allocation` field of the
:class:`ParticipantAttributes` struct.

Limiting the number of discovered participants
----------------------------------------------

Every participant in Fast-RTPS holds an internal collection of :class:`ParticipantProxyData` objects with the
information of the local and the remote participants.
Field :class:`participants` inside :class:`RTPSParticipantAllocationAttributes` allows the configuration of
the allocation behavior of that collection.
The user can specify the :class:`initial` number of elements preallocated, the :class:`maximum` number of elements
allowed, and the allocation :class:`increment`.
By default, a full dynamic behavior is used.

Limiting the number of discovered endpoints
-------------------------------------------

Every :class:`ParticipantProxyData` object holds internal collections with the :class:`ReaderProxyData` and
:class:`WriterProxyData` objects with the information of the readers and writers of a participant.
In a similar way to the :class:`participants` field, :class:`RTPSParticipantAllocationAttributes` has fields
:class:`readers` and :class:`writers` to set the configuration of the allocation behavior of those collections.
The user can specify the :class:`initial` number of elements preallocated, the :class:`maximum` number of elements
allowed, and the allocation :class:`increment`.
By default, a full dynamic behavior is used.

Limiting the size of parameters
-------------------------------

Most of the information held for participants and endpoints have a defined size limit, so the amount of memory to
allocate for each local and remote peer is known. For the parameters which size is not limited, a maximum size can be
configured with :class:`RTPSParticipantAllocationAttributes::data_limits`, which has the following attributes:

* :class:`max_partitions` limits the size of partition data to the given number of octets.
* :class:`max_user_data` limits the size of user data to the given number of octets.
* :class:`max_properties` limits the size of participant properties data to the given number of octets.

A value of zero implies no size limitation. If these sizes are configured to something different than zero, enough
memory will be allocated for them for each participant and endpoint.
If these sizes are not limited, memory will be dynamically allocated as needed.
By default, a full dynamic behavior is used.

Parameters on the publisher
===========================

Every publisher holds a collection with some information regarding the subscribers it has matched to.
Field :class:`matched_subscriber_allocation` inside :class:`PublisherAttributes` allows the configuration of
the allocation behavior of that collection.
The user can specify the :class:`initial` number of elements preallocated, the :class:`maximum` number of elements
allowed, and the allocation :class:`increment`.
By default, a full dynamic behavior is used.

Parameters on the subscriber
============================

Every subscriber holds a collection with some information regarding the publishers it has matched to.
Field :class:`matched_publisher_allocation` inside :class:`SubscriberAttributes` allows the configuration of
the allocation behavior of that collection.
The user can specify the :class:`initial` number of elements preallocated, the :class:`maximum` number of elements
allowed, and the allocation :class:`increment`.
By default, a full dynamic behavior is used.

Full example
============

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

* All the subscribers match exactly with 1 publisher.
* The publisher for topic 1 matches with 3 subscribers, and the publisher for topic 2 matches with 2 subscribers.
* The maximum number of publishers per participant is 1, and the maximum number of subscribers per participant is 2.
* The total number of participants is 3.

We will also limit the size of the parameters:

* Maximum partition data size: 256
* Maximum user data size: 256
* Maximum properties data size: 512

The following piece of code shows the set of parameters needed for the use case depicted in this example.

+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp          |
|    :language: c++                                   |
|    :start-after: //CONF-ALLOCATION-QOS-EXAMPLE      |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml           |
|    :language: xml                                   |
|    :start-after: <!-->CONF-ALLOCATION-QOS-EXAMPLE   |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+


.. _non-blocking-calls:

Non-blocking calls
******************

**Note:** This feature is not fully supported on OSX.
It doesn't support necessary POSIX Real-time features.
The feature is limited by the implementation of `std::timed_mutex` and `std::condition_variable_any`.

It is important that a method isn't blocked for indeterminate time to achieve real-time.
A method must only be blocked for a maximum period of time.
In Fast-RTPS API there are several methods that permit to set this. But first Fast-RTPS should be configured with the
CMake option ``-DSTRICT_REALTIME=ON``. The list of these functions is displayed in the table below.

.. list-table:: **Fast RTPS non-blocking API**
   :header-rows: 1
   :align: left

   * - Method
     - Description
   * - Publisher::write()
     - These methods are blocked for a period of time.
       *ReliabilityQosPolicy.max_blocking_time* on *PublisherAttributes* defines this period of time.
       Default value is 100 milliseconds.
   * - Subscriber::takeNextData()
     - This methods is blocked for a period of time.
       *ReliabilityQosPolicy.max_blocking_time* on *SubscriberAttributes* defines this period of time.
       Default value is 100 milliseconds.
   * - Subscriber::readNextData()
     - This method is blocked for a period of time.
       *ReliabilityQosPolicy.max_blocking_time* on *SubscriberAttributes* defines this period of time.
       Default value is 100 milliseconds.
   * - Subscriber::wait_for_unread_samples()
     - Accepts an argument specifying how long the method can be blocked.
