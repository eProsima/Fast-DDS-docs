.. |br| raw:: html

   <br />

.. _realtime:

Real-time
#########

Fast RTPS can be configured to offer real-time features.
These features will guarantee Fast RTPS responses within specified time constrains.
To maintain this compromise Fast RTPS is able to have the following behavior:

- Not allocate memory after the initialization of Fast RTPS entities.
- Several methods are blocked for a maximum period of time.

This section explains how to configure Fast RTPS to achieve this behavior.
For easier understanding it was divided in two subsections:

- :ref:`allocations`: configuration to avoid memory allocation after initialization.
- :ref:`non-blocking-calls`: usage of non-blocking methods for real-time behavior.

.. _allocations:

Allocations
***********

.. _non-blocking-calls:

Non-blocking calls
******************

**Note:** This feature is not fully supported on OSX.
It doesn't support necessary POSIX Real-time features.
The feature is limited by the implementation of `std::timed_mutex` and `std::condition_variable_any`.

It is important that  a method isn't blocked for indeterminate time to achieve real-time.
A method must only be blocked for a maximum period of time.
In Fast-RTPS API there are several methods that permit to set this.

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
