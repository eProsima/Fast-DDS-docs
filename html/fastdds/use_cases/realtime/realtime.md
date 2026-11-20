<a id="use-case-realtime"></a>

<a id="real-time-behavior"></a>

# Real-time behavior

Real-time applications have very tight constraints on data processing times.
In order to comply with these constraints, *Fast DDS* can be configured to guarantee responses within
a specified time.
This is achieved with the following restraints:

- Allocating all the required memory during entity initialization, so that all the data processing
  tasks are heap allocation free (see [Tuning allocations](allocations.md#realtime-allocations)).
- Returning from blocking functions if the provided timeout is reached (see [Non-blocking calls](blocking.md#non-blocking-calls)).

This section explains how to configure *Fast DDS* to achieve this behavior.

* [Tuning allocations](allocations.md)
  * [Parameters on the participant](allocations.md#parameters-on-the-participant)
  * [Parameters on the DataWriter](allocations.md#parameters-on-the-datawriter)
  * [Parameters on the DataReader](allocations.md#parameters-on-the-datareader)
  * [Full example](allocations.md#full-example)
* [Non-blocking calls](blocking.md)
