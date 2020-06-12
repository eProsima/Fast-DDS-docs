.. _use-case-realtime:

Real-time behavior
==================

Real-time applications have very tight constraints on data processing times.
In order to comply with these constraints, *Fast DDS* can be configured to guarantee responses within
a specified time.
This is achieved with the following restraints:

- Allocating all the required memory during entity initialization, so that all the data processing
  tasks are heap allocation free (see :ref:`realtime-allocations`).
- Returning from blocking functions if the provided timeout is reached (see :ref:`non-blocking-calls`).

This section explains how to configure *Fast DDS* to achieve this behavior.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/realtime/allocations.rst
    /fastdds/use_cases/realtime/blocking.rst
