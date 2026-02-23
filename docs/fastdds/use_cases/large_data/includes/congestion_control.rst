.. _congestion-control:

Congestion Control |Pro|
------------------------

When transmitting large amounts of data over a network, it is crucial to manage congestion to ensure optimal
performance and prevent packet loss.
Fast DDS |Pro| allows enabling a congestion control mechanism on a participant, which makes every DataWriter use
a different bandwidth limitation on each DataReader according to the network conditions.

This feature can be customized through :ref:`congestion control plugins <congestion-control-plugins>`, which define
the algorithm used to adjust the bandwidth based on the detected congestion.

To enable congestion control, the DomainParticipant must be configured with the :ref:`appropriate properties <_property_policies_congestion_control-control-properties>`.

.. _congestion-control-plugins:

Congestion Control Plugins |Pro|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The congestion control mechanism in Fast DDS |Pro| can be customized through plugins that define the algorithm used to
adjust the bandwidth associated to each DataReader. At the moment, only the ``basic`` plugin is available.

.. _basic-congestion-control-plugin:

The ``basic`` Congestion Control
""""""""""""""""""""""""""""""""

The ``basic`` congestion control plugin dynamically adjusts the sending rate to each
remote DataReader based on observed network feedback. It operates independently per
remote DataReader and only applies to **reliable**, non-builtin DataWriters. Best-effort
writers are unaffected.

.. note::

    Enabling congestion control forces all affected DataWriters into **asynchronous,
    separate sending mode**: instead of sending a single batched or multicast packet to
    all matched readers at once, a dedicated packet is sent to each reader individually.
    As a result, total network traffic scales with the number of matched DataReaders.
    In deployments with many readers on a multicast-capable network, this may **increase**
    overall network load. Congestion control is therefore most effective in unicast or
    low-reader-count scenarios.

.. note::

    If a DataWriter already has a custom flow controller configured, it will be
    **overridden** by the congestion control's internal flow controller.


The ``basic`` congestion control algorithm is governed by these four properties, that are f
urther detailed :ref:`here<_property_policies_congestion_control>``:

- **Period** (``period_duration_ms``): Time interval, in milliseconds, at which the
  algorithm evaluates and adjusts the bandwidth limit for each reader.
- **Initial bandwidth** (``initial_target_bytes_per_second``): Starting bandwidth cap,
  in bytes per second, assigned to each newly discovered DataReader.
- **Increase factor** (``increase_factor``): Multiplier applied to the current bandwidth
  limit when the algorithm decides to increase it.
- **Decrease factor** (``decrease_factor``): Multiplier applied to the current bandwidth
  limit when the algorithm decides to decrease it.

At the end of each period, the algorithm evaluates each tracked remote DataReader
independently resulting in three possible cases:

- **Decrease**: If the DataWriter had to retransmit data to the reader during the period
  (signalling packet loss or network congestion), the current bandwidth limit is multiplied
  by the ``decrease_factor``. Note that the first repair attemptfor a given sequence number
  is not counted to avoid overreacting to isolated packet losses.

  .. note::

    The bandwidth limit has a lower bound equal to the participant's maximum datagram size,
    ensuring that at least one message can always be sent per period regardless of how many
    times the limit is decreased.

- **Increase**: If no retransmissions occurred and the reader's acknowledged throughput
  exceeded 80% of the current bandwidth limit, the limit is multiplied by the
  ``increase_factor``. The 80% threshold allows the limit to grow before full saturation,
  preserving some space to absorb traffic bursts.
- **No change**: If neither condition is met, the bandwidth limit remains unchanged.




