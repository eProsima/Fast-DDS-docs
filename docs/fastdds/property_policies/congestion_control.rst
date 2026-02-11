.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_congestion_control:

Congestion Control Settings |Pro|
---------------------------------

When using :ref:`Congestion Control <congestion-control>` |Pro|, the DomainParticipant may need specific parameters
to be set.
Properties related with this feature lie on the ``fastdds.congestion`` namespace.
At least one of these properties should be set in order for the congestion control to be enabled.

* Property ``fastdds.congestion.plugin`` is used to set the congestion control plugin to be used.
  If the property is not present, it will be set to ``basic``.

* Property ``fastdds.congestion.period`` is used to set the interval in milliseconds for checking whether the bandwidth
  to certain destinations should be updated.
  If the property is not present, it will be set to ``10000`` (10 seconds).

* Property ``fastdds.congestion.initial_bandwidth`` is used to set the bandwidth that will be initially assigned to
  new destinations, in bytes per second.
  If the property is not present, it will be set to half the size of the socket send buffer.

* Property ``fastdds.congestion.increase_multiplier`` is used to set the multiplier that will be applied to increase
  the bandwidth to a destination when no congestion is detected.
  If the property is not present, it will be set to ``1.2``.

* Property ``fastdds.congestion.decrease_multiplier`` is used to set the multiplier that will be applied to decrease
  the bandwidth to a destination when congestion is detected.
  If the property is not present, it will be set to ``0.75``.
