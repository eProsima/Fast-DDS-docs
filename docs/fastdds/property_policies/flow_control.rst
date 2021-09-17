.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_flow_control:

Flow Controller Settings
------------------------

When using :ref:`flow-controllers`, the DataWriter may need specific parameters to be set.
Properties related with this feature lie on the ``fastdds.sfc`` namespace.

* Property ``fastdds.sfc.priority`` is used to set the priority of the DataWriter for
  |HIGH_PRIORITY_SCHED_POLICY-api| and |PRIORITY_WITH_RESERVATION_SCHED_POLICY-api| flow controllers.
  Allowed values are from -10 (highest priority) to 10 (lowest priority).
  If the property is not present, it will be set to the lowest priority.

* Property ``fastdds.sfc.bandwidth_reservation`` is used to set the percentage of the bandwidth that
  the DataWriter is requesting for |PRIORITY_WITH_RESERVATION_SCHED_POLICY-api| flow controllers.
  Allowed values are from 0 to 100, and express a percentage of the total flow controller limit.
  If the property is not present, it will be set to 0 (no bandwidth is reserved for the DataWriter).
