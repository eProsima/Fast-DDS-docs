.. _dds_layer_core:

Core
====

This module defines the infrastructure classes and types that will be used by the other ones. It contains the
definition of Entity class, QoS policies, and Statuses.

- **Entity:** An *Entity* is a DDS communication object that has a *Status* and can be configured with *Policies*.
- **Policy:** Each of the configuration objects that govern the behavior of an *Entity*.
- **Status:** Each of the objects associated with an *Entity*,
  whose values represent the *communication status* of that *Entity*.


.. toctree::
   :maxdepth: 3

   /fastdds/dds_layer/core/entity/entity
   /fastdds/dds_layer/core/policy/policy
   /fastdds/dds_layer/core/status/status
   /fastdds/dds_layer/core/waitsets/waitsets
