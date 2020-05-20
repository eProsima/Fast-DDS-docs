.. _dds_layer_core_policy:

Policy
======

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to  to define how each
entity will behave.
To increase the flexibility of the system, the QoS is decomposed in several QoS Policies that can be configured
independently.
However, there may be cases where several policies conflict.

There are QoS Policies that are immutable, which means that only can be specified either at the entity creation or
before calling the enable operation.

Each DDS Entity has a specific set of QoS Policies that can be a mix of Standard QoS Policies, XTypes Extensions and
eProsima Extensions.

.. toctree::
   :maxdepth: 2

   /fastdds/dds_layer/core/policy/standardQosPolicies.rst
   /fastdds/dds_layer/core/policy/eprosimaExtensions.rst
   /fastdds/dds_layer/core/policy/xtypesExtensions.rst
