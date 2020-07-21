.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-reduceMemory:

Reduce the usage of memory
==========================

A great number of modern systems have tight constraints over available memory. Reducing the memory usage is critical for
these applications. Memory consumption of your Fast-DDS application can be achieved through a different number of
approaches, mainly through architectural restructuring, but also via the limiting of resources and by avoiding static
allocations.

If your system has a low memory limit it would be recommended to limit the maximum resources of the application to
ensure that it will run properly. To lower the usage during runtime, allocations can be set dynamically to ensure the
lowest amount of memory is always in use.

QoS Adjustments
---------------

ResourceLimitsQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^
Limit the resources to the application's needs. At the lowest limit we can reduce it to one sample per |DataWriter-api|
and |DataReader-api|.


.. warning::

   The value of |ResourceLimitsQosPolicy::max_samples-api| must be higher or equal to the value of
   |ResourceLimitsQosPolicy::max_samples_per_instance-api|.

   The value established for the :ref:`historyqospolicy` |HistoryQosPolicy::depth-api| must be lower or equal to the
   value stated for |ResourceLimitsQosPolicy::max_samples_per_instance-api|.

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-MEMORY-QOS-PUBSUB              |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-MEMORY-QOS-PUBSUB           |
|    :end-before: <!--><-->                              |
|    :lines: 2-3,5-                                      |
|    :append: </profiles>                                |
+--------------------------------------------------------+

Set Dynamic Allocation
^^^^^^^^^^^^^^^^^^^^^^

Use the dynamic settings of the :ref:`rtpsendpointqos` to prevent allocating unused memory. Lowest footprint is achieved
with |DYNAMIC_RESERVE_MEMORY_MODE-api| at the cost of higher allocation counts, for higher determinism at a small memory
cost use |DYNAMIC_REUSABLE_MEMORY_MODE-api|.



+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp         |
|    :language: c++                                      |
|    :start-after: //CONF-MEMORY-QOS-ENDPOINTS           |
|    :end-before: //!--                                  |
|    :dedent: 8                                          |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-MEMORY-QOS-ENDPOINTS        |
|    :end-before: <!--><-->                              |
|    :lines: 2-3,5-                                      |
|    :append: </profiles>                                |
+--------------------------------------------------------+


.. note::
    Memory allocation modes have different names for XML config files and C++ source code.

    +--------------------------------------+----------------------------+
    |C++                                   | XML                        |
    +======================================+============================+
    |PREALLOCATED_MEMORY_MODE              | PREALLOCATED               |
    +--------------------------------------+----------------------------+
    |PREALLOCATED_WITH_REALLOC_MEMORY_MODE | PREALLOCATED_WITH_REALLOC  |
    +--------------------------------------+----------------------------+
    |DYNAMIC_RESERVE_MEMORY_MODE           | DYNAMIC                    |
    +--------------------------------------+----------------------------+
    |DYNAMIC_REUSABLE_MEMORY_MODE          | DYNAMIC_REUSABLE           |
    +--------------------------------------+----------------------------+

