.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-reduce-memory:

Reduce memory usage
===================

A great number of modern systems have tight constraints on available memory, making the reduction of memory usage to a
minimum critical. Reducing memory consumption of a *Fast DDS* application can be achieved through various approaches,
mainly through architectural restructuring of the application, but also by limiting the resources the
middleware utilizes, and by avoiding static allocations.

.. |max_samples| replace:: |ResourceLimitsQosPolicy::max_samples-api|
.. |max_instances| replace:: |ResourceLimitsQosPolicy::max_instances-api|
.. |max_samples_per_instance| replace:: |ResourceLimitsQosPolicy::max_samples_per_instance-api|
.. |allocated_samples| replace:: |ResourceLimitsQosPolicy::allocated_samples-api|
.. |depth| replace:: |HistoryQosPolicy::depth-api|

Limiting Resources
^^^^^^^^^^^^^^^^^^^^^^^
The :ref:`resourcelimitsqospolicy` controls the resources that the service can use in order to meet the requirements
imposed. It limits the amount of allocated memory per :ref:`dds_layer_publisher_dataWriter` or
:ref:`dds_layer_subscriber_dataReader`, as per the following parameters:

* |max_samples|: Configures the maximum number of samples that the DataWriter or DataReader can manage across all the
  instances associated with it, i.e. it represents the maximum samples that the middleware can store for a DataReader or
  DataWriter.
* |max_instances|: Configures the maximum number of instances that the DataWriter or DataReader can manage.
* |max_samples_per_instance|: Controls the maximum number of samples within an instance  that the DataWriter or
  DataReader can manage.
* |allocated_samples|: States the number of samples that will be allocated on initialization.

All these parameters may be lowered as much as needed to reduce memory consumption, limit the resources to the
application's needs. Below is an example of a configuration for the minimum resource limits possible.

.. warning::

  * The value of |max_samples| must be higher or equal to the value of |max_samples_per_instance|.

  * The value established for the :ref:`historyqospolicy` |depth| must be lower or equal to the value stated for
    |max_samples_per_instance|.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-MEMORY-QOS-PUBSUB
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-MEMORY-QOS-PUBSUB
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

Set Dynamic Allocation
^^^^^^^^^^^^^^^^^^^^^^
By default :ref:`memorymanagementpolicy` is set to |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|, meaning that the
amount of memory required by the configured :ref:`resourcelimitsqospolicy` will be allocated at initialization.
If some more memory has to be allocated at run time, it is reallocated.

Using the dynamic settings of the :ref:`rtpsendpointqos` will prevent unnecessary allocations. Lowest footprint is
achieved with |DYNAMIC_RESERVE_MEMORY_MODE-api| at the cost of higher allocation counts, in this mode memory is
allocated when needed and freed as soon as it stops being used. For higher determinism at a small memory cost the
|DYNAMIC_REUSABLE_MEMORY_MODE-api| option is available, this option is similar but once more memory is allocated it is
not freed and is reused for future messages.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-MEMORY-QOS-ENDPOINTS
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-MEMORY-QOS-ENDPOINTS
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

