.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _topicprofiles:

Topic profiles
--------------

The |Topic| determine whether Datawriters and DataReaders can exchange messages.
Please refer to :ref:`dds_layer_topic` section for a a deeper explanation on the |Topic| class.
The Topic profile defines the ``<historyQos>`` and the ``<resourceLimitsQos>`` elements
(see |HistoryQosPolicy| and |ResourceLimitsQosPolicy|) for all DataWriters/DataReaders using it.

+-------------------------+-----------------------------------------------+--------------------------------------------+
| Name                    | Description                                   | Values                                     |
+=========================+===============================================+============================================+
| ``<historyQos>``        | It controls the behavior of *Fast DDS* |br|   | :ref:`hQos`                                |
|                         | when the value of an instance changes  |br|   |                                            |
|                         | before it is finally communicated to |br|     |                                            |
|                         | some of its existing DataReaders. |br|        |                                            |
+-------------------------+-----------------------------------------------+--------------------------------------------+
| ``<resourceLimitsQos>`` | It controls the resources that *Fast DDS*     | :ref:`rLsQos`                              |
|                         | |br| can use in order to meet the |br|        |                                            |
|                         | requirements imposed by the application |br|  |                                            |
|                         | and other QoS settings.                       |                                            |
+-------------------------+-----------------------------------------------+--------------------------------------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-17, 19-20

.. _hQos:

HistoryQoS
""""""""""

It controls the behavior of *Fast DDS* when the value of an instance changes before it is finally
communicated to some of its existing DataReaders.
Please refer to :ref:`HistoryQosPolicyKind` for further information on HistoryQoS.

+-------------+---------------------------------------------------------+-----------------------+----------------------+
| Name        | Description                                             | Values                | Default              |
+=============+=========================================================+=======================+======================+
| ``<kind>``  | *Fast DDS* will only attempt to keep the latest values  | |KEEP_LAST-xml-api|   | |KEEP_LAST-xml-api|  |
|             | of the instance |br| and discard the older ones.        |                       |                      |
|             +---------------------------------------------------------+-----------------------+                      |
|             | *Fast DDS* will attempt to maintain and deliver all the | |KEEP_ALL-xml-api|    |                      |
|             | values of the instance |br| to existing DataReaders.    |                       |                      |
+-------------+---------------------------------------------------------+-----------------------+----------------------+
| ``<depth>`` | It must be consistent with the :ref:`rLsQos`            | ``uint32_t``          | 1                    |
|             | ``<max_samples_per_instance>`` |br|                     |                       |                      |
|             | element value. It must be verified that: |br|           |                       |                      |
|             | ``<depth>`` `<=` ``<max_samples_per_instance>``.        |                       |                      |
+-------------+---------------------------------------------------------+-----------------------+----------------------+

.. _rLsQos:

ResourceLimitsQos
"""""""""""""""""

It controls the resources that *Fast DDS* can use in order to meet the requirements imposed by the
application and other QoS settings.
Please refer to :ref:`ResourceLimitsQosPolicy` for further information on ResourceLimitsQos.

+--------------------------------+-----------------------------------------------------------+---------------+---------+
| Name                           | Description                                               | Values        | Default |
+================================+===========================================================+===============+=========+
| ``<max_samples>``              | It must verify that:                                      | ``uint32_t``  | 5000    |
|                                | ``<max_samples>`` `>=` ``<max_samples_per_instance>``.    |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_instances>``            | It defines the maximum number of instances.               | ``uint32_t``  | 10      |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_samples_per_instance>`` | It must verify that: :ref:`HistoryQos <hQos>`             | ``uint32_t``  | 400     |
|                                | ``<depth>`` `<=` ``<max_samples_per_instance>``.          |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<allocated_samples>``        | It controls the maximum number of samples to be stored.   | ``uint32_t``  | 100     |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
