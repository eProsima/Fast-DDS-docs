.. include:: ../../../03-exports/aliases-api.include

.. _dds_layer_topic_instances:

Topics, keys and instances
==========================

Even though, by definition, a Topic corresponds to a single data type, it is possible that several topics may refer to
the same data type.
Therefore, a Topic identifies data of a single type, ranging from one single instance to a whole collection of instances
of that given type, as shown in the figure below.

.. figure:: /01-figures/instances.png
    :align: center

The different instances gathered under the same topic are distingishable by means of some data fields that form the key
to that data set.
The key description has to be indicated to the middleware.
The rule is simple: different data values with the same key value represent succesive values for the same instance,
while different data values with different keys represent different instances.
If no key is provided, the data set associated with the Topic is restricted to a single instance.
Please refer to :ref:`dds_layer_topic_keyed_data_types` for more information about how to set the key in
*eProsima Fast DDS*.

.. _dds_layer_topic_instance_lifecycle:

Instance lifecycle
------------------

When reading or taking data from the :ref:`dds_layer_subscriber_dataReader` (as explained in
:ref:`dds_layer_subscriber_accessreceived`), a :ref:`dds_layer_subscriber_sampleInfo` is also returned.
This |SampleInfo-api| provides additional information about the instance lifecycle, specifically with the
:ref:`dds_layer_subscriber_sampleInfo_viewstate`, :ref:`dds_layer_subscriber_sampleInfo_instancestate`,
:ref:`dds_layer_subscriber_sampleInfo_disposedgenerationcount`, and
:ref:`dds_layer_subscriber_sampleInfo_nowritersgenerationcount`.
The diagram below shows the statechart of |SampleInfo::instance_state-api| and |SampleInfo::view_state-api| for a single
instance.

.. figure:: /01-figures/instance-lifecycle.png
    :align: center
