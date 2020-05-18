.. _dds_layer_subscriber_sampleInfo:

SampleInfo
==========

When a sample is *read* or *taken* on the :ref:`dds_layer_subscriber_dataReader`, in addition to the sample data,
a :class:`SampleInfo` instance is returned.
This object contains additional information that complements the returned data value and helps on it interpretation.
For example, if the :ref:`dds_layer_subscriber_sampleInfo_validdata` value is ``false``, the
:ref:`dds_layer_subscriber_dataReader` is not informing the application about a new value in the data instance,
but a change on its status, and the returned data value must be discarded.

The following sections describe the data members of :class:`SampleInfo` and the meaning of each one in relation
to the returned sample data.

sample_state
------------

A :ref:`api_pim_samplestatekind` enumeration that indicates whether or not the corresponding data sample has already
been read previously.
It can take one of these values:

* **READ**: This is the first time this data sample has been retrieved.
* **NOT_READ**: The data sample has already been *read* or *taken* previously.

.. note::
   Currently the ``sample_state`` is not implemented, and its value is always set to **NOT_READ**.
   It will be implemented on a future release of Fast DDS.


view_state
----------

A :ref:`api_pim_viewstatekind` enumeration that indicates whether or not this is the very first sample
of this data instance that the :ref:`dds_layer_subscriber_dataReader` retrieves.
It can take one of these values:

* **NEW**: This is the first time a sample of this instance is retrieved.
* **NOT_NEW**: Other samples of this instance have been retrieved previously.

.. note::
   Currently the ``view_state`` is not implemented, and its value is always set to **NOT_NEW**.
   It will be implemented on a future release of Fast DDS.


instance_state
--------------

A :ref:`api_pim_instancestatekind` enumeration that indicates whether the instance is currently in existence
or it has been disposed.
In the latter case, it also provides information about the reason for the disposal.
It can take one of these values:

* **ALIVE**: The instance is currently in existence.
* **NOT_ALIVE_DISPOSED**: A remote :ref:`dds_layer_publisher_dataWriter` disposed the instance.
* **NOT_ALIVE_NO_WRITERS**: The :ref:`dds_layer_subscriber_dataReader` disposed the instance because no remote
  :ref:`dds_layer_publisher_dataWriter` that was publishing the instance is *alive*.

.. note::
   Currently the ``instance_state`` is partially implemented, and the value **NOT_ALIVE_NO_WRITERS** will never be set.
   It will be fully implemented on a future release of Fast DDS.


disposed_generation_count
-------------------------

Indicates the number of times the instance had become alive after it was disposed.

.. note::
   Currently the ``disposed_generation_count`` is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of Fast DDS.


no_writers_generation_count
---------------------------

Indicates the number of times the instance had become alive after it was disposed as ``NOT_ALIVE_NO_WRITERS``.

.. note::
   Currently the ``no_writers_generation_count`` is not implemented, and its value is always set to ``1``.
   It will be implemented on a future release of Fast DDS.


sample_rank
-----------

Indicates the number of samples of the same instance that have been received after this one.
For example, a value of ``5`` means that there are 5 newer samples available
on the :ref:`dds_layer_subscriber_dataReader`.

.. note::
   Currently the ``sample_rank`` is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of Fast DDS.


generation_rank
---------------

Indicates the number of times the instance was disposed and become alive again
between the time the sample was received and the time the most recent sample of the same instance
that is still held in the collection was received.

.. note::
   Currently the ``generation_rank`` is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of Fast DDS.


absolute_generation_rank
------------------------

Indicates the number of times the instance was disposed and become alive again
between the time the sample was received and the time the most recent sample of the same instance
(which may not be in the collection) was received.

.. note::
   Currently the ``absolute_generation_rank`` is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of Fast DDS.


source_timestamp
----------------

It holds the time stamp provided by the :ref:`dds_layer_publisher_dataWriter` when the sample was published.


instance_handle
---------------

The instance handle of the local instance.


publication_handle
------------------

The instance handle of the :ref:`dds_layer_publisher_dataWriter` that published the data change.


.. _dds_layer_subscriber_sampleInfo_validdata:

valid_data
----------

A boolean indicating whether the data sample contains a change in the value of the instance of is only used
to communicate a change in the instance status, e.g., a change in the liveliness of the instance.
In the latter case, the data sample should be dismissed as all the relevant information is in the
data members of :class:`SampleInfo`.


sample_identity
---------------

The sample identity.
This is an extension for RPC.


related_sample_identity
-----------------------

The related sample identity.
This is an extension for RPC.





