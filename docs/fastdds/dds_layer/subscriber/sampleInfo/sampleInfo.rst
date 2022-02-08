.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_sampleInfo:

SampleInfo
==========

When a sample is retrieved from the :ref:`dds_layer_subscriber_dataReader`, in addition to the sample data,
a |SampleInfo-api| instance is returned.
This object contains additional information that complements the returned data value and helps on it interpretation.
For example, if the :ref:`dds_layer_subscriber_sampleInfo_validdata` value is ``false``, the
DataReader is not informing the application about a new value in the data instance,
but a change on its status, and the returned data value must be discarded.

Please, refer to the section :ref:`dds_layer_subscriber_accessreceived` for more information regarding how received
data can be accessed on the DataReader.

The following sections describe the data members of |SampleInfo-api|
and the meaning of each one in relation to the returned sample data.

* :ref:`dds_layer_subscriber_sampleInfo_samplestate`
* :ref:`dds_layer_subscriber_sampleInfo_viewstate`
* :ref:`dds_layer_subscriber_sampleInfo_instancestate`
* :ref:`dds_layer_subscriber_sampleInfo_disposedgenerationcount`
* :ref:`dds_layer_subscriber_sampleInfo_nowritersgenerationcount`
* :ref:`dds_layer_subscriber_sampleInfo_samplerank`
* :ref:`dds_layer_subscriber_sampleInfo_generationrank`
* :ref:`dds_layer_subscriber_sampleInfo_absolutegenerationrank`
* :ref:`dds_layer_subscriber_sampleInfo_sourcetimestamp`
* :ref:`dds_layer_subscriber_sampleInfo_instancehandle`
* :ref:`dds_layer_subscriber_sampleInfo_publicationhandle`
* :ref:`dds_layer_subscriber_sampleInfo_validdata`
* :ref:`dds_layer_subscriber_sampleInfo_sampleidentity`
* :ref:`dds_layer_subscriber_sampleInfo_relatedsampleidentity`


.. _dds_layer_subscriber_sampleInfo_samplestate:

sample_state
------------
|SampleInfo::sample_state-api| indicates whether or not the corresponding data sample has already
been read previously.
It can take one of these values:

* **READ**: This is the first time this data sample has been retrieved.
* **NOT_READ**: The data sample has already been *read* or *taken* previously.

.. _dds_layer_subscriber_sampleInfo_viewstate:

view_state
----------
|SampleInfo::view_state-api| indicates whether or not this is the very first sample
of this data instance that the DataReader retrieves.
It can take one of these values:

* **NEW**: This is the first time a sample of this instance is retrieved.
* **NOT_NEW**: Other samples of this instance have been retrieved previously.

.. _dds_layer_subscriber_sampleInfo_instancestate:

instance_state
--------------
|SampleInfo::instance_state-api| indicates whether the instance is currently in existence
or it has been disposed.
In the latter case, it also provides information about the reason for the disposal.
It can take one of these values:

* **ALIVE**: The instance is currently in existence.
* **NOT_ALIVE_DISPOSED**: A remote :ref:`dds_layer_publisher_dataWriter` disposed the instance.
* **NOT_ALIVE_NO_WRITERS**: The DataReader disposed the instance because no remote
  DataWriter that was publishing the instance is *alive*.

.. _dds_layer_subscriber_sampleInfo_disposedgenerationcount:

disposed_generation_count
-------------------------
|SampleInfo::disposed_generation_count-api| indicates the number of times the instance had become alive after it was
disposed.

.. _dds_layer_subscriber_sampleInfo_nowritersgenerationcount:

no_writers_generation_count
---------------------------
|SampleInfo::no_writers_generation_count-api| indicates the number of times the instance had become alive after it was
disposed as ``NOT_ALIVE_NO_WRITERS``.

.. _dds_layer_subscriber_sampleInfo_samplerank:

sample_rank
-----------
|SampleInfo::sample_rank-api| indicates the number of samples of the same instance that have been received after
this one.
For example, a value of ``5`` means that there are 5 newer samples available on the DataReader.

.. note::
   Currently the |SampleInfo::sample_rank-api| is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of *Fast DDS*.

.. _dds_layer_subscriber_sampleInfo_generationrank:

generation_rank
---------------
|SampleInfo::generation_rank-api| indicates the number of times the instance was disposed and become alive again
between the time the sample was received and the time the most recent sample of the same instance
that is still held in the collection was received.

.. note::
   Currently the |SampleInfo::generation_rank-api| is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of *Fast DDS*.

.. _dds_layer_subscriber_sampleInfo_absolutegenerationrank:

absolute_generation_rank
------------------------
|SampleInfo::absolute_generation_rank-api| indicates the number of times the instance was disposed and become alive
again between the time the sample was received and the time the most recent sample of the same instance
(which may not be in the collection) was received.

.. note::
   Currently the |SampleInfo::absolute_generation_rank-api| is not implemented, and its value is always set to ``0``.
   It will be implemented on a future release of *Fast DDS*.

.. _dds_layer_subscriber_sampleInfo_sourcetimestamp:

source_timestamp
----------------
|SampleInfo::source_timestamp-api| holds the time stamp provided by the DataWriter when
the sample was published.

.. _dds_layer_subscriber_sampleInfo_instancehandle:

instance_handle
---------------
|SampleInfo::instance_handle-api| handles of the local instance.

.. _dds_layer_subscriber_sampleInfo_publicationhandle:

publication_handle
------------------
|SampleInfo::publication_handle-api| handles of the DataWriter that published the data change.

.. _dds_layer_subscriber_sampleInfo_validdata:

valid_data
----------
|SampleInfo::valid_data-api| is a boolean that indicates whether the data sample contains a change in the value or not.
Samples with this value set to false are used to communicate a change in the instance status, e.g.,
a change in the liveliness of the instance.
In this case, the data sample should be dismissed as all the relevant information is in the
data members of SampleInfo.

.. _dds_layer_subscriber_sampleInfo_sampleidentity:

sample_identity
---------------
|SampleInfo::sample_identity-api| is an extension for requester-replier configuration.
It contains the DataWriter and the sequence number of the current message, and it is used
by the replier to fill the :ref:`dds_layer_subscriber_sampleInfo_relatedsampleidentity` when it sends the reply.

.. _dds_layer_subscriber_sampleInfo_relatedsampleidentity:

related_sample_identity
-----------------------
|SampleInfo::related_sample_identity-api| is an extension for requester-replier configuration.
On reply messages, it contains the :ref:`dds_layer_subscriber_sampleInfo_sampleidentity` of the related request message.
It is used by the requester to be able to link each reply to the appropriate request.





