.. _dds_layer_publisher_write:

Publishing data
===============

The user informs of a change in the value of a data instance with the :func:`write` member function on the
:ref:`dds_layer_publisher_dataWriter`. This change will then be communicated to every
:ref:`dds_layer_subscriber_subscriber` matched with the :ref:`dds_layer_publisher_dataWriter`.
As a side effect, this operation asserts liveliness on the :ref:`dds_layer_publisher_dataWriter` itself,
the :ref:`dds_layer_publisher_publisher` and the :ref:`dds_layer_domainParticipant`.

The function takes two arguments:

 * A pointer to the data instance with the new values.
 * The handler to the instance.

An empty (i.e., default constructed :func:`InstanceHandle_t`) instance handler can be used for the argument handle.
This indicates that the identity of the instance should be automatically deduced from the key of the
instance data.
Alternatively, the member function :func:`write` is overloaded to take only the pointer to the data instance,
which will always deduced the identity from the key of the instance data.

If the handle is not empty, then it must correspond to the value obtained with the :func:`getKey` of the
:class:`TypeSupport` instance.
Otherwise the write function will fail with ``RETCODE_PRECONDITION_NOT_MET``.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_WRITE
   :end-before: //!
   :dedent: 8


.. _dds_layer_publisher_write_blocking:

Blocking of the write operation
-------------------------------

If the reliability kind is set to ``RELIABLE`` on the :ref:`dds_layer_publisher_dataWriterQos`,
the :func:`write` operation may block.
Specifically, if the limits specified in the configured resource limits have been reached, the
:func:`write` operation will block waiting for space to become available.
Under these circumstances, the reliability ``max_blocking_time`` configures the maximum time
the write operation may block waiting.
If ``max_blocking_time`` elapses before the :ref:`dds_layer_publisher_dataWriter` is able to store
the modification without exceeding the limits, the write operation will fail and return ``TIMEOUT``.



