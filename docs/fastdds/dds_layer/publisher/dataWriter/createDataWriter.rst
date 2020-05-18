.. _dds_layer_publisher_datawriter_creation:

Creating a DataWriter
=====================

A :ref:`dds_layer_publisher_dataWriter` always belongs to a :ref:`dds_layer_publisher_publisher`.
Creation of a :ref:`dds_layer_publisher_dataWriter` is done with the :func:`create_datawriter` member function on the
:ref:`dds_layer_publisher_publisher` instance, that acts as a factory for the :ref:`dds_layer_publisher_dataWriter`.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * The :ref:`dds_layer_publisher_dataWriterQos` describing the behavior of the :ref:`dds_layer_publisher_dataWriter`.
   If the provided value is :class:`DATAWRITER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDataWriterQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_publisher_dataWriterListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_dataWriter`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_dataWriterListener`.
   By default all events are enabled.

:func:`create_datawriter` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DATAWRITER
   :end-before: //!
   :dedent: 8


.. _dds_layer_publisher_datawriter_creation_profile:

Profile based creation of a DataWriter
--------------------------------------

Instead of using a :ref:`dds_layer_publisher_dataWriterQos`, the name of a profile
can be used to create a :ref:`dds_layer_publisher_dataWriter` with the :func:`create_datawriter_with_profile`
member function on the :ref:`dds_layer_publisher_publisher` instance.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * A string with the name that identifies the :ref:`dds_layer_publisher_dataWriter`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_publisher_dataWriterListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_dataWriter`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_dataWriterListener`.
   By default all events are enabled.

:func:`create_datawriter_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DATAWRITER
   :end-before: //!
   :dedent: 8


.. _dds_layer_publisher_datawriter_deletion:

Deleting a DataWriter
---------------------

A :ref:`dds_layer_publisher_dataWriter` can be deleted with the :func:`delete_datawriter` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_publisher_dataWriter` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DATAWRITER
   :end-before: //!
   :dedent: 8


