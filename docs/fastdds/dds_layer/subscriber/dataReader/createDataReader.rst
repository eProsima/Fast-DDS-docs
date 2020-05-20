.. _dds_layer_subscriber_datareader_creation:

Creating a DataReader
=====================

A :ref:`dds_layer_subscriber_dataReader` always belongs to a :ref:`dds_layer_subscriber_subscriber`.
Creation of a :ref:`dds_layer_subscriber_dataReader` is done with the :func:`create_datareader` member function on the
:ref:`dds_layer_subscriber_subscriber` instance, that acts as a factory for the :ref:`dds_layer_subscriber_dataReader`.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * The :ref:`dds_layer_subscriber_dataReaderQos` describing the behavior of the :ref:`dds_layer_subscriber_dataReader`.
   If the provided value is :class:`DATAREADER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDataReaderQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_subscriber_dataReaderListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_subscriber_dataReader`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_subscriber_dataReaderListener`.
   By default all events are enabled.

:func:`create_datareader` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DATAREADER
   :end-before: //!
   :dedent: 8


.. _dds_layer_subscriber_datareader_creation_profile:

Profile based creation of a DataReader
--------------------------------------

Instead of using a :ref:`dds_layer_subscriber_dataReaderQos`, the name of a profile
can be used to create a :ref:`dds_layer_subscriber_dataReader` with the :func:`create_datareader_with_profile`
member function on the :ref:`dds_layer_subscriber_subscriber` instance.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * A string with the name that identifies the :ref:`dds_layer_subscriber_dataReader`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_subscriber_dataReaderListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_subscriber_dataReader`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_subscriber_dataReaderListener`.
   By default all events are enabled.

:func:`create_datareader_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DATAREADER
   :end-before: //!
   :dedent: 8


.. _dds_layer_subscriber_datareader_deletion:

Deleting a DataReader
---------------------

A :ref:`dds_layer_subscriber_dataReader` can be deleted with the :func:`delete_datareader` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_subscriber_dataReader` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DATAREADER
   :end-before: //!
   :dedent: 8


