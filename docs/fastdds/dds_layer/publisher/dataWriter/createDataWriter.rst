.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_datawriter_creation:

Creating a DataWriter
=====================

A :ref:`dds_layer_publisher_dataWriter` always belongs to a :ref:`dds_layer_publisher_publisher`.
Creation of a DataWriter is done with the |Publisher::create_datawriter-api| member function on the
Publisher instance, that acts as a factory for the DataWriter.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * The :ref:`dds_layer_publisher_dataWriterQos` describing the behavior of the DataWriter.
   If the provided value is :code:`DATAWRITER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDataWriterQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_publisher_dataWriterListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DataWriter.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   DataWriterListener.
   By default all events are enabled.

|Publisher::create_datawriter-api| will return a null pointer if there was an error during the operation, e.g.
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

Instead of using a DataWriterQos, the name of a profile
can be used to create a DataWriter with the |Publisher::create_datawriter_with_profile-api|
member function on the Publisher instance.

Mandatory arguments are:

 * A Topic bound to the data type that will be transmitted.

 * A string with the name that identifies the DataWriter.

Optional arguments are:

 * A Listener derived from DataWriterListener, implementing the callbacks
   that will be triggered in response to events and state changes on the DataWriter.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   DataWriterListener.
   By default all events are enabled.

|Publisher::create_datawriter_with_profile-api| will return a null pointer if there was an error during the
operation, e.g. if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DATAWRITER
   :end-before: //!
   :dedent: 8

.. _dds_layer_publisher_datawriter_with_payload_pool_creation:

Creating a DataWriter with a custom PayloadPool
-----------------------------------------------

A custom :ref:`PayloadPool<rtps_layer_custom_payload_pool>` can be passed as an argument during the creation of a
:ref:`dds_layer_publisher_dataWriter`.
This allows for customizing the management of the information exchanged between DataWriters and DataReaders.
The same configuration can be set in the
:ref:`opposite endpoint<dds_layer_subscriber_datareader_with_payload_pool_creation>`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PAYLOAD_POOL_DATAWRITER
   :end-before: //!
   :dedent: 8

This configuration can be performed also in the :ref:`RTPS layer<rtps_layer_custom_payload_pool>`.
The :ref:`customization example<rtps_layer_payload_pool_example>` applies both layers.

.. _dds_layer_publisher_datawriter_deletion:

Deleting a DataWriter
---------------------

A DataWriter can be deleted with the |Publisher::delete_datawriter-api| member function on the
:ref:`dds_layer_publisher_publisher` instance where the DataWriter was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DATAWRITER
   :end-before: //!
   :dedent: 8


