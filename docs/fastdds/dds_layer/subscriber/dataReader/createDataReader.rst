.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_datareader_creation:

Creating a DataReader
=====================

A :ref:`dds_layer_subscriber_dataReader` always belongs to a :ref:`dds_layer_subscriber_subscriber`.
Creation of a DataReader is done with the |Subscriber::create_datareader-api| member function on the
Subscriber instance, that acts as a factory for the DataReader.

Mandatory arguments are:

 * A :ref:`dds_layer_topic_topic` bound to the data type that will be transmitted.

 * The :ref:`dds_layer_subscriber_dataReaderQos` describing the behavior of the DataReader.
   If the provided value is :code:`DATAREADER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDataReaderQos` is used.
<<<<<<< HEAD
=======
   If the provided value is :code:`DATAREADER_QOS_USE_TOPIC_QOS`,
   the values of the default QoS and the provided TopicQoS are used, whereby any policy
   that is set on the TopicQoS overrides the corresponding policy on the default QoS.
>>>>>>> 0a46fe1 (Fixed various typos and unified some formats in documentation (#996))

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_subscriber_dataReaderListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DataReader.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   DataReaderListener.
   By default all events are enabled.

|Subscriber::create_datareader-api| will return a null pointer if there was an error during the operation, e.g.
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

Instead of using a DataReaderQos, the name of a profile
can be used to create a DataReader with the |Subscriber::create_datareader_with_profile-api|
member function on the Subscriber instance.

Mandatory arguments are:

 * A Topic bound to the data type that will be transmitted.

 * A string with the name that identifies the DataReader.

Optional arguments are:

 * A Listener derived from DataReaderListener, implementing the callbacks
   that will be triggered in response to events and state changes on the DataReader.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   DataReaderListener.
   By default all events are enabled.

|Subscriber::create_datareader_with_profile-api| will return a null pointer if there was an error during the operation,
e.g. if the provided QoS is not compatible or is not supported.
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

A DataReader can be deleted with the |Subscriber::delete_datareader-api| member function on the
:ref:`dds_layer_subscriber_subscriber` instance where the DataReader was created.

.. note::

   A DataReader can only be deleted if all Entities belonging to the DataReader
   (QueryConditions) have already been deleted.
   Otherwise, the function will issue an error and the DataReader will not be deleted.
   This can be performed by using the |DataReader::delete_contained_entities-api| member function of the
   :ref:`dds_layer_subscriber_dataReader`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DATAREADER
   :end-before: //!
   :dedent: 8


