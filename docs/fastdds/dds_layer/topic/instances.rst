.. include:: ../../../03-exports/aliases-api.include

.. _dds_layer_topic_instances:

Topics, keys and instances
==========================

By definition, a Topic is linked to a single data type, so each data sample related to a Topic could be understood as an
update on the information described by the data type.
However, it is possible to include a logical separation and have, within the same Topic, several instances referring to
the same data type.
Thus, the received data sample will be an update for a specific instance of that Topic.
Therefore, a Topic identifies data of a single type, ranging from one single instance to a whole collection of instances
of that given type, as shown in the figure below.

.. figure:: /01-figures/instances.png
    :align: center

The different instances gathered under the same topic are distinguishable by means of one or more data fields that form
the key to that data set.
The key description has to be indicated to the middleware.
The rule is simple: different data values with the same key value represent successive data samples for the same
instance, while different data values with different keys represent different topic instances.
If no key is provided, the data set associated with the Topic is restricted to a single instance.
Please refer to :ref:`dds_layer_topic_keyed_data_types` for more information about how to set the key in
*eProsima Fast DDS*.

.. _dds_layer_topic_instance_advantages:

Instance advantages
-------------------

The advantage of using instances instead of creating a new :ref:`dds_layer_publisher_dataWriter`,
:ref:`dds_layer_subscriber_dataReader`, and :ref:`dds_layer_topic_topic` is that the corresponding entity is already
created and discovered.
Consequently, there is less memory usage, and no new discovery (with the related metatraffic involved as explained in
:ref:`discovery`) is necessary.
Another advantage is that several QoS are applied per topic instance; e.g. the :ref:`historyqospolicy` is kept for each
instance in the DataWriter.
Thus, instances could be tuned to a wide range of applications.

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

.. _dds_layer_topic_instance_examples:

Practical applications
----------------------

This section provides a couple of examples to help clarify the use of DDS instances.

Commercial flights tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Airspace and the air traffic going through it are typically managed by the air traffic controllers that are in charge of
organizing the air traffic, preventing collisions, and providing information.
In this scenario, each air traffic control center takes responsibility for a specific flight area and delivers the data
to the airspace traffic management system, which unifies the flight information.

Any time an air traffic control center discovers a plane coming into its controlled flight zone, tracking information
about that specific flight is notified to the airspace traffic management center.
Such a flow of information could be implemented by means of DDS by creating a specific Topic where the information
related to the flight location is published.
In that case, the management center would be required to create, if not existing previously, the corresponding Topic and
DataReader to have access to the flight information, with the corresponding memory consumption and discovery metatraffic
required.
On the other hand, a cleverer implementation could leverage topic instances to relay the information from the local
air traffic control centers to the airspace traffic management center.
The topic instances might be identified using the airline name and the flight number (i.e. `IBERIA` `1234`) as Topic
instance key.
The sample data being relayed would be the location of each flight being tracked at any given time.
The following IDL defines the data described model:

.. code-block:: idl

    struct FlightPosition
    {
        // Unique ID: airline name
        @key string<256> airline_name;

        // Unique ID: flight number
        @key short flight_number;

        // Coordinates
        double latitude;
        double longitude;
        double altitude;
    };

Once a new flight is discovered by a control center, the corresponding instance is registered into the system:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //REGISTER-INSTANCE
    :end-before: //!

|DataWriter::register_instance-api| returns an |InstanceHandle_t-api| which can be used to efficiently call the next
operations (i.e. |DataWriter::write-api|, |DataWriter::dispose-api|, or |DataWriter::unregister_instance-api|) over
the instance.
The returned ``InstanceHandle_t`` contains the instance keyhash so it does not have to be recalculated again from the
data sample.
In case of following this approach, the application must take charge of mapping the instance handles to the
corresponding instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //WRITE-REGISTERED-INSTANCE
    :end-before: //!

On the other hand, the user application could directly call the DataWriter instance operations with a ``NIL`` instance
handle.
In this case, the instance handle would be calculated every time an operation is done over the instance, which can be
time consuming depending on the specific data type being used.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //WRITE-NON-REGISTERED-INSTANCE
    :end-before: //!

.. warning::

  The correct management of the instance handles in the user application is paramount.
  Otherwise, a sample corresponding to a different instance could wrongly update the instance which handle the user has
  passed to the operation (if a non ``NIL`` instance is provided, the instance handle is not recalculated, trusting
  that the one passed by the user is the correct one).
  The following code updates the first instance of this example with the information coming from the second instance.

  .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :dedent: 12
      :start-after: //WRONG-INSTANCE-UPDATE
      :end-before: //!

Once the plane leaves the controlled area, the air traffic control center may unregister the instance.
Unregistering implies that the DataWriter for this specific center has no more information about the unregistered
instance, and in this way the matched DataReaders in the management center are notified.
The flight is still in the air but out of scope of this particular DataWriter.
The instance is alive but no longer tracked by this center.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //UNREGISTER-INSTANCE
    :end-before: //!

Finally, when the flight lands, the instance may be disposed.
This means, in this specific example, that as far as the DataWriter knows, the instance no longer exists and should be
considered not alive.
With this operation, the DataWriter conveys this information to the matched DataReaders.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //DISPOSE-INSTANCE
    :end-before: //!

From the management center point of view, the samples are read using the same DataReader subscribed to the Topic where
the instances are being published.
However, |SampleInfo::valid_data-api| must be checked to ensure that the sample received contains a data sample.
Otherwise, a change of the instance state is being notified.
:ref:`dds_layer_topic_instance_lifecycle` contains a diagram showing the instance statechart.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 12
    :start-after: //READING-INSTANCE
    :end-before: //!

Relational databases
^^^^^^^^^^^^^^^^^^^^

Consider now that the air traffic management center wants to keep a database with the flights being tracked.
Using DDS instances, maintaining a relational database is almost direct.
The instance key (unique identifier of the instance) is analogous to the primary key of the database.
Thus, the airspace traffic management center can keep the latest update for each instance in a table like the one below:

.. list-table::
    :header-rows: 1
    :align: left

    * - Instance handle [PK]
      - Data
    * - 1
      - Position1
    * - 2
      - Position2
    * - 3
      - Position3
    * - 4
      - Position4
    * - 5
      - Position5

In this case, every time a new sample is received, the corresponding instance entry in the database will be updated with
the latest known location.
Disposing the instance may translate in erasing the corresponding data from the database.
In this scenario, registering and unregistering the instances do not reflect in the database.
A DataWriter communicating that it is going to be publishing data about a specific instance is of no interest to the
database until a new data is received and then an insert is directly done with the new discovered instance.

Historical data can also be stored in the relational database, even though depending on the use case, a time series
database might be considered to improve efficiency.
In the scenario being considered, the sample timestamp could be used, besides the instance handle, as primary key to
be able to access the historical tracking data of an specific flight.

.. list-table::
    :header-rows: 1
    :align: left

    * - Instance handle [PK]
      - Source Timestamp [PK]
      - Data
    * - 1
      - 1
      - Position1
    * - 2
      - 1
      - Position2
    * - 1
      - 2
      - Position3
    * - 1
      - 3
      - Position4
    * - 2
      - 2
      - Position5

In this case, looking for a specific instance handle would return the flight tracking information:

.. list-table::
    :header-rows: 1
    :align: left

    * - Instance handle [Fixed]
      - Source Timestamp
      - Data
    * - 1
      - 1
      - Position1
    * - 1
      - 2
      - Position3
    * - 1
      - 3
      - Position4

Whereas looking for a specific timestamp would allow to have a picture of the different flight locations at a specific
time:

.. list-table::
    :header-rows: 1
    :align: left

    * - Instance handle
      - Source Timestamp [Fixed]
      - Data
    * - 1
      - 2
      - Position3
    * - 2
      - 2
      - Position5
