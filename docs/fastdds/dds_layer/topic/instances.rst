.. include:: ../../../03-exports/aliases-api.include

.. _dds_layer_topic_instances:

Topics, keys and instances
==========================

Even though, by definition, a Topic corresponds to a single data type, it is possible that several topic instances may
refer to the same data type.
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

For example, instances could be used to track commercial flights.
The key identifying the instances could be the airline name and the flight number, while the sample data would be the
location of each flight being tracked at any given time.
Instead of creating a DataReader and Topic each time a flight takes off, the same control tower DataReader could manage
the information published by each plane DataWriter in the same Topic using this DDS standard feature.
Once the flight has landed, the instance can be disposed (no updates are expected); and if the flight is returned to the
hangar, the instance is unregistered.

The following IDL may define the data model of this specific example:

.. code-block:: idl

    struct FlightPosition
    {
        // Unique ID: airline name
        @key
        string<256> airline_name;

        // Unique ID: flight number
        int flight_number;

        // Coordinates
        double latitude;
        double longitude;
        double altitude;
    };

Then, writing to an specific instance will be done as below:

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //INSTANCES
   :end-before: //!

Instances could also be used as a relational database.
The instance key is analogous to the primary key in the database (unique identifier of something within the data).
In this scenario, writing a new sample to the instance is similar to insert/update the database; and disposing the
instance is like deleting the data corresponding to the given primary key from the database.
