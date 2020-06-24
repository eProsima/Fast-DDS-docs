.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _dds_layer:

DDS Layer
=========

*eProsima Fast DDS* exposes two different APIs to interact with the communication service at different levels.
The main API is the Data Distribution Service (DDS) Data-Centric Publish-Subscribe (DCPS) Platform Independent Model
(PIM) API, or :ref:`api_pim_dds_dcps_pim` for short, which is defined by the
`Data Distribution Service (DDS) version 1.4 specification <https://www.omg.org/spec/DDS/1.4>`_, to which *Fast DDS*
complies.
This section is devoted to explain the main characteristics and modes-of-use of this API under *Fast DDS*, providing an
in depth explanation of the five modules into which it is divided:

* :ref:`dds_layer_core`:
  It defines the abstract classes and interfaces that are refined by the other modules.
  It also provides the Quality of Service (QoS) definitions, as well as support for the notification-based interaction
  style with the middleware.

* :ref:`dds_layer_domain`:
  It contains the |DomainParticipant-api| class that acts as an entry-point of the Service,
  as well as a factory for many of the classes. The |DomainParticipant-api| also acts as a container for the other
  objects that make up the Service.

* :ref:`dds_layer_publisher`:
  It describes the classes used on the publication side, including |Publisher-api| and
  |DataWriter-api| classes, as well as the |PublisherListener-api| and
  |DataWriterListener-api| interfaces.

* :ref:`dds_layer_subscriber`:
  It describes the classes used on the subscription side, including |Subscriber-api| and
  |DataReader-api| classes, as well as the |SubscriberListener-api|
  and |DataReaderListener-api| interfaces.

* :ref:`dds_layer_topic`:
  It describes the classes used to define communication topics and data types, including |Topic-api|
  and |TopicDescription-api| classes, as well as |TypeSupport-api|, and the
  |TopicListener-api| interface.


.. toctree::
   :maxdepth: 2

   /fastdds/dds_layer/core/core
   /fastdds/dds_layer/domain/domain
   /fastdds/dds_layer/publisher/publisher
   /fastdds/dds_layer/subscriber/subscriber
   /fastdds/dds_layer/topic/topic
