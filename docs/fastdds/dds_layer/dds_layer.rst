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
  It contains the :ref:`dds_layer_domainParticipant` class that acts as an entry-point of the Service,
  as well as a factory for many of the classes. The :class:`DomainParticipant` also acts as a container for the other
  objects that make up the Service.

* :ref:`dds_layer_publisher`:
  It contains the :ref:`dds_layer_publisher_publisher` and :ref:`dds_layer_publisher_dataWriter` classes as well as
  the :ref:`api_pim_publisherlistener` and :ref:`dds_layer_publisher_dataWriterListener` interfaces, and more
  generally, all that is needed on the publication side.

* :ref:`dds_layer_subscriber`:
  It contains the :ref:`dds_layer_subscriber_subscriber`, an the :ref:`dds_layer_subscriber_dataReader`, as well as the
  :ref:`dds_layer_subscriber_subscriberListener` and :ref:`dds_layer_subscriber_dataReaderListener` interfaces, and more
  generally, all that is needed on the subscription side.

* :ref:`dds_layer_topic`:
  It contains the :ref:`dds_layer_topic_topic` class, the :ref:`dds_layer_topic_topicDescription`, the
  :ref:`dds_layer_topic_typeSupport`, the :ref:`dds_layer_topic_topicListener` interface, and more generally, all that
  is needed by the application to define Topic objects and attach QoS policies to them.

.. toctree::

   /fastdds/dds_layer/core/core
   /fastdds/dds_layer/domain/domain
   /fastdds/dds_layer/publisher/publisher
   /fastdds/dds_layer/subscriber/subscriber
   /fastdds/dds_layer/topic/topic
