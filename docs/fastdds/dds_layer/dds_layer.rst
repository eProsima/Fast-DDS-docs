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
  It contains the :ref:`api_pim_domainparticipant` class that acts as an entry-point of the Service,
  as well as a factory for many of the classes. The :class:`DomainParticipant` also acts as a container for the other
  objects that make up the Service.

* :ref:`dds_layer_publisher`:
  It describes the classes used on the publication side, including :ref:`api_pim_publisher_class` and
  :ref:`api_pim_datawriter` classes, as well as the :ref:`api_pim_publisherlistener` and
  :ref:`api_pim_datawriterlistener` interfaces.

* :ref:`dds_layer_subscriber`:
  It describes the classes used on the subscription side, including :ref:`api_pim_subscriber_class` and
  :ref:`api_pim_datareader` classes, as well as the :ref:`api_pim_subscriberlistener`
  and :ref:`api_pim_datareaderlistener` interfaces.

* :ref:`dds_layer_topic`:
  It describes the classes used to define communication topics and data types, including :ref:`api_pim_topic_class`
  and :ref:`api_pim_topicdescription` classes, as well as :ref:`api_pim_typesupport`, and the
  :ref:`api_pim_topiclistener` interface.


.. toctree::
   :maxdepth: 2

   /fastdds/dds_layer/core/core
   /fastdds/dds_layer/domain/domain
   /fastdds/dds_layer/publisher/publisher
   /fastdds/dds_layer/subscriber/subscriber
   /fastdds/dds_layer/topic/topic
