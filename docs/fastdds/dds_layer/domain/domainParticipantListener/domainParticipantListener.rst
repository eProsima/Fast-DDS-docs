.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipantListener:

DomainParticipantListener
=========================

|DomainParticipantListener-api| is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_domainParticipant`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

DomainParticipantListener inherits from :ref:`dds_layer_topic_topicListener`,
:ref:`dds_layer_publisher_publisherListener`, and :ref:`dds_layer_subscriber_subscriberListener`.
Therefore, it has the ability to react to every kind of event that is
reported to any of its attached Entities.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that DomainParticipantListener inherits from other Listeners will only be called
if no other Entity was able to handle the event,
either because it has no Listener attached,
or because the callback is disabled by the |StatusMask-api| on the Entity.


Additionally, DomainParticipantListener adds the following non-standard callbacks:

* |DomainParticipantListener::on_participant_discovery-api|: A new DomainParticipant is discovered in the same domain,
  a previously known DomainParticipant has been removed, or some DomainParticipant
  has changed its QoS.
  This method provides an overload with an additional boolean output parameter so a discovery callback can tell the middleware if a newly discovered participant has to be ignored via the use of the |DomainParticipant::ignore_participant-api|.
  This overload should be used when there is a need to ignore participants inside the discovery callback, since calling |DomainParticipant::ignore_participant-api| inside the listener might deadlock.
  If both callbacks are implemented, the discovery callback with the ``should_be_ignored`` boolean flag takes precedence.
  The second discovery callback is only executed if the discovered DomainParticipant is not ignored in the first callback (``should_be_ignored`` parameter returns ``false``).

* |DomainParticipantListener::on_data_reader_discovery-api|: A new :ref:`dds_layer_subscriber_dataReader` is discovered in the same domain,
  a previously known DataReader has been removed, or some DataReader has changed its QoS.

* |DomainParticipantListener::on_data_writer_discovery-api|: A new :ref:`dds_layer_publisher_dataWriter` is discovered in the same domain,
  a previously known DataWriter has been removed, or some DataWriter has changed its QoS.

* |DomainParticipantListener::onParticipantAuthentication-api|: Informs about the result of the authentication process
  of a remote DomainParticipant (either on failure or success).

.. important::

   For more information about callbacks and its hierarchy, please refer to
   :ref:`dds_layer_core_entity_commonchars_listener`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DOMAINPARTICIPANT_LISTENER_SPECIALIZATION
   :end-before: //!


