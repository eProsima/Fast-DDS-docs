.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _domainparticipant_discovery_callbacks:

DomainParticipantListener Discovery Callbacks
----------------------------------------------

As stated in :ref:`dds_layer_domainParticipantListener`, the |DomainParticipantListener-api| is an abstract class
defining the callbacks that will be triggered in response to state changes on the DomainParticipant.
Fast DDS defines three callbacks attached to events that may occur during discovery:
|DomainParticipantListener::on_participant_discovery-api|,
|DomainParticipantListener::on_data_reader_discovery-api|,
|DomainParticipantListener::on_data_writer_discovery-api|.
Further information about the DomainParticipantListener is provided in the :ref:`dds_layer_domainParticipantListener`
section.
The following is an example of the implementation of DomainParticipantListener discovery callbacks.


.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DOMAINPARTICIPANTLISTENER-DISCOVERY-CALLBACKS
   :end-before: //!--

To use the previously implemented discovery callbacks in :class:`DiscoveryDomainParticipantListener` class, which
inherits from the DomainParticipantListener, an object of this class is created and registered as a listener
of the DomainParticipant.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //SET-DISCOVERY-CALLBACKS
   :end-before: //!--
   :dedent: 8

.. important::

   Read more about callbacks and its hierarchy :ref:`here<dds_layer_core_entity_commonchars_listener>`
