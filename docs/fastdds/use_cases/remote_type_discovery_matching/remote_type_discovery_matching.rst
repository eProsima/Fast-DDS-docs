.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-remote-type-discovery-and-matching:

Remote type discovery and endpoint matching
===========================================

This section explains how to create an endpoint using a remotely discovered data type that was previously unknown to the
local participant.

Prerequisites
-------------

This use case focuses on the strategy to follow in order to create an endpoint at runtime in a previously unknown topic
using the information provided by the remote endpoint discovery information.
Therefore, the prerequisites are:

.. TODO: third prerequiste will be obsolete once the DynamicTypes defined using the Dynamic Language Binding are
   able to be registered into TypeObjectRegistry.

* Two participants, A and B, running in different process (type information is shared within the same
  DomainParticipantFactory).
* Participant A must not know the data type registered in participant B.
* Participant B data type must be registered using the code generated by
  :ref:`eProsima Fast DDS-Gen<fastddsgen_intro>` without disabling the TypeObjectSupport code generation.
* Participant B must create an endpoint using the data type unknown by participant A.
* Participant A must be attached to a :ref:`dds_layer_domainParticipantListener`.

Remote type discovery
---------------------

Following the :ref:`participant discovery phase <disc_phases>`, the endpoint information is exchanged.
The appropriate |DomainParticipantListener::on_data_reader_discovery-api| or
|DomainParticipantListener::on_data_writer_discovery-api| callback is called, depending on the kind of endpoint
created on the remote participant.
The endpoint discovery callback provides access to the remotely discovered information including the |TypeInformation|.

Provided the :code:`TypeInformation`, |ITypeObjectRegistry-api| singleton can be queried for the corresponding
|TypeObject| calling |ITypeObjectRegistry::get_type_object| API.

Register remote type
--------------------

|DynamicTypeBuilderFactory-api| provides a specific API that given a |TypeObject| returns the corresponding
|DynamicTypeBuilder-api|: |DynamicTypeBuilderFactory::create_type_w_type_object|.
The |DynamicType-api| can then be obtained and registered using |DynamicPubSubType-api|.

Create local endpoint
---------------------

Once the remote type has been locally registered, a :ref:`Topic<dds_layer_topic_topic>` can be created within the
DomainParticipant and endpoints using this Topic might be also created.

.. note::

  Endpoint matching takes into consideration QoS consistency.
  Consequently, for the local endpoint to match, the remote QoS has to be taken into account.
  The remote endpoint discovery information provided by the discovery callback includes also this data.

Example
-------

The following snippet shows the previously explained steps:

.. literalinclude:: /../code/DDSCodeTester.cpp
     :language: c++
     :start-after: //!--REMOTE_TYPE_MATCHING
     :end-before: //!--
