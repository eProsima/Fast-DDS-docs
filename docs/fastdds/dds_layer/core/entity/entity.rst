.. _dds_layer_core_entity:

Entity
======

Entity is the abstract base class for all the DDS entities, meaning a DSCP object that supports QoS policies,
a listener, and statuses.
But there is a restriction, a Domain Participant cannot contain other Domain Participants.
The Domain Entity exists in order to avoid that problem, as it inherits from Entity and acts as the base class
for all DDS entities except for the Domain Participant.

.. _dds_layer_core_entity_types:

Types of Entities
-----------------

- **Domain Participant**: This entity is the entry-point of the Service and acts as a factory for Publishers,
  Subscribers, and Topics.
  See :ref:`dds_layer_domainParticipant` for further details.

- **Publisher**: It acts as a factory that can create any number of DataWriters.
  See :ref:`dds_layer_publisher_publisher` for further details.

- **Subscriber**:  It acts as a factory that can create any number of DataReaders.
  See :ref:`dds_layer_subscriber_subscriber` for further details.

- **Topic**: This entity fits between the publication and subscription entities and acts as a channel.
  See :ref:`dds_layer_topic_topic` for further details.

- **DataWriter**: Is the object responsible for the data distribution.
  See :ref:`dds_layer_publisher_datawriter` for further details.

- **DataReader**: Is the object used to access the received data.
  See :ref:`dds_layer_subscriber_datareader` for further details.

The following figure shows the hierarchy between all DDS entities:

.. image:: /01-figures/entity_diagram.svg
    :align: center

.. _dds_layer_core_entity_commonchars:

Common Entity Characteristics
-----------------------------

All entity types share some characteristics that are common to the concept of an entity.

.. _dds_layer_core_entity_commonchars_identifier:

Entity Identifier
^^^^^^^^^^^^^^^^^
Each entity is identified by a unique ID, which is shared between the DDS entity and its corresponding RTPS entity
if it exists.
That ID is stored on an Instance Handle object declared on Entity base class, which can be accessed using the getter
function.

.. _dds_layer_core_entity_commonchars_qos:

Qos policy
^^^^^^^^^^
The behavior of each entity can be configured with a set of configuration policies.
For each entity type, there is a corresponding Quality of Service (QoS) class that groups all the policies that affect
said entity type.
Users can create instances of these QoS classes, modify the contained policies to their needs,
and use them to configure the entities, either during their creation or later with the :func:`set_qos` function
that every entity exposes.

See :ref:`dds_layer_core_policy` for a list of the available policies and their description.
The QoS classes and the policies they contain are explained in the documentation for each entity type.

.. _dds_layer_core_entity_commonchars_listener:

Listener
^^^^^^^^
A listener is an object with functions that an entity will call in response to events.
Therefore, the listener acts as an asynchronous notification system that allows the entity to notify the application
about the :ref:`dds_layer_core_entity_commonchars_status` changes in the entity.

All entity types define an abstract listener interface, which contains the callback functions that the entity will
trigger to communicate the :ref:`dds_layer_core_entity_commonchars_status` changes to the application.
Users can implement their own listeners inheriting from these interfaces and implementing the callbacks that are
are needed on their application.
Then they can link this listeners to each entity, either during their creation or later with the :func:`set_listener`
function that every entity exposes.

The listener interfaces that each entity type and their callbacks are explained in the documentation
for each entity type.

.. _dds_layer_core_entity_commonchars_status:

Status
^^^^^^
Each entity is associated with a set of status objects whose value represents the *communication status* of that entity.
The changes on these status values are the ones that trigger the invocation of the appropriate
:ref:`dds_layer_core_entity_commonchars_listener` listener callback to asynchronously inform the application.

See :ref:`dds_layer_core_status` for a list of all the status objects and a description of their content.
There you can also find which status applies to which entity type.

.. _dds_layer_core_entity_commonchars_enabling:

Enabling Entities
^^^^^^^^^^^^^^^^^
All the entities can be created either enabled or not enabled.
By default, the factories are configured to create the
entities enabled, but it can be changed using the :ref:`EntityFactoryQosPolicy <entityfactoryqospolicy>` on enabled
factories.
A disabled factory creates disabled entities regardless of its QoS.

A disabled entity has its operations limited to the following ones:

- Set/Get the entity :ref:`dds_layer_core_entity_commonchars_qos`
- Set/Get the entity :ref:`dds_layer_core_entity_commonchars_listener`
- Create/Delete subentities
- Get the :ref:`dds_layer_core_entity_commonchars_status` of the entity, even if they will not change
- Lookup operations

Any other function called in this state will return ``NOT_ENABLED``.
