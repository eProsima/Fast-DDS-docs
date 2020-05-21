.. _dds_layer_core_entity:

Entity
======

Entity is the abstract base class for all the DDS entities, meaning a DSCP object that supports QoS policies,
a listener, and statuses.
But there is a restriction, a Domain Participant cannot contain other Domain Participants.
The Domain Entity exists in order to avoid that problem, as it inherits from Entity and acts as the base class
for all DDS entities except for the Domain Participant.

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

Common Entities Characteristics
-------------------------------

Each entity is identified by a unique ID, which is shared between the DDS entity and its corresponding RTPS entity
if it exists.
That ID is stored on an Instance Handle object declared on Entity base class, which can be accessed using the getter
function.

All the entities can be created either enabled or not enabled.
By default, the factories are configured to create the
entities enabled, but it can be changed using the :ref:`EntityFactoryQosPolicy <entityfactoryqospolicy>` on enabled
factories.
A disabled factory creates disabled entities regardless of its QoS.

A disabled entity has its operations limited to the following ones:

- Set/Get the entity QoS policies
- Create/Delete subentities
- Get the statuses of the entity, even if they will not change
- Lookup operations

Any other function called in this state will return ``NOT_ENABLED``.
