.. _what_is_dds:

What is DDS?
------------

The `Data Distribution Service (DDS) <https://www.omg.org/spec/DDS/About-DDS/>`_
is a data-centric communication protocol used for distributed software
application communications.
It describes the communications Application Programming Interfaces (APIs) and Communication Semantics that enable
communication between data providers and data consumers.

Since it is a Data-Centric Publish Subscribe (DCPS) model, three key application entities are defined in its
implementation: publication entities, which define the information-generating objects and their properties;
subscription entities, which define the information-consuming objects and their properties; and configuration entities
that define the types of information that are transmitted as topics, and create the publisher and subscriber with
its Quality of Service (QoS) properties, ensuring the correct performance of the above entities.

DDS uses QoS to define the behavioral characteristics of DDS Entities. QoS are comprised of individual QoS policies
(objects of type deriving from QoSPolicy). These are described in :ref:`dds_layer_core_policy`.


The DCPS conceptual model
^^^^^^^^^^^^^^^^^^^^^^^^^

In the DCPS model, four basic elements are defined for the development of a system of communicating applications.

*   **Publisher**.
    It is the DCPS entity in charge of the creation and configuration of the **DataWriters** it implements.
    The **DataWriter** is the entity in charge of the actual publication of the messages.
    Each one will have an assigned **Topic** under which the messages are published.
    See :ref:`dds_layer_publisher` for further details.
*   **Subscriber**.
    It is the DCPS Entity in charge of receiving the data published under the topics to which it subscribes.
    It serves one or more **DataReader** objects, which are responsible for communicating the availability of new data
    to the application.
    See :ref:`dds_layer_subscriber` for further details.
*   **Topic**.
    It is the entity that binds publications and subscriptions.
    It is unique within a DDS domain.
    Through the **TopicDescription**, it allows the uniformity of data types of publications and subscriptions.
    See :ref:`dds_layer_topic` for further details.
*   **Domain**.
    This is the concept used to link all publishers and subscribers, belonging to one or more applications,
    which exchange data under different topics.
    These individual applications that participate in a domain are called **DomainParticipant**.
    The DDS Domain is identified by a domain ID.
    The DomainParticipant defines the domain ID to specify the DDS domain to which it belongs.
    Two DomainParticipants with different IDs are not aware of each other's presence in the network.
    Hence, several communication channels can be created.
    This is applied in scenarios where several DDS applications are involved, with their respective DomainParticipants
    communicating with each other, but these applications must not interfere.
    The **DomainParticipant** acts as a container for other DCPS Entities, acts as a factory for
    **Publisher**, **Subscriber** and **Topic** Entities, and provides administrative services in the domain.
    See :ref:`dds_layer_domain` for further details.

These elements are shown in the figure below.

.. figure:: /01-figures/fast_dds/getting_started/dds_domain.svg
    :align: center

    DCPS model entities in the DDS Domain.



