.. _dds_layer_domain:

Domain
======

.. image:: /01-figures/domain_class_diagram.svg

A domain represents a separate communication plane.
It creates a logical separation among the Entities that share a common communication infrastructure.
Conceptually, it can be seen as a *virtual network* linking all applications running on the same domain
and isolating them from applications running on different domains.
This way, several independent distributed applications can coexist in the same physical network without interfering,
or even being aware of each other.

Every domain has a unique identifier, called domainId, that is implemented as a ``uint32`` value.
Applications that share this domainId belong to the same domain and will be able to communicate.

For an application to be added to a domain, it must create an instance of
:ref:`dds_layer_domainParticipant` with the appropriate domainId.
Instances of :ref:`dds_layer_domainParticipant` are created through the
:ref:`dds_layer_domainParticipantFactory` singleton.

.. figure:: /01-figures/domain_class_diagram.svg
    :align: center

    Domain class diagram
.. toctree::

    /fastdds/dds_layer/domain/domainParticipant/domainParticipant
    /fastdds/dds_layer/domain/domainParticipantListener/domainParticipantListener
    /fastdds/dds_layer/domain/domainParticipantFactory/domainParticipantFactory
    /fastdds/dds_layer/domain/domainParticipant/createDomainParticipant

