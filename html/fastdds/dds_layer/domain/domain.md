<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domain"></a>

<a id="domain"></a>

# Domain

A domain represents a separate communication plane.
It creates a logical separation among the Entities that share a common communication infrastructure.
Conceptually, it can be seen as a *virtual network* linking all applications running on the same domain
and isolating them from applications running on different domains.
This way, several independent distributed applications can coexist in the same physical network without interfering,
or even being aware of each other.

Every domain has a unique identifier, called domainId, that is implemented as a `uint32` value.
Applications that share this domainId belong to the same domain and will be able to communicate.

For an application to be added to a domain, it must create an instance of
[DomainParticipant](domainParticipant/domainParticipant.md#dds-layer-domainparticipant) with the appropriate domainId.
Instances of DomainParticipant are created through the
[DomainParticipantFactory](domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) singleton.

[Partitions](domainParticipant/partition.md#partitions) introduce another entity isolation level within the domain.
While DomainParticipant will be able to communicate with each other if they
are in the same domain, it is still possible to isolate their [Publishers](../publisher/publisher/publisher.md#dds-layer-publisher-publisher) and
[Subscribers](../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber) assigning them to different [Partitions](domainParticipant/partition.md#partitions).

![image](01-figures/domain_class_diagram.svg)

*Domain class diagram*

* [DomainParticipant](domainParticipant/domainParticipant.md)
  * [DomainParticipantQos](domainParticipant/domainParticipant.md#domainparticipantqos)
    * [Default DomainParticipantQos](domainParticipant/domainParticipant.md#default-domainparticipantqos)
  * [DomainParticipantExtendedQos](domainParticipant/domainParticipant.md#domainparticipantextendedqos)
* [DomainParticipantListener](domainParticipantListener/domainParticipantListener.md)
* [CongestionControlListener <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](domainParticipantListener/congestion_control_listener.md)
  * [Callbacks](domainParticipantListener/congestion_control_listener.md#callbacks)
  * [Example](domainParticipantListener/congestion_control_listener.md#example)
* [DomainParticipantFactory](domainParticipantFactory/domainParticipantFactory.md)
  * [DomainParticipantFactoryQos](domainParticipantFactory/domainParticipantFactory.md#domainparticipantfactoryqos)
  * [Loading profiles from an XML file](domainParticipantFactory/domainParticipantFactory.md#loading-profiles-from-an-xml-file)
* [Creating a DomainParticipant](domainParticipant/createDomainParticipant.md)
  * [Profile based creation of a DomainParticipant](domainParticipant/createDomainParticipant.md#profile-based-creation-of-a-domainparticipant)
  * [Default profile DomainParticipant creation](domainParticipant/createDomainParticipant.md#default-profile-domainparticipant-creation)
  * [Deleting a DomainParticipant](domainParticipant/createDomainParticipant.md#deleting-a-domainparticipant)
* [Partitions](domainParticipant/partition.md)
  * [Wildcards in Partitions](domainParticipant/partition.md#wildcards-in-partitions)
  * [Full example](domainParticipant/partition.md#full-example)
