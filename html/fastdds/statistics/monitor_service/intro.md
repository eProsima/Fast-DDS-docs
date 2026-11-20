<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="monitor-intro"></a>

<a id="introduction"></a>

# Introduction

The `Monitor Service` targets any application implementing the subscription side of
the [Monitor Service Status Topic](monitor_service_topics.md#monitor-service-topics),
giving the possibility of retrieving the [Monitoring Information](#monitor-service-keywords)
of the local entities (incompatible QoS, deadlines missed,
active connections,…).

<a id="monitor-service-keywords"></a>

<a id="keywords"></a>

## Keywords

* **Proxy**: An entity that acts on behalf of another entity.
* **Proxy Data**: The way in which a Proxy can be described.
* **Monitoring Information**: The collection of different sources of information and statuses of an entity,
  including: the Proxy Data, incompatible QoS, connections, liveliness, deadlines missed, inconsistent
  topics and lost sample status.

<a id="description"></a>

## Description

Enabling the service makes each [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) publish its local entities, each one with its related
`Monitoring Information`.

The [Monitor Service](monitor_service.md#monitor-service) is disabled by default, as it may entail a performance cost.
Further information on the [Monitor Service](monitor_service.md#monitor-service) topics and how to configure it is described
in the following sections.

The [Monitor Service](monitor_service.md#monitor-service) is available in both the [DDS Layer](../../dds_layer/dds_layer.md#dds-layer) and [RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer).

<a id="monitor-service-in-rtps-note"></a>

#### NOTE
If the service is activated within a [RTPS](../../rtps_layer/rtps_layer.md#rtps-layer) context, not all the `Monitoring Information`
may be published by the service.

<a id="use-cases"></a>

## Use Cases

The [Monitor Service](monitor_service.md#monitor-service) can be particularly useful in the following scenarios:

* Collecting the `Monitoring Information` of any local entity of a remote [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
  in order to extend the default discovery information (see [Discovery](../../discovery/discovery.md#discovery)) about it.
* Troubleshooting issues regarding discovery or entity-matching, leveraging the information
  of the current locators in use, for example.
* Recreating an entity graph of a certain domain given that all participants are able to discover each other.
