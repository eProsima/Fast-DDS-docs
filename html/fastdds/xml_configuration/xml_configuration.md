<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xml-profiles"></a>

<a id="id1"></a>

# XML profiles

*eProsima Fast DDS* allows for loading XML configuration files, each one containing one or more XML profiles.
In addition to the API functions for loading user XML files, *Fast DDS* tries to locate and load several XML files
upon initialization.
*Fast DDS* offers the following options:

* Load an XML file named *DEFAULT_FASTDDS_PROFILES.xml* located in the current execution path.
* Load an XML file which location is defined using the environment variable `FASTDDS_DEFAULT_PROFILES_FILE`
  (see [FASTDDS_DEFAULT_PROFILES_FILE](../env_vars/env_vars.md#env-vars-fastdds-default-profiles-file)).
* Load the configuration parameters directly from the classes’ definitions without looking for the
  *DEFAULT_FASTDDS_PROFILES.xml* in the working directory (see [SKIP_DEFAULT_XML](../env_vars/env_vars.md#env-vars-skip-default-xml)).
* Load directly the XML as a string data buffer.

An XML profile is defined by a unique name that is used to reference the XML profile
during the creation of an [Entity](../dds_layer/core/entity/entity.md#dds-layer-core-entity), the [Transport](../transport/transport.md#comm-transports-configuration) configuration, or the
[DynamicTypes](../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-topic-dynamic-data-types) definition.

Both options can be complemented, i.e. it is possible to load multiple XML files but these must not have XML profiles
with the same name.
This section explains how to configure DDS entities using XML profiles.
This includes the description of all the configuration values available for each of the XML profiles, as well as how
to create complete XML files.

* [Creating an XML profiles file](making_xml_profiles.md)
  * [Loading and applying profiles](making_xml_profiles.md#loading-and-applying-profiles)
  * [Get QoS from raw XML profiles](making_xml_profiles.md#get-qos-from-raw-xml-profiles)
  * [Rooted vs Standalone profiles definition](making_xml_profiles.md#rooted-vs-standalone-profiles-definition)
  * [Modifying predefined XML profiles](making_xml_profiles.md#modifying-predefined-xml-profiles)
  * [Dynamic content by leveraging environment variables](making_xml_profiles.md#dynamic-content-by-leveraging-environment-variables)
* [DomainParticipantFactory profiles](domainparticipantfactory.md)
  * [DomainParticipantFactory XML attributes](domainparticipantfactory.md#domainparticipantfactory-xml-attributes)
  * [DomainParticipantFactory configuration](domainparticipantfactory.md#domainparticipantfactory-configuration)
* [DomainParticipant profiles](domainparticipant.md)
  * [DomainParticipant XML attributes](domainparticipant.md#domainparticipant-xml-attributes)
  * [DomainParticipant configuration](domainparticipant.md#domainparticipant-configuration)
* [DataWriter profiles](datawriter.md)
  * [DataWriter XML attributes](datawriter.md#datawriter-xml-attributes)
  * [DataWriter configuration](datawriter.md#datawriter-configuration)
* [DataReader profiles](datareader.md)
  * [DataReader XML attributes](datareader.md#datareader-xml-attributes)
  * [DataReader configuration](datareader.md#datareader-configuration)
* [Topic profiles](topic.md)
  * [Topic XML attributes](topic.md#topic-xml-attributes)
  * [Topic configuration](topic.md#topic-configuration)
* [Transport descriptors](transports.md)
  * [TLS Configuration](transports.md#tls-configuration)
* [Intra-process delivery profiles](library_settings.md)
  * [Intra-process delivery configuration](library_settings.md#intra-process-delivery-configuration)
* [Log profiles](log.md)
  * [ConsumerDataType](log.md#consumerdatatype)
  * [PropertyType](log.md#propertytype)
* [Dynamic Types profiles](dynamic_types.md)
  * [XML Structure](dynamic_types.md#xml-structure)
  * [Type definition](dynamic_types.md#type-definition)
  * [Loading XML Types profile in *Fast DDS* application](dynamic_types.md#loading-xml-types-profile-in-fast-dds-application)
* [Common](common.md)
  * [LocatorListType](common.md#locatorlisttype)
  * [ExternalLocatorListType](common.md#externallocatorlisttype)
  * [PropertiesPolicyType](common.md#propertiespolicytype)
  * [DurationType](common.md#durationtype)
  * [TopicType](common.md#topictype)
  * [ThreadSettings](common.md#threadsettings)
  * [BuiltinTransports](common.md#builtintransports)
  * [QoS](common.md#qos)
  * [HistoryMemoryPolicy](common.md#historymemorypolicy)
  * [Allocation Configuration](common.md#allocation-configuration)
  * [Flow Controller Descriptors](common.md#flow-controller-descriptors)
* [Example](example.md)
