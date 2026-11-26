<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="access-permissions"></a>

<a id="access-control-plugin-dds-access-permissions"></a>

# Access control plugin: DDS:Access:Permissions

The access control plugin provides the mechanisms and operations required for validating the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
permissions.
If the security module was activated at Fast DDS compilation, after a remote DomainParticipant is authenticated,
its permissions need to be validated and enforced.

Access rights that each DomainParticipant has over a resource are defined using the access control plugin.
For the proper functioning of a DomainParticipant in a DDS Domain, the DomainParticipant must be authorized to
operate in that specific domain.
The DomainParticipant is responsible for creating the [DataWriters](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReaders](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) that communicate over a certain
[Topic](../../dds_layer/topic/topic.md#dds-layer-topic).
Hence, a DomainParticipant must have the permissions needed to create a Topic, to publish
through its DataWriters under defined Topics, and to subscribe via its DataReaders to other Topics.
Access control plugin can configure the Cryptographic plugin as its usage is based on the DomainParticipant’s
permissions.

The authentication plugin implemented in Fast DDS is referred to as “DDS:Access:Permissions”, in compliance with the
[DDS Security](https://www.omg.org/spec/DDS-SECURITY/1.1/) specification.
This plugin is explained in detail below.

This builtin plugin provides access control using a permissions document signed by a trusted CA.
The DDS:Access:Permissions plugin requires three documents for its configuration which contents are explained
in detail below.

1. The Permissions CA certificate.
2. The Domain governance signed by the Permissions CA.
3. The DomainParticipant permissions signed by the Permissions CA.

The DDS:Access:Permissions authentication plugin, can be activated setting the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)
[`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`dds.sec.access.plugin` with the value `builtin.Access-Permissions`.
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.

The following table outlines the properties used for the DDS:Access:Permissions plugin configuration.

| Property name                               | Property value                                                                                                                                                |
|---------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| permissions_ca                              | URI to the X509 certificate of the Permissions CA.<br/>Supported URI schemes: file.<br/>The file schema shall refer to an X.509 v3 certificate in PEM format. |
| governance                                  | URI to shared Governance Document signed by the Permissions CA  in S/MIME format.<br/>Supported URI schemes: file.                                            |
| permissions                                 | URI to the Participant permissions document signed by the  Permissions CA in S/MIME format.<br/>Supported URI schemes: file.                                  |
| transmit_algorithms_as_legacy  *(optional)* | Whether to transmit algorithm identifiers in non-standard legacy format.<br/>Will default to `false` if the property is not present.                          |

#### NOTE
All listed properties have “dds.sec.access.builtin.Access-Permissions.” prefix.
For example: `dds.sec.access.builtin.Access-Permissions.permissions_ca`.

The following is an example of how to set the properties of [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) for the DDS:Access:Permissions
configuration.

### C++

```c++
DomainParticipantQos pqos;

// Activate DDS:Access:Permissions plugin
pqos.properties().properties().emplace_back("dds.sec.access.plugin",
        "builtin.Access-Permissions");

// Configure DDS:Access:Permissions plugin
pqos.properties().properties().emplace_back(
    "dds.sec.access.builtin.Access-Permissions.permissions_ca",
    "file://certs/maincacert.pem");
pqos.properties().properties().emplace_back(
    "dds.sec.access.builtin.Access-Permissions.governance",
    "file://certs/governance.smime");
pqos.properties().properties().emplace_back(
    "dds.sec.access.builtin.Access-Permissions.permissions",
    "file://certs/permissions.smime");
pqos.properties().properties().emplace_back(
    "dds.sec.access.builtin.Access-Permissions.transmit_algorithms_as_legacy",
    "true");
```

### XML

```xml
<participant profile_name="secure_domainparticipant_conf_access_control_plugin_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Activate DDS:Access:Permissions plugin -->
                <property>
                    <name>dds.sec.access.plugin</name>
                    <value>builtin.Access-Permissions</value>
                </property>
                <!-- Configure DDS:Access:Permissions plugin -->
                <property>
                    <name>dds.sec.access.builtin.Access-Permissions.permissions_ca</name>
                    <value>file://maincacet.pem</value>
                </property>
                <property>
                    <name>dds.sec.access.builtin.Access-Permissions.governance</name>
                    <value>file://governance.smime</value>
                </property>
                <property>
                    <name>dds.sec.access.builtin.Access-Permissions.permissions</name>
                    <value>file://permissions.smime</value>
                </property>
                <property>
                    <name>dds.sec.access.builtin.Access-Permissions.transmit_algorithms_as_legacy</name>
                    <value>true</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

#### IMPORTANT
Since version v3.7.0, Fast DDS **[Pro]** enforces to configure the three main security plugins
(authentication, access control and cryptography) to enable secure communication.
The previous example focuses only on the access control plugin, but all three plugins must be configured.

<a id="permissions-ca-cert"></a>

<a id="permissions-ca-certificate"></a>

## Permissions CA Certificate

This is an X.509 certificate that contains the Public Key of the CA that will be used to sign the
[Domain Governance Document](#domain-governance-doc) and the [DomainParticipant Permissions Document](#domainparticipant-permissions-doc).

<a id="domain-governance-doc"></a>

<a id="domain-governance-document"></a>

## Domain Governance Document

Domain Governance document is an XML document that specifies the mechanisms to secure the DDS Domain.
It shall be signed by the Permissions CA in S/MIME format.
The XML scheme of this document is defined in [Domain Governance XSD](governance.md#domain-governance-xsd-rst).
The following is an example of the Domain Governance XML file contents.

```xml
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="omg_shared_ca_domain_governance.xsd">
    <domain_access_rules>
        <domain_rule>
            <domains>
                <id_range>
                    <min>0</min>
                    <max>230</max>
                </id_range>
            </domains>
            <allow_unauthenticated_participants>false</allow_unauthenticated_participants>
            <enable_join_access_control>true</enable_join_access_control>
            <discovery_protection_kind>ENCRYPT</discovery_protection_kind>
            <liveliness_protection_kind>ENCRYPT</liveliness_protection_kind>
            <rtps_protection_kind>ENCRYPT</rtps_protection_kind>
            <topic_access_rules>
                <topic_rule>
                    <topic_expression>HelloWorldTopic</topic_expression>
                    <enable_discovery_protection>true</enable_discovery_protection>
                    <enable_liveliness_protection>false</enable_liveliness_protection>
                    <enable_read_access_control>true</enable_read_access_control>
                    <enable_write_access_control>true</enable_write_access_control>
                    <metadata_protection_kind>ENCRYPT</metadata_protection_kind>
                    <data_protection_kind>ENCRYPT</data_protection_kind>
                </topic_rule>
            </topic_access_rules>
        </domain_rule>
    </domain_access_rules>
</dds>
```

The [Governance XSD file](https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/governance.xsd) and
the
[Governance XML example](https://github.com/eProsima/Fast-DDS/blob/master/examples/cpp/dds/SecureHelloWorldExample/certs/governance.xml)
can also be downloaded from the [eProsima Fast DDS Github repository](https://github.com/eProsima/Fast-DDS).

<a id="domain-rules"></a>

### Domain Rules

It allows the application of rules to the DDS Domain.
The domain rules define aspects of the DDS Domain such as:

* Whether the discovery data should be protected and the type of protection: MAC only or encryption followed by MAC.
* Whether the whole RTPS message should be encrypted.
* Whether the liveliness of the messages should be protected.
* Whether a non-authenticated DomainParticipant can access or not to the unprotected discovery metatraffic and
  unprotected Topics.
* Whether an authenticated DomainParticipant can access the domain without evaluating the access control policies.
* Whether discovery information on a certain Topic should be sent with secure DataWriters.
* Whether or not the access to Topics should be restricted to DomainParticipants with the appropriate permission to
  read them.
* Whether the metadata sent on a certain Topic should be protected and the type of protection.
* Whether payload data on a certain Topic should be protected and the type of protection.

The domain rules are evaluated in the same order as they appear in the document.
A rule only applies to a particular DomainParticipant if the domain section matches the DDS
`Domain_Id` to which the DomainParticipant belongs.
If multiple rules match, the first rule that matches is the only one that applies.
Each domain rule is delimited by the `<domain_rule>` XML element tag.

Some domain rules may have an additional configuration if enabled.
This configuration defines the level of protection that the rule applies to the domain:

* **NONE**: no cryptographic transformation is applied.
* **SIGN**: cryptographic transformation based on Message Authentication Code (MAC) is applied, without additional
  encryption.
* **ENCRYPT**: the data is encrypted and followed by a MAC computed on the ciphertext, also known as Encrypt-then-MAC.

The following table summarizes the elements and sections that each domain rule may contain.

| **Type**                                                  | **Name**                                                                  | **XML element tag**                    | **Values**   |
|-----------------------------------------------------------|---------------------------------------------------------------------------|----------------------------------------|--------------|
| Element                                                   | [Allow Unauthenticated Participants](#allow-unauthenticated-participants) | `<allow_unauthenticated_participants>` | `false`      |
| `true`                                                    |                                                                           |                                        |              |
| [Enable Join Access Control](#enable-join-access-control) | `<enable_join_access_control>`                                            | `false`                                |              |
| `true`                                                    |                                                                           |                                        |              |
| [Discovery Protection Kind](#discovery-protection-kind)   | `<discovery_protection_kind>`                                             | `SIGN`                                 |              |
| `ENCRYPT`                                                 |                                                                           |                                        |              |
| `NONE`                                                    |                                                                           |                                        |              |
| [Liveliness Protection Kind](#liveliness-protection-kind) | `<liveliness_protection_kind>`                                            | `SIGN`                                 |              |
| `ENCRYPT`                                                 |                                                                           |                                        |              |
| `NONE`                                                    |                                                                           |                                        |              |
| [RTPS Protection Kind](#rtps-protection-kind)             | `<rtps_protection_kind>`                                                  | `SIGN`                                 |              |
| `ENCRYPT`                                                 |                                                                           |                                        |              |
| `NONE`                                                    |                                                                           |                                        |              |
| Section                                                   | [Domains](#domains)                                                       | `<domains>`                            | `<domains>`  |
| [Topic Access Rules](#topic-rules)                        | `<topic_access_rules>`                                                    | `<topic_rule>`                         |              |
<!-- todo:

check  that link `Topic Access Rules <topic_rules>` works correctly -->

The following describes the possible configurations of each of the elements and sections listed above that are
contained in the domain rules.

<a id="domains-section"></a>

<a id="domains"></a>

#### Domains

This element is delimited by the `<domains>` XML element tag.
The value in this element identifies the collection of DDS Domains to which the rule applies.
The `<domains>` element can contain:

* A single domain identifier:

```xml
<domains>
    <id>1</id>
</domains>
```

* A range of domain identifiers:

```xml
<domains>
    <id_range>
        <min>1</min>
        <max>10</max>
    </id_range>
</domains>
```

Or a combination of both, a list of domain identifiers and ranges of domain identifiers.

<a id="allow-unauthenticated-section"></a>

<a id="allow-unauthenticated-participants"></a>

#### Allow Unauthenticated Participants

This element is delimited by the `<allow_unauthenticated_participants>` XML element tag.
It indicates whether the matching of a DomainParticipant with a remote DomainParticipant requires authentication.
The possible values for this element are:

* `false`: the DomainParticipant shall enforce the authentication of remote [DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) and
  disallow matching those that cannot be successfully authenticated.
* `true`: the DomainParticipant shall allow matching other DomainParticipants (event if the remote
  DomainParticipant cannot authenticate) as long as there is not an already valid authentication with the same
  DomainParticipant’s GUID.

In accordance with the [DDS Security specification](https://www.omg.org/spec/DDS-SECURITY/), the following premises should be considered:

* Topics protected with `enable_read_access_control` or `enable_write_access_control` will not communicate
  regardless of the allow_unauthenticated_participants flag value.
* If [RTPS Protection Kind](#rtps-protection-kind-section) is not `NONE` and [Allow Unauthenticated Participants](#allow-unauthenticated-section) is enabled,
  the entity creation will fail with an error.
* Authentication is always attempted first regardless of `<allow_unauthenticated_participants>` configuration.

<a id="enable-join-access-control"></a>

#### Enable Join Access Control

This element is delimited by the `<enable_join_access_control>` XML element tag.
Indicates whether the matching of the participant with a remote DomqainParticipant requires authorization by the
DDS:Access:Permissions plugin.
Its possible values are:

* `false`: the DomainParticipant shall not check the permissions of the authenticated remote DomainParticipant.
* `true`: the DomainParticipant shall check the permissions of the authenticated remote DomainParticipant.

<a id="discovery-protection-kind"></a>

#### Discovery Protection Kind

This element is delimited by the `<discovery_protection_kind>` XML element tag.
Indicates whether the secure channel of the endpoint discovery phase needs to be encrypted.
The possible values are:

* `NONE`: the secure channel shall not be protected.
* `SIGN`: the secure channel shall be protected by MAC.
* `ENCRYPT`: the secure channel shall be encrypted.

<a id="liveliness-protection-kind"></a>

#### Liveliness Protection Kind

This element is delimited by the `<liveliness_protection_kind>` XML element tag.
Indicates whether the secure channel of the liveliness mechanism needs to be encrypted.
The possible values are:

* `NONE`: the secure channel shall not be protected.
* `SIGN`: the secure channel shall be protected by MAC.
* `ENCRYPT`: the secure channel shall be encrypted.

<a id="rtps-protection-kind-section"></a>

<a id="rtps-protection-kind"></a>

#### RTPS Protection Kind

This element is delimited by the `<rtps_protection_kind>` XML element tag.
Indicates whether the whole RTPS Message needs to be encrypted.
The possible values are:

* `NONE`: whole RTPS Messages shall not be protected.
* `SIGN`: whole RTPS Messages shall be protected by MAC.
* `ENCRYPT`: whole RTPS Messages shall be encrypted.

<a id="topic-rules"></a>

<a id="topic-rule"></a>

#### Topic Rule

This element is delimited by the `<topic_rule>` XML element tag and appears within the Topic Access Rules Section
whose XML element tag is `<topic_access_rules>`.
The following table summarizes the elements and sections that each domain rule may contain.

| **Elements**                                                  | **XML element tag**              | **Values**   |
|---------------------------------------------------------------|----------------------------------|--------------|
| [Topic expression](#topic-expression)                         | `<topic_expression>`             | Topic name   |
| [Enable Discovery Protection](#enable-discovery-protection)   | `<enable_discovery_protection>`  | `false`      |
| `true`                                                        |                                  |              |
| [Enable Liveliness Protection](#enable-liveliness-protection) | `<enable_liveliness_protection>` | `false`      |
| `true`                                                        |                                  |              |
| [Enable Read Access Control](#enable-read-access-control)     | `<enable_read_access_control>`   | `false`      |
| `true`                                                        |                                  |              |
| [Enable Write Access Control](#enable-write-access-control)   | `<enable_write_access_control>`  | `false`      |
| `true`                                                        |                                  |              |
| [Metadata protection Kind](#metadata-protection-kind)         | `<metadata_protection_kind>`     | `SIGN`       |
| `ENCRYPT`                                                     |                                  |              |
| `NONE`                                                        |                                  |              |
| [Data protection Kind](#data-protection-kind)                 | `<data_protection_kind>`         | `SIGN`       |
| `ENCRYPT`                                                     |                                  |              |
| `NONE`                                                        |                                  |              |

The topic expression within the rules selects a set of Topic names.
The rule applies to any [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) or [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) associated with a [Topic](../../dds_layer/topic/topic.md#dds-layer-topic) whose name matches the Topic expression
name.
The topic access rules are evaluated in the same order as they appear within the `<topic_access_rules>` section.
If multiple rules match, the first rule that matches is the only one that applies.
If no matching `<topic_rule>` is found, the entity creation will fail.

<a id="topic-expression"></a>

<a id="id1"></a>

##### Topic expression

This element is delimited by the `<topic_expression>` XML element tag.
The value in this element identifies the set of Topic names to which the rule applies.
The rule applies to any DataReader or DataWriter associated with a Topic whose name matches the value.

The Topic name expression syntax and matching shall use the syntax and rules of the POSIX `fnmatch()` function as
specified in [IEEE 1003.1-2017](https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html).

<a id="enable-discovery-protection"></a>

<a id="id2"></a>

##### Enable Discovery Protection

This element is delimited by the `<enable_discovery_protection>` XML element tag.
Indicates whether the entity related discovery information shall go through the secure channel of endpoint discovery
phase.

* `false`: the entity discovery information shall be sent by an unsecured channel of discovery.
* `true`: the information shall be sent by the secure channel.

<a id="enable-liveliness-protection"></a>

<a id="id3"></a>

##### Enable Liveliness Protection

This element is delimited by the `<enable_liveliness_protection>` XML element tag.
Indicates whether the entity related liveliness information shall go through the secure channel of liveliness
mechanism.

* `false`: the entity liveliness information shall be sent by an unsecured channel of liveliness.
* `true`: the information shall be sent by the secure channel.

<a id="enable-read-access-control"></a>

<a id="id4"></a>

##### Enable Read Access Control

This element is delimited by the `<enable_read_access_control>` XML element tag.
Indicates whether read access to the Topic is protected.

* `false`: then local Subscriber creation and remote Subscriber matching can proceed without further
  access-control mechanisms imposed.
* `true`: they shall be checked using Access control plugin.

<a id="enable-write-access-control"></a>

<a id="id5"></a>

##### Enable Write Access Control

This element is delimited by the `<enable_write_access_control>` XML element tag.
Indicates whether write access to the Topic is protected.

* `false`: then local Publisher creation and remote Publisher matching can proceed without further
  access-control mechanisms imposed.
* `true`: they shall be checked using Access control plugin.

<a id="metadata-protection-kind"></a>

<a id="id6"></a>

##### Metadata Protection Kind

This element is delimited by the `<metadata_protection_kind>` XML element tag.
Indicates whether the entity’s RTPS submessages shall be encrypted by the Cryptographic plugin.

* `NONE`: shall not be protected.
* `SIGN`: shall be protected by MAC.
* `ENCRYPT`: shall be encrypted.

<a id="data-protection-kind"></a>

<a id="id7"></a>

##### Data Protection Kind

This element is delimited by the `<data_protection_kind>` XML element tag.
Indicates whether the data payload shall be encrypted by the Cryptographic plugin.

* `NONE`: shall not be protected.
* `SIGN`: shall be protected by MAC.
* `ENCRYPT`: shall be encrypted.

<a id="domainparticipant-permissions-doc"></a>

<a id="domainparticipant-permissions-document"></a>

## DomainParticipant Permissions Document

The permissions document is an XML file which contains the permissions of a DomainParticipant and binds them to the
DomainParticipant distinguished name defined in the DDS:Auth:PKI-DH plugin.
The permissions document shall be signed by the Permissions CA in S/MIME format.
The XML scheme of this document is defined in [DomainParticipant Permissions XSD](permissions.md#domainparticipant-permissions-xsd-rst).
The following is an example of the DomainParticipant Permissions XML file contents.

```xml
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-Security/20170801/omg_shared_ca_permissions.xsd">
    <permissions>
        <grant name="ParticipantPermissions">
            <subject_name>emailAddress=example@eprosima.com, CN=DomainParticipantName, O=eProsima, ST=MA, C=ES</subject_name>
            <validity>
                <not_before>2013-06-01T13:00:00</not_before>
                <not_after>2038-06-01T13:00:00</not_after>
            </validity>
            <allow_rule>
                <domains>
                    <id_range>
                        <min>0</min>
                        <max>230</max>
                    </id_range>
                </domains>
                <publish>
                    <topics>
                        <topic>HelloWorldTopic</topic>
                    </topics>
                </publish>
            </allow_rule>
            <default>DENY</default>
        </grant>

        <!-- Fill the subject name with the information specified in the Participant certificate -->
        <grant name="OtherParticipantPermissions">
            <subject_name> emailAddress=xxx@eprosima.com, CN=xxx, OU=x, O=x, ST=XX, C=XX</subject_name>
            <validity>
                <not_before>2013-06-01T13:00:00</not_before>
                <not_after>2038-06-01T13:00:00</not_after>
            </validity>
            <allow_rule>
                <domains>
                    <id_range>
                        <min>0</min>
                        <max>230</max>
                    </id_range>
                </domains>
                <subscribe>
                    <topics>
                        <topic>HelloWorldTopic</topic>
                    </topics>
                </subscribe>
            </allow_rule>
            <default>DENY</default>
        </grant>
    </permissions>
</dds>
```

The [Permissions XSD file](https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/governance.xsd) and
the
[Permissions XML example](https://github.com/eProsima/Fast-DDS/blob/master/examples/cpp/dds/SecureHelloWorldExample/certs/governance.xml)
can also be downloaded from the [eProsima Fast DDS Github repository](https://github.com/eProsima/Fast-DDS).

<a id="grant-section"></a>

### Grant Section

This section is delimited by the `<grant>` XML element tag.
Each grant section contains three sections:

* Subject name
* Validity
* Rules

<a id="subject-name"></a>

#### Subject name

This section is delimited by XML element `<subject_name>`.
The subject name identifies the DomainParticipant to which the permissions apply.
Each subject name can only appear in a single `<permissions>` section within the XML Permissions document.
The contents of the subject name element shall be the X.509 subject name of the DomainParticipant that was given in
the authorization X.509 Certificate.

<a id="validity"></a>

#### Validity

This section is delimited by the XML element `<validity>`.
It reflects the valid dates for the permissions.

<a id="rules"></a>

#### Rules

This section contains the permissions assigned to the DomainParticipant.
The rules are applied in the same order that appears in the document.
If the criteria for the rule matched the Domain join, publish or subscribe operation that is being attempted,
then the *allow* or *deny* decision is applied.
If the criteria for a rule does not match the operation being attempted, the evaluation shall proceed to the next rule.
If all rules have been examined without a match, then the decision specified by the `<default>` rule is applied.
The default rule, if present, must appear after all *allow* and *deny* rules.
If the default rule is not present, the implied default decision is `DENY`.

For the grant to match there shall be a match of the topics and partitions criteria.

Allow rules are delimited by the XML element `<allow_rule>`.
Deny rules are delimited by the XML element\`\`<deny_rule>\`\`.
Both contain the same element children.

<a id="id9"></a>

### Domains Section

This section is delimited by the XML element `<domains>`.
The value in this element identifies the collection of DDS Domains to which the rule applies.
The syntax is the same as for the [Domains](#domains-section) of the [Domain Governance Document](#domain-governance-doc).

<a id="format-of-the-allowed-denied-actions-sections"></a>

### Format of the Allowed/Denied Actions sections

The sections for each of the three actions have a similar format.
The only difference is the name of the XML element used to delimit the action:

| **Action**           | **XML element tag**   |
|----------------------|-----------------------|
| Allow/Deny Publish   | `<publish>`           |
| Allow/Deny Subscribe | `<subscribe>`         |
| Allow/Deny Relay     | `<relay>`             |

Each action contains two conditions.

* Allowed/Denied [Topics Condition](#topics-condition)
* Allowed/Denied [Partitions Condition](#partitions-condition)

<a id="topics-condition"></a>

#### Topics Condition

This section is delimited by the `<topics>` XML element.
It defines the Topic names that must be matched for the allow/deny rule to apply.
Topic names may be given explicitly or by means of Topic name expressions.
Each explicit topic name or Topic name expressions appears separately in a `<topic>` sub-element within the
`<topics>` element.

The Topic name expression syntax and matching shall use the syntax and rules of the POSIX `fnmatch()` function as
specified in

```xml
<topics>
    <topic>Plane</topic>
    <topic>Hel*</topic>
</topics>
```

<a id="partitions-condition"></a>

#### Partitions Condition

This section is delimited by the `<partitions>` XML element.
It limits the set Partitions names that may be associated with the (publish, subscribe, relay) action for the rule to
apply.
Partition names expression syntax and matching shall use the syntax and rules of the POSIX `fnmatch()` function as
specified in [IEEE 1003.1-2017](https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html).
If there is no `<partitions>` section within a rule, then the default “empty string” partition is assumed.

```xml
<partitions>
    <partition>A</partition>
    <partition>B*</partition>
</partitions>
```

<a id="signing-documents-using-x509-certificate"></a>

## Signing documents using x509 certificate

[Domain Governance Document](#domain-governance-doc) and [DomainParticipant Permissions Document](#domainparticipant-permissions-doc) have to be signed using an X.509 certificate.
Generation of an X.509 certificate is explained in [Generation of X.509 certificates](../auth_plugin/auth_plugin.md#generate-x509).
Next commands sign the necessary documents for its use by the DDS:Access:Permissions plugin.

```bash
# Governance document: governance.xml
openssl smime -sign -in governance.xml -text -out governance.smime -signer maincacert.pem -inkey maincakey.pem

# Permissions document: permissions.xml
openssl smime -sign -in permissions.xml -text -out permissions.smime -signer maincacert.pem -inkey maincakey.pem
```
