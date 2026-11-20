<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="security"></a>

<a id="id1"></a>

# Security

The [DDS Security](https://www.omg.org/spec/DDS-SECURITY/1.1/) specification includes five security builtin plugins.

1. Authentication plugin: [DDS:Auth:PKI-DH](auth_plugin/auth_plugin.md#auth-pki-dh).
   This plugin provides authentication for each [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) joining a DDS Domain using a trusted
   *Certificate Authority* (CA).
   Support mutual authentication between DomainParticipants and establish a shared secret.
2. Access Control plugin: [DDS:Access:Permissions](access_control_plugin/access_control_plugin.md#access-permissions).
   This plugin provides access control to DomainParticipants which perform protected operations.
3. Cryptographic plugin: [DDS:Crypto:AES-GCM-GMAC](crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac).
   This plugin provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter Mode
   (AES-GCM).
4. Logging plugin: [DDS:Logging:DDS_LogTopic](logging_plugin/logging_plugin.md#logging-logtopic).
   This plugin logs security events.
5. Data Tagging: DDS:Tagging:DDS_Discovery.
   This plugin enables the addition of security labels to the data.
   Thus it is possible to specify classification levels of the data.
   In the DDS context it can be used as a complement to access control, creating an access control based on data
   tagging; for message prioritization; and to prevent its use by the middleware to be used instead by the
   application or service.

#### NOTE
Currently the  DDS:Tagging:DDS_Discovery plugin is not implemented in Fast DDS.
Its implementation is expected for future release of Fast DDS.

In compliance with the [DDS Security](https://www.omg.org/spec/DDS-SECURITY/1.1/) specification, Fast DDS provides
secure communication by implementing pluggable security at three levels: a) DomainParticipants authentication
(DDS:Auth:PKI-DH), b) access control of Entities (DDS:Access:Permissions), and c) data encryption
(DDS:Crypto:AES-GCM-GMAC).
Furthermore, for the monitoring of the security plugins and logging relevant events, Fast DDS implements
the logging plugin (DDS:Logging:DDS_LogTopic).

By default, Fast DDS does not compile any security support, but it can be activated adding `-DSECURITY=ON` at CMake
configuration step.
For more information about Fast DDS compilation, see [Linux installation from sources](../../installation/sources/sources_linux.md#linux-sources) and [Windows installation from sources](../../installation/sources/sources_windows.md#windows-sources).

Security plugins can be activated through the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) properties.
A [`Property`](../api_reference/rtps/common/Property/Property.md#_CPPv4N8eprosima7fastdds4rtps8PropertyE) is defined by its name (`std::string`)
and its value (`std::string`).

#### IMPORTANT
Since version v3.7.0, Fast DDS **[Pro]** enforces to configure the three main security plugins
(authentication, access control and cryptography) to enable secure communication.
A laxer configuration can be achieved by configuring the access control plugin to be
permissive, but it is still mandatory to configure the authentication and cryptography plugins.

#### WARNING
For the full understanding of this documentation it is required the user to have basic knowledge of network security
since terms like Certificate Authority (CA), Public Key Infrastructure (PKI), and Diffie-Hellman encryption protocol
are not explained in detail.
However, it is possible to configure basic system security settings, i.e. authentication, access control and
encryption, to Fast DDS without this knowledge.

#### WARNING
The use of security plugins is incompatible with [Data-sharing delivery](../transport/datasharing.md#datasharing-delivery).

The following sections describe how to configure each of these properties to set up the Fast DDS security plugins.

* [Authentication plugin: DDS:Auth:PKI-DH](auth_plugin/auth_plugin.md)
  * [Generation of X.509 certificates](auth_plugin/auth_plugin.md#generation-of-x-509-certificates)
* [Access control plugin: DDS:Access:Permissions](access_control_plugin/access_control_plugin.md)
  * [Permissions CA Certificate](access_control_plugin/access_control_plugin.md#permissions-ca-certificate)
  * [Domain Governance Document](access_control_plugin/access_control_plugin.md#domain-governance-document)
  * [DomainParticipant Permissions Document](access_control_plugin/access_control_plugin.md#domainparticipant-permissions-document)
  * [Signing documents using x509 certificate](access_control_plugin/access_control_plugin.md#signing-documents-using-x509-certificate)
* [Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC](crypto_plugin/crypto_plugin.md)
* [Logging plugin: DDS:Logging:DDS_LogTopic](logging_plugin/logging_plugin.md)
* [PKCS#11 support](pkcs11_support/pkcs11_support.md)

Also, the following section contains the list of [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos)
that can be set within *Fast DDS Security*:
[Security Property QoS Settings](../property_policies/security.md#property-policies-security).
