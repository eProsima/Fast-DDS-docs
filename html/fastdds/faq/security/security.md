<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-security-questions"></a>

<a id="security-frequently-asked-questions"></a>

# Security Frequently Asked Questions

**What security plugins does Fast DDS offer for secure communication?**

<br/> Fast DDS offers five built-in security plugins as part of the DDS Security specification:
the Authentication plugin ([Authentication plugin: DDS:Auth:PKI-DH](../../security/auth_plugin/auth_plugin.md#auth-pki-dh)) provides authentication between DomainParticipants using a trusted Certificate Authority (CA) and mutual authentication;
the Access Control plugin ([Access control plugin: DDS:Access:Permissions](../../security/access_control_plugin/access_control_plugin.md#access-permissions)) enforces permissions for protected operations;
the Cryptographic plugin ([Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC](../../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac)) ensures authenticated encryption and data integrity using AES in Galois Counter Mode (AES-GCM);
the Logging plugin ([Logging plugin: DDS:Logging:DDS_LogTopic](../../security/logging_plugin/logging_plugin.md#logging-logtopic)) logs security-related events.
For further information, refer to [Security](../../security/security.md#security).

---

**Is the security support configured by default?**

<br/> No. It must be activated using `-DSECURITY=ON` at the CMake configuration step.
For further information, refer to [Security](../../security/security.md#security).

---

<a id="authentication"></a>

## Authentication

**What is the purpose of authentication?**

<br/> When a [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is either locally created or discovered, it needs to be authenticated in order to be able to communicate in a DDS Domain.
For further information, refer to [Authentication plugin: DDS:Auth:PKI-DH](../../security/auth_plugin/auth_plugin.md#auth-pki-dh).

---

**What happens if the authentication fails?**

<br/> The remote [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is rejected, therefore communication cannot take place in the DDS Domain for this DomainParticipant.
For further information, refer to [Authentication plugin: DDS:Auth:PKI-DH](../../security/auth_plugin/auth_plugin.md#auth-pki-dh).

---

**How is the DDS:Auth:PKI-DH authentication plugin activated?**

<br/> By setting the [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `dds.sec.auth.plugin` with the value\`\` `builtin.PKI-DH`.
For further information, refer to [Authentication plugin: DDS:Auth:PKI-DH](../../security/auth_plugin/auth_plugin.md#auth-pki-dh).
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.

---

**What is the process for generating and managing security certificates for the Authentication plugin?**

<br/> The process for generating and managing security certificates for the Authentication plugin involves creating and managing X.509 certificates.
First, since multiple certificates will need to be issued, one for each of the DomainParticipants, a dedicated CA is set up, and the CA’s certificate is installed as the root key of all DomainParticipants.
Thus, the DomainParticipants will accept all certificates issued by our own CA. To create a proprietary CA certificate, a configuration file must first be written with the CA information.
After writing the configuration file, the certificate  is generates using the Elliptic Curve Digital Signature Algorithm (ECDSA).
As was done for the CA, a DomainParticipant certificate configuration file needs to be created first.
After writing the DomainParticipant certificate configuration file, the X.509 certificate is generated using ECDSA, for a DomainParticipant.
Finally, the CRL is created.
This is a list of the X.509 certificates revoked by the certificate issuing CA before they reach their expiration date.
Any certificate that is on this list will no longer be trusted.
For further information, refer to [Generation of X.509 certificates](../../security/auth_plugin/auth_plugin.md#generate-x509).

---

<a id="access-control"></a>

## Access control

**What is the purpose of access control?**

<br/> Provides the mechanisms and operations required to validate the DomainParticipant permissions and define access rights over a resource.
For further information, refer to [Access control plugin: DDS:Access:Permissions](../../security/access_control_plugin/access_control_plugin.md#access-permissions).

---

**How is the DDS:Access:Permissions authentication plugin activated?**

<br/> By setting the [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `dds.sec.access.plugin` with the value `builtin.Access-Permissions`.
For further information, refer to [Access control plugin: DDS:Access:Permissions](../../security/access_control_plugin/access_control_plugin.md#access-permissions).

---

**Can a DomainParticipant match with a remote DomainParticipant without authentication?**

<br/> Yes. This can be delimited by the `<allow_unauthenticated_participants>` XML element tag.
When it is set to true, the DomainParticipant can match other DomainParticipants without authentication.
For further information, refer to [Allow Unauthenticated Participants](../../security/access_control_plugin/access_control_plugin.md#allow-unauthenticated-section).

---

**Can the secure channel of the endpoint discovery phase be encrypted?**

<br/> Yes, if the `<discovery_protection_kind>` XML element is set to ENCRYPT.
This is also applicable for Liveliness and RTPS.
For further information, refer to [Access control plugin: DDS:Access:Permissions](../../security/access_control_plugin/access_control_plugin.md#access-permissions).

---

**How is the access to topics managed?**

<br/> By applying topic rules to any DataReader or DataWriter associated with a topic that matches the [`Topic`](../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) expression name.
For further information, refer to [Topic Rule](../../security/access_control_plugin/access_control_plugin.md#topic-rules).

---

**What is the purpose of a DomainParticipant Permissions Document in the DDS:Auth:PKI-DH plugin?**

<br/> The permissions document is an XML file that contains the permissions of a DomainParticipant and binds them to the DomainParticipant distinguished name defined in the `DDS:Auth:PKI-DH` plugin.
For further information, refer to [DomainParticipant Permissions Document](../../security/access_control_plugin/access_control_plugin.md#domainparticipant-permissions-doc).

---

**What are the main components of a DomainParticipant Permissions document in DDS?**

<br/> There are several sections. Grant Section, delimited by the `<grant>` XML element tag, including the subject name, validity, and rules.
Domains sections, delimited by the XML element `<domains>`, identifying the collection of DDS Domains to which the rule applies.
Allowed/Denied Actions sections for publishing, subscribing, relaying, topics, and partitions.
For further information, refer to [DomainParticipant Permissions Document](../../security/access_control_plugin/access_control_plugin.md#domainparticipant-permissions-doc).

---

<a id="data-encryption"></a>

## Data encryption

**What is the function of the cryptographic plugin in the context of DDS?**

<br/> The cryptographic plugin provides the tools and operations required to support encryption and decryption, digests computation, message authentication codes computation and verification, key generation, and key exchange for DomainParticipants, DataWriters, and DataReaders.
For further information, refer to [Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC](../../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac).

---

**How is the DDS:Crypto:AES-GCM-GMAC authentication plugin activated?**

<br/> By setting the [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `dds.sec.crypto.plugin` with the value `builtin.AES-GCM-GMAC`.
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.
Moreover, this plugin needs the activation of the Authentication plugin: `DDS:Auth:PKI-DH` and the `DDS:Access:Permissions`.
For further information, refer to [Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC](../../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac).

---

<a id="logging"></a>

## Logging

**What is the function of the logging plugin in Fast DDS?**

<br/> The logging plugin provides the necessary operations to log the security events triggered by the other security plugins supported by Fast DDS.
For further information, refer to [Logging plugin: DDS:Logging:DDS_LogTopic](../../security/logging_plugin/logging_plugin.md#logging-logtopic).

---

**How is the DDS:Logging:DDS_LogTopic authentication plugin activated?**

<br/> By setting the [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `dds.sec.log.plugin` with the value `builtin.DDS_LogTopic`.
For further information, refer to [Logging plugin: DDS:Logging:DDS_LogTopic](../../security/logging_plugin/logging_plugin.md#logging-logtopic).

<br/>
