<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="property-policies-security"></a>

<a id="security-plugins-settings"></a>

# Security Plugins Settings

As described in the [Security](../security/security.md#security) section, the security
plugins admit a set of settings that can be configured.

<a id="authentication-plugin-settings"></a>

## Authentication plugin settings

The [DDS:Auth:PKI-DH](../security/auth_plugin/auth_plugin.md#auth-pki-dh) authentication plugin, can be activated setting the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)
[`properties()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`dds.sec.auth.plugin` with the value `builtin.PKI-DH`.
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.

The following table outlines the properties used for the [DDS:Auth:PKI-DH](../security/auth_plugin/auth_plugin.md#auth-pki-dh) plugin configuration.

| PropertyPolicyQos name                        | PropertyPolicyQos value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|-----------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `identity_ca`                                 | URI to the X.509 v3 certificate of the Identity CA in PEM format.<br/>Supported URI schemes: file.                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `identity_certificate`                        | URI to an X.509 v3 certificate signed by the Identity CA in PEM format<br/>containing the signed public key for the Participant.<br/>Supported URI schemes: file.                                                                                                                                                                                                                                                                                                                                                                       |
| `identity_crl`  *(optional)*                  | URI to a X.509 Certificate Revocation List (CRL).<br/>Supported URI schemes: file.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `private_key`                                 | URI to access the private Private Key for the Participant.<br/>Supported URI schemes: file, [PKCS#11](../security/pkcs11_support/pkcs11_support.md#pkcs11-support).                                                                                                                                                                                                                                                                                                                                                                     |
| `password`  *(optional)*                      | A password used to decrypt the *private_key*.<br/>If the *password* property is not present, then the value supplied in the<br/>*private_key* property must contain the decrypted private key.<br/>The *password* property is ignored if the *private_key* is given in PKCS#11 scheme.                                                                                                                                                                                                                                                  |
| `preferred_key_agreement`  *(optional)*       | The preferred algorithm to use for generating the session’s shared secret<br/>at the end of the authentication phase. Supported values are:<br/>a) `DH`, `DH+MODP-2048-256` for  Diffie-Hellman Ephemeral with 2048-bit MODP Group parameters.<br/>b) `ECDH`, `ECDH+prime256v1-CEUM` for Elliptic Curve Diffie-Hellman Ephemeral with the NIST P-256 curve.<br/>c) `AUTO` for selecting the key agreement based on the signature algorithm in the Identity CA’s certificate.<br/>Will default to `AUTO` if the property is not present. |
| `transmit_algorithms_as_legacy`  *(optional)* | Whether to transmit algorithm identifiers in non-standard legacy format.<br/>Will default to `false` if the property is not present.                                                                                                                                                                                                                                                                                                                                                                                                    |

#### NOTE
All properties listed above have the `dds.sec.auth.builtin.PKI-DH."` prefix.
For example: `dds.sec.auth.builtin.PKI-DH.identity_ca`. For examples
and further information, please refer to the [Authentication plugin: DDS:Auth:PKI-DH](../security/auth_plugin/auth_plugin.md#auth-pki-dh) section.

<a id="authentication-handshake-settings"></a>

## Authentication handshake settings

The authentication phase starts when discovery information is received
from the remote [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant). At this moment, the participant sends
a handshake request until a handshake response is received from the remote participant.
Some parameters are involved in the behavior of this exchange:

* `max_handshake_requests` controls the maximum number of handshake requests to be sent.
* `initial_handshake_resend_period` represents the initial waiting time (in milliseconds)
  for the first handshake request that has to be resent.
* `handshake_resend_period_gain` is the gain against which the period is multiplied
  between two handshake requests.

Hence, the period of time to wait for sending a new handshake request is computed at each
iteration as the period between the last two handshake requests multiplied by the gain
(so that the period increases).

The following table lists the
settings to configure the authentication handshake behavior within
the `dds.sec.auth.builtin.PKI-DH` plugin:

| PropertyPolicyQos name            | PropertyPolicyQos value   | PropertyPolicyQos bounds   | Default value   |
|-----------------------------------|---------------------------|----------------------------|-----------------|
| `max_handshake_requests`          | `<int>`                   | `[1, max)`                 | `10`            |
| `initial_handshake_resend_period` | `<int>`                   | `[1, max)`                 | `125`           |
| `handshake_resend_period_gain`    | `<double>`                | `(1.0, max)`               | `1.5`           |

#### NOTE
All listed properties have the `dds.sec.auth.builtin.PKI-DH.` prefix.
For example: `dds.sec.auth.builtin.PKI-DH.max_handshake_requests`.

The following is an example of how to set the properties of DomainParticipantQoS for the
authentication handshake configuration.

### C++

```c++
DomainParticipantQos pqos;

pqos.properties().properties().emplace_back(
    "dds.sec.auth.builtin.PKI-DH.max_handshake_requests",
    "5");
pqos.properties().properties().emplace_back(
    "dds.sec.auth.builtin.PKI-DH.initial_handshake_resend_period",
    "250");
pqos.properties().properties().emplace_back(
    "dds.sec.auth.builtin.PKI-DH.handshake_resend_period_gain",
    "1.5");
```

### XML

```xml
<participant profile_name="secure_domainparticipant_conf_auth_handshake_props_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <property>
                    <name>dds.sec.auth.builtin.PKI-DH.max_handshake_requests</name>
                    <value>5</value>
                </property>
                <property>
                    <name>dds.sec.auth.builtin.PKI-DH.initial_handshake_resend_period</name>
                    <value>250</value>
                </property>
                <property>
                    <name>dds.sec.auth.builtin.PKI-DH.handshake_resend_period_gain</name>
                    <value>1.5</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

<a id="cryptographic-plugin-settings"></a>

## Cryptographic plugin settings

The [DDS:Crypto:AES-GCM-GMAC](../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac) authentication plugin,
can be activated setting the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) [`properties()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`dds.sec.crypto.plugin` with the value `builtin.AES-GCM-GMAC`.
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.

Moreover, this plugin needs the activation of the [Authentication plugin: DDS:Auth:PKI-DH](../security/auth_plugin/auth_plugin.md#auth-pki-dh).
The [DDS:Crypto:AES-GCM-GMAC](../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac) plugin is configured using the
[Access control plugin: DDS:Access:Permissions](../security/access_control_plugin/access_control_plugin.md#access-permissions), i.e the cryptography plugin is configured through the properties
and configuration files of the access control plugin.
For further information and examples in this regard please refer to [Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC](../security/crypto_plugin/crypto_plugin.md#crypto-aes-gcm-gmac).

<a id="logging-plugin-settings"></a>

## Logging plugin settings

The [DDS:Logging:DDS_LogTopic](../security/logging_plugin/logging_plugin.md#logging-logtopic) authentication plugin,
can be activated setting the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) [`properties()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`dds.sec.log.plugin` with the value `builtin.DDS_LogTopic`.
The following table outlines the properties used for the DDS:Logging:DDS_LogTopic plugin configuration.
For further information and examples follow the dedicated documentation: [Logging plugin: DDS:Logging:DDS_LogTopic](../security/logging_plugin/logging_plugin.md#logging-logtopic).
