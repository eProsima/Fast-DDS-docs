<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="crypto-aes-gcm-gmac"></a>

<a id="cryptographic-plugin-dds-crypto-aes-gcm-gmac"></a>

# Cryptographic plugin: DDS:Crypto:AES-GCM-GMAC

The cryptographic plugin provides the tools and operations required to support encryption and decryption,
digests computation, message authentication codes computation and verification, key generation, and key exchange for
DomainParticipants, [DataWriters](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReaders](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
Encryption can be applied over three different levels of DDS protocol:

* The whole RTPS messages.
* The RTPS submessages of a specific DDS Entity (DataWriter or DataReader).
* The payload (user data) of a particular DataWriter.

The authentication plugin implemented in Fast DDS is referred to as “DDS:Crypto:AES-GCM-GMAC”, in compliance with the
[DDS Security](https://www.omg.org/spec/DDS-SECURITY/1.1/) specification.
This plugin is explained in detail below.

The DDS:Crypto:AES-GCM-GMAC plugin provides authentication encryption using Advanced Encryption Standard (AES) in
Galois Counter Mode ([AES-GCM](https://csrc.nist.gov/publications/detail/sp/800-38d/final)).
It supports 128 bits and 256 bits AES key sizes.
It may also provide additional DataReader-specific Message Authentication Codes (MACs) using Galois MAC
([AES-GMAC](https://csrc.nist.gov/publications/detail/sp/800-38d/final)).

The DDS:Crypto:AES-GCM-GMAC authentication plugin, can be activated setting the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)
[`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`dds.sec.crypto.plugin` with the value `builtin.AES-GCM-GMAC`.
Note that since Fast DDS **[Pro]** `v3.7.0`, secure participants must configure the authentication, access control, and
cryptography plugins together.

Moreover, this plugin needs the activation of the [Authentication plugin: DDS:Auth:PKI-DH](../auth_plugin/auth_plugin.md#auth-pki-dh).
The DDS:Crypto:AES-GCM-GMAC plugin is configured using the [Access control plugin: DDS:Access:Permissions](../access_control_plugin/access_control_plugin.md#access-permissions), i.e the cryptography
plugin is configured through the properties and configuration files of the access control plugin.

The following is an example of how to set the properties of DomainParticipantQoS for the DDS:Crypto:AES-GCM-GMAC
configuration.

### C++

```c++
DomainParticipantQos pqos;

// Activate DDS:Crypto:AES-GCM-GMAC plugin
pqos.properties().properties().emplace_back("dds.sec.crypto.plugin",
        "builtin.AES-GCM-GMAC");
```

### XML

```xml
<participant profile_name="secure_domainparticipant_conf_crypto_plugin_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Activate DDS:Crypto:AES-GCM-GMAC plugin -->
                <property>
                    <name>dds.sec.crypto.plugin</name>
                    <value>builtin.AES-GCM-GMAC</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

#### IMPORTANT
Since version v3.7.0, Fast DDS **[Pro]** enforces to configure the three main security plugins
(authentication, access control and cryptography) to enable secure communication.
The previous example focuses only on the authentication plugin, but all three plugins must be configured.
