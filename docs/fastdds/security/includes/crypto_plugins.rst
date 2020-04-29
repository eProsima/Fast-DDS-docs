Cryptographic plugins
---------------------

They provide encryption support. Encryption can be applied over three different levels of RTPS protocol. Cryptographic
plugins can encrypt whole RTPS messages, RTPS submessages of a particular entity (Writer or Reader) or the payload
(user data) of a particular Writer. You can combine them.

You can activate an Cryptographic plugin using Participant property ``dds.sec.crypto.plugin``. Fast RTPS provides a
built-in Cryptographic plugin. More information on :ref:`crypto-aes-gcm-gmac`.

The Cryptographic plugin is configured by the Access control plugin.
If Access control will not be used, you can configure the Cryptographic plugin manually with the next properties:

**Encrypt whole RTPS messages**

You can configure a Participant to encrypt all RTPS messages using Participant property
``rtps.participant.rtps_protection_kind`` with the value ``ENCRYPT``.

**Encrypt RTPS submessages of a particular entity**

You can configure an entity (Writer or Reader) to encrypt its RTPS submessages using Entity property
``rtps.endpoint.submessage_protection_kind`` with the value ``ENCRYPT``.

**Encrypt payload of a particular Writer**

You can configure a Writer to encrypt its payload using Writer property ``rtps.endpoint.payload_protection_kind`` with
the value ``ENCRYPT``.
