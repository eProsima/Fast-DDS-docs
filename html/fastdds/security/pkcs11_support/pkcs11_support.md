<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="pkcs11-support"></a>

<a id="pkcs-11-support"></a>

# PKCS#11 support

The *private key* property used for the DDS:Auth:PKI-DH plugin configuration can be specified using
a PKCS#11 compliant URI that represents a key stored in a HSM (Hardware Security Module).
When a PKCS#11 URI is given, the *private key* is never taken out of the HSM, providing a more secure setup.

Support for PKCS#11 URIs is provided by the [libp11](https://github.com/OpenSC/libp11) library.
This library provides a PKCS#11 engine for OpenSSL that acts as a proxy between OpenSSL
and the HSM driver provided by the manufacturer.
To make OpenSSL aware of the new engine, the OpenSSL configuration file might need to be updated.
For details on how to set up the PKCS#11 engine in different platforms follow the
dedicated documentation:

* [Libp11 and SoftHSM libraries](../../../installation/sources/sources_windows.md#libp11-sw) on Windows.
* [Libp11 and SoftHSM libraries](../../../installation/sources/sources_linux.md#libp11-sl) on Linux distributions
