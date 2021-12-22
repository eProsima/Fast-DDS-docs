.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _pkcs11-support:

PKCS#11 support
---------------

The *private key* property used for the DDS:\Auth\:PKI-DH plugin configuration can be specified using
a PKCS#11 compliant URI that represents a key stored in a HSM (Hardware Security Module).
When a PKCS#11 URI is given, the *private key* is never taken out of the HSM, providing a more secure setup.

Support for PKCS#11 URIs is provided by the `libp11 <https://github.com/OpenSC/libp11>`_ library.
This library provides a PKCS#11 engine for OpenSSL that acts as a proxy between OpenSSL
and the HSM driver provided by the manufacturer.
To make OpenSSL aware of the new engine, the OpenSSL configuration file might need to be updated.
For details on how to set up the PKCS#11 engine in different platforms follow the
dedicated documentation:

* :ref:`libp11_sw` on Windows.
* :ref:`libp11_sl` on Linux distributions
