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
To make OpenSSL aware of the new engine, the OpenSSL configuration file must be updated.
This file is usually located at */etc/ssl/openssl.cnf* in UNIX systems, and at *c:/{path to openSSL}/bin/openssl.cfg*
on Windows systems.

Add the following line to the top of the configuration file, before any section is defined:

.. code-block::

   openssl_conf = openssl_init

Then add the following sections at the bottom:

.. code-block::

   [openssl_init]
   engines=engine_section

   [engine_section]
   pkcs11 = pkcs11_section

   [pkcs11_section]
   dynamic_path = {path to libpkcs11 library}
   MODULE_PATH = {path to HSM manufacturer library}
   init = 0

This adds a new PKCS#11 engine that can be used with openSSL.
The *dynamic_path* value refers to the library provided by *libp11*,
while the *MODULE_PATH* value refers to the PKCS#11 compatible library
provided by the HSM manufacturer.

For example, on a Linux machine with *libp11* and *SoftHSM* installed on the default paths:

.. code-block::

   [openssl_init]
   engines=engine_section

   [engine_section]
   pkcs11 = pkcs11_section

   [pkcs11_section]
   dynamic_path = /usr/lib/x86_64-linux-gnu/engines-1.1/libpkcs11.so
   MODULE_PATH = /usr/local/lib/softhsm/libsofthsm2.so
   init = 0

