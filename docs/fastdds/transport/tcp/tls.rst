.. include:: ../../../03-exports/aliases-api.include

.. _transport_tcp_tls:

TLS over TCP
============

.. warning::

   This documentation assumes the reader has basic knowledge of TLS concepts
   since terms like Certificate Authority (CA), Private Key, `Rivest–Shamir–Adleman` (RSA) cryptosystem,
   and Diffie-Hellman encryption protocol are not explained in detail.

*Fast DDS* allows configuring TCP Transports to use TLS (Transport Layer Security).
In order to set up TLS, the :ref:`transport_tcp_transportDescriptor` must
have its |TCPTransportDescriptor::apply_security-api| data member set to ``true``, and its
|TCPTransportDescriptor::tls_config-api| data member filled with the desired configuration on the
|TCPTransportDescriptor-api|.
The following is an example of configuration of TLS on the *TCP server*.

.. tabs::

   .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF-TCP-TLS-SERVER
         :end-before: //!--
         :dedent: 8

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
         :language: xml
         :start-after: <!-->CONF-TCP-TLS-SERVER
         :end-before: <!--><-->
         :lines: 2-3,5-
         :append: </profiles>

The corresponding configuration on the *TCP client* is shown in the following example.

.. tabs::

   .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF-TCP-TLS-CLIENT
         :end-before: //!--
         :dedent: 8

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
         :language: xml
         :start-after: <!-->CONF-TCP-TLS-CLIENT
         :end-before: <!--><-->
         :lines: 2-3,5-
         :append: </profiles>

The following table describes the data members that are configurable on |TCPTransportDescriptor::TLSConfig-api|.

.. |TLSVerifyMode| replace:: :ref:`transport_tcp_tls_verifyMode`
.. |TLSOptions| replace:: :ref:`transport_tcp_tls_options`
.. |TLSRole| replace:: :ref:`transport_tcp_tls_role`

.. _Boost.Asio: https://www.boost.org/doc/libs/1_73_0/doc/html/boost_asio.html
.. _Boost.Asio context: https://www.boost.org/doc/libs/1_73_0/doc/html/boost_asio/reference/ssl__context.html

.. list-table::
   :header-rows: 1
   :align: left

   * - Member
     - Data type
     - Default
     - Description
   * - |TCPTransportDescriptor::TLSConfig::password-api|
     - ``string``
     - ``""``
     - Password of the |TCPTransportDescriptor::TLSConfig::private_key_file-api| or
       |TCPTransportDescriptor::TLSConfig::rsa_private_key_file-api|.
   * - |TCPTransportDescriptor::TLSConfig::private_key_file-api|
     - ``string``
     - ``""``
     - Path to the private key certificate file.
   * - |TCPTransportDescriptor::TLSConfig::rsa_private_key_file-api|
     - ``string``
     - ``""``
     - Path to the private key RSA certificate file.
   * - |TCPTransportDescriptor::TLSConfig::cert_chain_file-api|
     - ``string``
     - ``""``
     - Path to the public certificate chain file.
   * - |TCPTransportDescriptor::TLSConfig::tmp_dh_file-api|
     - ``string``
     - ``""``
     - Path to the Diffie-Hellman parameters file.
   * - |TCPTransportDescriptor::TLSConfig::verify_file-api|
     - ``string``
     - ``""``
     - Path to the CA (Certification- Authority) file.
   * - |TCPTransportDescriptor::TLSConfig::verify_mode-api|
     - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode-api|
     - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::UNUSED-api|
     - Establishes the verification mode mask.
       See |TLSVerifyMode|.
   * - |TCPTransportDescriptor::TLSConfig::options-api|
     - |TCPTransportDescriptor::TLSConfig::TLSOptions-api|
     - |TCPTransportDescriptor::TLSConfig::TLSOptions::NONE-api|
     - Establishes the SSL Context options mask.
       See |TLSOptions|.
   * - |TCPTransportDescriptor::TLSConfig::verify_paths-api|
     - ``vector<string>``
     - ``""``
     - Paths where the system will look for verification files.
   * - |TCPTransportDescriptor::TLSConfig::verify_depth-api|
     - ``int32_t``
     - -1
     - Maximum allowed depth for verifying intermediate certificates.
   * - |TCPTransportDescriptor::TLSConfig::default_verify_path-api|
     - ``bool``
     - ``false``
     - Look for verification files on the default paths.
   * - |TCPTransportDescriptor::TLSConfig::handshake_role-api|
     - |TCPTransportDescriptor::TLSConfig::TLSHandShakeRole-api|
     - |TCPTransportDescriptor::TLSConfig::TLSHandShakeRole::DEFAULT-api|
     - Role that the transport will take on handshaking.
       See |TLSRole|.

.. note::

   *Fast DDS* uses the `Boost.Asio`_ library to handle TLS secure connections.
   These data members are used to build the asio library context, and most of them are mapped directly into this context
   without further manipulation.
   You can find more information about the implications of each member on the `Boost.Asio context`_ documentation.

.. _transport_tcp_tls_verifyMode:

TLS Verification Mode
---------------------

.. _OpenSSL documentation: https://www.openssl.org/docs/man1.0.2/man3/SSL_CTX_set_verify.html

The verification mode defines how the peer node will be verified.
The following table describes the available verification options.
Several verification options can be combined in the same |TCPTransportDescriptor-api|
using the |TCPTransportDescriptor::TLSConfig::add_verify_mode-api| member function.

.. list-table::
   :header-rows: 1
   :align: left

   * - Value
     - Description
   * - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_NONE-api|
     - Perform no verification.
   * - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER-api|
     - Perform verification of the peer.
   * - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT-api|
     - Fail verification if the peer has no certificate.
       Ignored unless |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER-api| is also set.
   * - |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_CLIENT_ONCE-api|
     - Do not request client certificate on renegotiation.
       Ignored unless |TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER-api| is also set.

.. note::

   For a complete description of the different verification modes, please refer to the
   `OpenSSL documentation`_.

.. _transport_tcp_tls_options:

TLS Options
-----------

These options define which TLS features are to be supported.
The following table describes the available options.
Several options can be combined in the same TransportDescriptor
using the |TCPTransportDescriptor::TLSConfig::add_option-api| member function.

.. list-table::
   :header-rows: 1
   :align: left

   * - Value
     - Description
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::DEFAULT_WORKAROUNDS-api|
     - Implement various bug workarounds.
       See `Boost.Asio context`_.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_COMPRESSION-api|
     - Disable compression.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_SSLV2-api|
     - Disable SSL v2.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_SSLV3-api|
     - Disable SSL v3.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_TLSV1-api|
     - Disable TLS v1.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_TLSV1_1-api|
     - Disable TLS v1.1.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_TLSV1_2-api|
     - Disable TLS v1.2.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::NO_TLSV1_3-api|
     - Disable TLS v1.3.
   * - |TCPTransportDescriptor::TLSConfig::TLSOptions::SINGLE_DH_USE-api|
     - Always create a new key when using *Diffie-Hellman* parameters.

.. _transport_tcp_tls_role:

TLS Handshake Role
------------------

The role can take the following values:

.. list-table::
   :header-rows: 1
   :align: left

   * - Value
     - Description
   * - |TCPTransportDescriptor::TLSConfig::TLSHandShakeRole::DEFAULT-api|
     - Configured as client if connector, and as server if acceptor
   * - |TCPTransportDescriptor::TLSConfig::TLSHandShakeRole::CLIENT-api|
     - Configured as client.
   * - |TCPTransportDescriptor::TLSConfig::TLSHandShakeRole::SERVER-api|
     - Configured as server.
