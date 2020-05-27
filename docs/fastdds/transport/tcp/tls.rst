.. _transport_tcp_tls:

TLS over TCP
============

Fast-RTPS allows configuring a TCP Transport to use TLS (Transport Layer Security).
In order to set up TLS, the ``apply_security`` must be set to ``true`` and the ``tls_config``
must be filled with the desired configuration on the :ref:`transport_tcp_transportDescriptor`.

The following is an example of configuration of TLS on the *TCP server*.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TLS-SERVER           |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TLS-SERVER        |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

The corresponding configuration on the *TCP client* is shown in the following example.

+------------------------------------------------------+
| **C++**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp          |
|    :language: c++                                    |
|    :start-after: //CONF-TCP-TLS-CLIENT               |
|    :end-before: //!--                                |
+------------------------------------------------------+
| **XML**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml           |
|    :language: xml                                    |
|    :start-after: <!-->CONF-TCP-TLS-CLIENT            |
|    :end-before: <!--><-->                            |
+------------------------------------------------------+


The following table describes the data members that are configurable on :class:`TLSConfig`.

.. |TLSVerifyMode| replace:: :ref:`transport_tcp_tls_verifyMode`
.. |TLSOptions| replace:: :ref:`transport_tcp_tls_options`
.. |TLSRole| replace:: :ref:`transport_tcp_tls_role`

+--------------------------+----------------------+-------------+-----------------------------------------------------+
| Member                   | Data type            | Default     | Description                                         |
+==========================+======================+=============+=====================================================+
| ``password``             | ``string``           | empty       | Password of the ``private_key_file`` or             |
|                          |                      |             | ``rsa_private_key_file``.                           |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``private_key_file``     | ``string``           | empty       | Path to the private key certificate file.           |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``rsa_private_key_file`` | ``string``           | empty       | Path to the private key RSA certificate file.       |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``cert_chain_file``      | ``string``           | empty       | Path to the public certificate chain file.          |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``tmp_dh_file``          | ``string``           | empty       | Path to the Diffie-Hellman parameters file.         |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_file``          | ``string``           | empty       | Path to the CA (Certification- Authority) file.     |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_mode``          | ``TLSVerifyMode``    | empty       | Establishes the verification mode mask.             |
|                          |                      |             | See |TLSVerifyMode|                                 |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``options``              | ``TLSOptions``       | empty       | Establishes the SSL Context options mask.           |
|                          |                      |             | See |TLSOptions|                                    |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_paths``         | ``vector<string>``   | empty       | Paths where the system will look for                |
|                          |                      |             | verification files.                                 |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_depth``         | ``int32_t``          | empty       | Maximum allowed depth for verifying                 |
|                          |                      |             | intermediate certificates.                          |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``default_verify_path``  | ``bool``             | empty       | Look for verification files on the default paths.   |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``handshake_role``       | ``TLSHandShakeRole`` | ``DEFAULT`` | Role that the transport will take on handshaking.   |
|                          |                      |             | See |TLSRole|                                       |
+--------------------------+----------------------+-------------+-----------------------------------------------------+

.. _transport_tcp_tls_verifyMode:

TLS Verification Mode
---------------------

The verification mode defines how the peer node will be verified.
The following table describes the available verification options.
Several verification options can be combined in the same :ref:`transport_tcp_transportDescriptor`.

+---------------------------------+-----------------------------------------------------------------------------------+
| Value                           | Description                                                                       |
+=================================+===================================================================================+
| ``VERIFY_NONE``                 | Perform no verification.                                                          |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_PEER``                 | Perform verification of the peer.                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_FAIL_IF_NO_PEER_CERT`` | Fail verification if the peer has no certificate.                                 |
|                                 | Ignored unless ``VERIFY_PEER`` is also set.                                       |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_CLIENT_ONCE``          | Do not request client certificate on renegotiation.                               |
|                                 | Ignored unless ``VERIFY_PEER`` is also set.                                       |
+---------------------------------+-----------------------------------------------------------------------------------+


.. _transport_tcp_tls_options:

TLS Options
-----------

These options define which TLS features are to be supported.
The following table describes the available options.
Several options can be combined in the same :ref:`transport_tcp_transportDescriptor`.

+---------------------------------+-----------------------------------------------------------------------------------+
| Value                           | Description                                                                       |
+=================================+===================================================================================+
| ``DEFAULT_WORKAROUNDS``         | Perform no verification.                                                          |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_COMPRESSION``              | Disable compression.                                                              |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_SSLV2``                    | Disable SSL v2.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_SSLV3``                    | Disable SSL v3.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1``                    | Disable TLS v1.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_1``                  | Disable TLS v1.1.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_2``                  | Disable TLS v1.2.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_3``                  | Disable TLS v1.3.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``SINGLE_DH_USE``               | Always create a new key when using *Diffie-Hellman* parameters.                   |
+---------------------------------+-----------------------------------------------------------------------------------+


.. _transport_tcp_tls_role:

TLS Handshake Role
------------------

The role can take the following values:

+---------------------+-----------------------------------------------------------------------------------+
| Value               | Description                                                                       |
+=====================+===================================================================================+
| ``DEFAULT``         | Configured as client if connector, and as server if acceptor                      |
+---------------------+-----------------------------------------------------------------------------------+
| ``CLIENT``          | Configured as client.                                                             |
+---------------------+-----------------------------------------------------------------------------------+
| ``SERVER``          | Configured as server.                                                             |
+---------------------+-----------------------------------------------------------------------------------+



