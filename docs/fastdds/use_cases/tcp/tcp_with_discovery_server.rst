.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-tcp-discovery-server:

TCP Communication with Discovery Server
==========================================

*Fast DDS* :ref:`discovery-server-use-case` consists on a client-server discovery mechanism, in which a server
|DomainParticipant| operates as the central point of communication. It collects and processes the metatraffic
sent by the client DomainParticipants, and then distributes the appropriate information among the rest of
the clients. An extended description of the feature can be found at :ref:`discovery_server`.

To use TCP communication along with Discovery Server, both the server participant and the client participant
need to use custom user transports. There exists several ways of configuring the server participant, being
*Fast DDS* :ref:`ffastddscli_cli` the fastest solution:

.. tabs::

   .. tab:: Fast DDS CLI

      It can be configured to work over a TCP transport layer by using the arguments ``-t`` and ``-q`` to set
      up the IP address and the TCP port, respectively. After sourcing the environment, the following command
      can be used to instantiate a server listening on localhost and port 12345 (see :ref:`ffastddscli_cli`).

      .. code-block:: bash

            fastdds discovery -t 127.0.0.1 -q 12345

   .. tab:: C++

      The following snippet can be used to instantiate a server on IP 192.168.10.57 listening on port 12345.

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //TCP-AND-DISCOVERY-SERVER-SERVER
        :end-before: //!

   .. tab:: XML

      The following snippet can be used to instantiate a server on IP 192.168.10.57 listening on port 12345.

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->TCP-AND-DISCOVERY-SERVER-SERVER<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-40, 42-43

   .. tab:: Fast DDS Discovery Server Example

      It can be configured to work over a TCP transport layer by using the argument ``--transport tcpv4``. The IP
      address and the TCP port can be set up with arguments ``--listening-address`` and ``--listening-port``,
      respectively. From the *DiscoveryServerExample* folder, the following command can be used to instantiate a
      server listening on localhost and port 12345.

      .. code-block:: bash

            ./DiscoveryServerExample server --transport tcpv4 --listening-address 127.0.0.1 --listening-port 12345


The client participant can be configured by either using the ``ROS_DISCOVERY_SERVER`` environment variable (see
:ref:`env_vars_ros_discovery_server`) or by manually setting it.

.. tabs::

   .. tab:: Environment Variable

      To configure a client participant to communicate over the TCP transport layer with the
      ``ROS_DISCOVERY_SERVER`` environment variable, the prefix `TCPv4` needs to be used. The following command
      can be used to configure the variable to set up a client using TCP communication and connecting to a
      server on localhost and port 12345.

      .. code-block:: bash

              export ROS_DISCOVERY_SERVER=TCPv4:[127.0.0.1]:12345

   .. tab:: C++

      The following snippet can be used to instantiate a client that will try to connect to a server on IP
      192.168.10.57 and port 12345, that is, the server instantiated above.

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //TCP-AND-DISCOVERY-SERVER-CLIENT
        :end-before: //!

   .. tab:: XML

      The following snippet can be used to instantiate a client that will try to connect to a server on IP
      192.168.10.57 and port 12345, that is, the server instantiated above.

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->TCP-AND-DISCOVERY-SERVER-CLIENT<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-43, 45-46
