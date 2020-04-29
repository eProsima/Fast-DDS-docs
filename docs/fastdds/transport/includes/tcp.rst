.. _comm-transports-tcp:

TCP Transport
-------------

Unlike UDP, TCP transport is connection oriented and for that Fast-RTPS must establish a TCP connection
before sending the RTPS messages.
Therefore TCP transport can have two behaviors, acting as a server (**TCP Server**) or as a client (**TCP Client**).
The server opens a TCP port listening for incoming connections and the client tries to connect
to the server.
The server and the client concepts are independent from the RTPS concepts: **Publisher**,
**Subscriber**, **Writer**, and **Reader**.
Any of them can operate as a **TCP Server** or a **TCP Client** because
these entities are used only to establish the TCP connection and the RTPS protocol works over it.

To use TCP transports you need to define some more configurations:

You must create a new TCP transport descriptor, for example TCPv4.
This transport descriptor has a field named ``listening_ports`` that indicates to Fast-RTPS
in which physical TCP ports our participant will listen for input connections.
If omitted, the participant will not be able to receive incoming connections but will be able
to connect to other participants that have configured their listening ports.
The transport must be added to the ``userTransports`` list of the participant attributes.
The field ``wan_addr`` can be used to allow incoming connections using the public IP in a WAN environment or the
Internet.
See `WAN or Internet Communication over TCP/IPv4`_ for more information about how to configure a TCP Transport
to allow or connect to WAN connections.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

To configure the participant to connect to another node through TCP, you must configure and add a Locator to its
``initialPeersList`` that points to the remote *listening port*.

+---------------------------------------------------+
| **C++**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp       |
|    :language: c++                                 |
|    :start-after: //CONF-TCP2-TRANSPORT-SETTING    |
|    :end-before: //!--                             |
+---------------------------------------------------+
| **XML**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml        |
|    :language: xml                                 |
|    :start-after: <!-->CONF-TCP2-TRANSPORT-SETTING |
|    :end-before: <!--><-->                         |
+---------------------------------------------------+

A TCP version of helloworld example can be found in this
`link <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C%2B%2B/HelloWorldExampleTCP>`_.


WAN or Internet Communication over TCP/IPv4
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast-RTPS is able to connect through the Internet or other WAN networks when configured properly.
To achieve this kind of scenarios, the involved network devices such as routers and firewalls
should add the rules to allow the communication.

For example, to allow incoming connections through our NAT, Fast-RTPS must be configured as a **TCP Server** listening
to incoming TCP connections.
To allow incoming connections through a WAN, the TCP descriptor associated must indicate
its public IP through its field ``wan_addr``.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

In this case, configuring the router (which public IP is ``80.80.99.45``) is mandatory to allow the incoming traffic to
reach the **TCP Server**.
Typically a NAT routing with the ``listening_port`` ``5100`` to our machine is enough.
Any existing firewall should be configured as well.

In the client side, it is needed to specify the public IP of the **TCP Server** with its ``listening_port`` as
``initial_peer``.

+---------------------------------------------------+
| **C++**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp       |
|    :language: c++                                 |
|    :start-after: //CONF-TCP2-TRANSPORT-SETTING    |
|    :end-before: //!--                             |
+---------------------------------------------------+
| **XML**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml        |
|    :language: xml                                 |
|    :start-after: <!-->CONF-TCP2-TRANSPORT-SETTING |
|    :end-before: <!--><-->                         |
+---------------------------------------------------+

The combination of the above configurations in both **TCP Server** and **TCP Client** allows a scenario similar to
the represented by the following image.

.. image:: /01-figures/TCP_WAN.png
    :align: center

**IPLocator**

IPLocator is an auxiliary static class that offers methods to ease the management of IP based locators, as UDP or TCP.
In TCP, the port field of the locator is divided into physical and logical port.
The physical port is the port used by the network device, the real port that the operating system understands.
The logical port can be seen as RTPS port, or UDP's equivalent port (physical ports of UDP, are logical ports in TCP).
Logical ports normally are not necessary to manage explicitly, but you can do it through IPLocator class.
Physical ports instead, must be set to explicitly use certain ports, to allow the communication through a NAT, for
example.

.. literalinclude:: /../code/CodeTester.cpp
    :language: c++
    :start-after: //CONF-IPLOCATOR-USAGE
    :end-before: //!--

**NOTE**

TCP doesn't support multicast scenarios, so you must plan carefully your network architecture.

.. _TLS:

TLS over TCP
^^^^^^^^^^^^

Fast-RTPS allows configuring a TCP Transport to use TLS (Transport Layer Security)
by setting up **TCP Server** and **TCP Client** properly.

 **TCP Server**

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

 **TCP Client**

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

More TLS related options can be found in the section :ref:`transportdescriptors`.
