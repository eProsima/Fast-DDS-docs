.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _transport_transportApi:

Transport API
=============

The following diagram presents the classes defined on the transport API of *eProsima Fast DDS*.
It shows the abstract API interfaces, and the classes required to implement a transport.

.. uml::
    :caption: Transport API diagram
    :align: center

    hide empty members
    interface TransportDescriptorInterface
    {
        +uint32_t maxMessageSize
        +uint32_t maxInitialPeersRange
    }
    interface TransportInterface
    {
        #int32_t transport_kind_
    }
    class Locator
    {
        +int32_t kind
        +uint32_t port
        +octet[16] address
    }
    TransportDescriptorInterface <|-- CustomTransportDescriptor
    TransportInterface <|-- CustomTransport
    CustomTransport <--right CustomTransportDescriptor : create
    Locator <--right TransportInterface


.. contents::
    :local:
    :backlinks: none
    :depth: 1


.. _transport_transportApi_transportDescriptor:

TransportDescriptorInterface
----------------------------

Any class that implements the :class:`TransportDescriptorInterface` is known as a :class:`TransportDescriptor`.
It acts as a *builder* for a given transport, meaning that is allows to configure the transport,
and then a new :ref:`Transport <transport_transportApi_transport>` can be built according to this configuration
using its :func:`create_transport` factory member function.

Data members
^^^^^^^^^^^^

The TransportDescriptorInterface defines the following data members:

+--------------------------+------------------+-----------------------------------------------------------------------+
| Member                   | Data type        | Description                                                           |
+==========================+==================+=======================================================================+
| ``maxMessageSize``       | ``uint32_t``     | Maximum size of a single message in the transport.                    |
+--------------------------+------------------+-----------------------------------------------------------------------+
| ``maxInitialPeersRange`` | ``uint32_t``     | Number of channels opened with each initial remote peer               |
+--------------------------+------------------+-----------------------------------------------------------------------+

Any implementation of :ref:`transport_transportApi_transportDescriptor` should add as many
data members as required to full configure the transport it describes.


.. _transport_transportApi_transport:

TransportInterface
------------------

A :class:`Transport` is any class that implements the :class:`TransportInterface`.
It is the object that actually performs the message distribution over a physical transport.

Each :class:`Transport` class defines its own ``transport_kind``, a unique identifier that is used to
check the compatibility of a :ref:`transport_transportApi_locator` with a Transport, i.e.,
determine whether a Locator refers to a Transport or not.

Applications do not create the :class:`Transport` instance themselves.
Instead, applications use a :class:`TransportDescriptor` instance to configure the desired transport, and add
this configured instance to the list of user-defined transports of the :ref:`dds_layer_domainParticipant`.
The DomainParticipant will use the factory function on the :class:`TransportDescriptor`
to create the :class:`Transport` when required.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CONF-UDP-TRANSPORT-SETTING
    :end-before: //!--
    :dedent: 8

Data members
^^^^^^^^^^^^

The TransportInterface defines the following data members:

+-------------------------+------------------+-----------------------------------------------------------------------+
| Member                  | Data type        | Description                                                           |
+=========================+==================+=======================================================================+
| ``transport_kind_``     | ``int32_t``      | Unique identifier of the transport type.                              |
+-------------------------+------------------+-----------------------------------------------------------------------+

.. note::

   ``transport_kind_`` is a protected data member for internal use.
   It cannot be accessed nor modified from the public API.
   However, users that are implementing a custom Transport need to fill it with a unique constant value
   in the new implementation.

Currently the following identifiers are used in *Fast DDS*:

+-----------------------------------+----------+-----------------------------------------------------------------------+
| Identifier                        | Value    | Transport type                                                        |
+===================================+==========+=======================================================================+
| ``LOCATOR_KIND_RESERVED-api``     | ``0``    | None. Reserved value for internal use.                                |
+-----------------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_UDPv4-api``        | ``1``    | :ref:`transport_udp_udp` over IPv4.                                   |
+-----------------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_UDPv6-api``        | ``2``    | :ref:`transport_udp_udp` over IPv6.                                   |
+-----------------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_TCPv4-api``        | ``4``    | :ref:`transport_tcp_tcp` over IPv4.                                   |
+-----------------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_TCPv6-api``        | ``8``    | :ref:`transport_tcp_tcp` over IPv6.                                   |
+-----------------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_SHM-api``          | ``16``   | :ref:`transport_sharedMemory_sharedMemory`.                           |
+-----------------------------------+----------+-----------------------------------------------------------------------+


.. _transport_transportApi_locator:

Locator
-------

A |Locator_t-api| uniquely identifies a communication channel with a remote peer for a particular transport.
For example, on UDP transports, the Locator will contain the information of the IP address and port
of the remote peer.

The Locator class is not abstract, and no specializations are implemented for each transport type.
Instead, transports should map the data members of the Locator class to their own channel identification
concepts. For example, on :ref:`transport_sharedMemory_sharedMemory` the ``address`` contains a unique ID
for the local host, and the ``port`` represents the shared ring buffer used to communicate buffer descriptors.

Please refer to :ref:`listening_locators` for more information about how to configure
DomainParticipant to listen to incoming traffic.

Data members
^^^^^^^^^^^^

The Locator defines the following data members:

+--------------+------------------+-----------------------------------------------------------------------+
| Member       | Data type        | Description                                                           |
+==============+==================+=======================================================================+
| ``kind``     | ``int32_t``      | Unique identifier of the transport type.                              |
+--------------+------------------+-----------------------------------------------------------------------+
| ``port``     | ``uint32_t``     | The channel *port*.                                                   |
+--------------+------------------+-----------------------------------------------------------------------+
| ``address``  | ``octet[16]``    | The channel *address*.                                                |
+--------------+------------------+-----------------------------------------------------------------------+

In TCP, the port of the locator is divided into a physical and a logical port.

* The *physical port* is the port used by the network device, the real port that the operating system understands.
  It is stored in the two least significant bytes of the member ``port``.
* The *logical port* is the RTPS port.
  It is stored in the two most significant bytes of the member ``port``.

In UDP there is only the *physical port*, which is also the RTPS port, and is stored in the two least significant bytes
of the member ``port``.

.. _transport_transportApi_ipLocator:

Configuring IP locators with IPLocator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:class:`IPLocator` is an auxiliary static class that offers methods to manipulate IP based locators.
It is convenient when setting up a new :ref:`transport_udp_udp` or :ref:`transport_tcp_tcp`,
as it simplifies setting IPv4 and IPv6 addresses, or manipulating ports.

For example, normally users configure the physical port and do not need to worry about logical ports.
However, :class:`IPLocator` allows to manage them if needed.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CONF-IPLOCATOR-USAGE
    :end-before: //!--
    :dedent: 8

*Fast DDS* also allows to specify locator addresses using names.
When an address is specified by a name, *Fast DDS* will query the known hosts and available DNS servers to try to
resolve the IP address.
This address will in turn be used to create the listening locator in the case of *server*, or as the address of the
remote *server* in the case of *clients* (and *servers* that connect to other *servers*).

+---------------------------------------------------------------------+
| **C++**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                      |
|    :language: c++                                                   |
|    :start-after: //CONF_SERVER_DNS_LOCATORS                         |
|    :end-before: //!--                                               |
|    :dedent: 8                                                       |
|                                                                     |
+---------------------------------------------------------------------+
| **XML**                                                             |
+---------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                          |
|    :language: xml                                                   |
|    :start-after: <!-->CONF_SERVER_DNS_LOCATORS<-->                  |
|    :end-before: <!--><-->                                           |
|    :dedent: 28                                                      |
+---------------------------------------------------------------------+

.. warning::

    Currently, XML only supports loading IP addresses by name for UDP transport.

.. _transport_transportApi_chaining:

Chaining of transports
----------------------

There are use cases where the user needs to pre-process out-coming information before being sent to network and also
the incoming information after being received.
*Transport API* offers two interfaces for implementing this kind of functionality: :class:`ChainingTransportDescriptor`
and :class:`ChainingTransport`.

.. uml::
    :align: center

    hide empty members
    interface TransportDescriptorInterface
    {
        +uint32_t maxMessageSize
        +uint32_t maxInitialPeersRange
    }
    interface TransportInterface
    {
        #int32_t transport_kind_
    }
    interface ChainingTransportDescriptor
    {
        +std::shared_ptr<TransportDescriptorInterface> low_level_descriptor
    }
    interface ChainingTransport
    {
        #std::unique_ptr<TransportInterface> low_level_transport_
        {abstract} bool send(...)
        {abstract} void receive(...)
    }
    TransportDescriptorInterface <|-- ChainingTransportDescriptor
    TransportInterface <|-- ChainingTransport
    ChainingTransportDescriptor <|-- CustomChainingTransportDescriptor
    ChainingTransport <|-- CustomChainingTransport
    CustomChainingTransport <-- CustomChainingTransportDescriptor : create
    CustomChainingTransportDescriptor "1" *-- "1" TransportDescriptorInterface : contains
    CustomChainingTransport "1" *-- "1" TransportInterface : contains
    CustomChainingTransportDescriptor --- UDPv4TransportDescriptor : example
    CustomChainingTransport ---  UDPv4Transport : example


These extensions allow to implement a new Transport which depends on another one (called here as *low level transport*).
The user can override the `send()` function, pre-processing the out-coming buffer before calling the associated
*low level transport*.
Also, when a incoming buffer arrives to the *low level transport*, this one calls the overridden `receive()`
function to
allow to pre-process the buffer.

ChainingTransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Implementing :class:`ChainingTransportDescriptor` allows to configure the new Transport and set the *low level
transport* on which it depends.
The associated *low level transport* can be any transport which inherits from
:class:`TransportInterface` (including another :class:`ChainingTransport`).


The :class:`ChainingTransportDescriptor` defines the following data members:

+--------------------------+---------------------------------------------------+----------------------------------+
| Member                   | Data type                                         | Description                      |
+==========================+===================================================+==================================+
| ``low_level_descriptor`` | ``std::shared_ptr<TransportDescriptorInterface>`` | Transport descriptor of the *low |
|                          |                                                   | level transport*                 |
+--------------------------+---------------------------------------------------+----------------------------------+

User has to specify the *low level tranport* in the definition of its new custom transport.

.. literalinclude:: ../../../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //CONF-CUSTOM-CHAINING-TRANSPORT-SETTING
   :end-before: //!

ChainingTransport
^^^^^^^^^^^^^^^^^

This interface forces the user to implement `send()` and `receive()` functions.
The idea is to pre-process the buffer and after, call to the next level.

.. literalinclude:: ../../../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //CHAINING_TRANSPORT_OVERRIDE
   :end-before: //!
