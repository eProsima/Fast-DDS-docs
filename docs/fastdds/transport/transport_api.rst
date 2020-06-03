.. _transport_transportApi:

Transport API
=============

The following diagram presents the classes defined on the transport API of *eProsima Fast DDS*.
It shows the abstract API interfaces, and the classes required to implement a transport.

.. figure:: /01-figures/transport_api_class_diagram.svg
    :align: center

    Transport API diagram

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

The :ref:`transport_transportApi_transportDescriptor` defines the following data members:

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
check the compatibility of a :ref:`transport_transportApi_locator` with a :class:`Transport`, i.e.,
determine whether a :ref:`transport_transportApi_locator` refers to a :class:`Transport` or not.

Applications do not create the :class:`Transport` instance themselves.
Instead, applications use a :class:`TransportDescriptor` instance to configure the desired transport, and add
this configured instance to the list of user-defined transports of the :ref:`dds_layer_domainParticipant`.
The :ref:`dds_layer_domainParticipant` will use the factory function on the :class:`TransportDescriptor`
to create the :class:`Transport` when required.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CONF-UDP-TRANSPORT-SETTING
    :end-before: //!--
    :dedent: 8

Data members
^^^^^^^^^^^^

The :ref:`transport_transportApi_transport` defines the following data members:

+-------------------------+------------------+-----------------------------------------------------------------------+
| Member                  | Data type        | Description                                                           |
+=========================+==================+=======================================================================+
| ``transport_kind_``     | ``int32_t``      | Unique identifier of the transport type.                              |
+-------------------------+------------------+-----------------------------------------------------------------------+

.. note::

   ``transport_kind_`` is a protected data member for internal use.
   It cannot be accessed nor modified from the public API.
   However, users that are implementing a custom :class:`Transport` need to fill it with a unique constant value
   in the new implementation.

Currently the following identifiers are used in *Fast DDS*:

+---------------------------+----------+-----------------------------------------------------------------------+
| Identifier                | Value    | Transport type                                                        |
+===========================+==========+=======================================================================+
| ``LOCATOR_KIND_RESERVED`` | ``0``    | None. Reserved value for internal use.                                |
+---------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_UDPv4``    | ``1``    | :ref:`transport_udp_udp` over IPv4.                                   |
+---------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_UDPv6``    | ``2``    | :ref:`transport_udp_udp` over IPv6.                                   |
+---------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_TCPv4``    | ``4``    | :ref:`transport_tcp_tcp` over IPv4.                                   |
+---------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_TCPv6``    | ``8``    | :ref:`transport_tcp_tcp` over IPv6.                                   |
+---------------------------+----------+-----------------------------------------------------------------------+
| ``LOCATOR_KIND_SHM``      | ``16``   | :ref:`transport_sharedMemory_sharedMemory`.                           |
+---------------------------+----------+-----------------------------------------------------------------------+


.. _transport_transportApi_locator:

Locator
-------

A :class:`Locator` uniquely identifies a communication channel with a remote peer for a particular transport.
For example, on UDP transports, the :class:`Locator` will contain the information of the IP address and port
of the remote peer.

The :class:`Locator` class is not abstract, and no specializations are implemented for each transport type.
Instead, transports should map the data members of the :class:`Locator` class to their own channel identification
concepts. For example, on :ref:`transport_sharedMemory_sharedMemory` the ``address`` contains a unique ID
for the local host, and the ``port`` represents the shared ring buffer used to communicate buffer descriptors.

Please refer to :ref:`listening_locators` for more information about how to configure
:ref:`dds_layer_domainParticipant` to listen to incoming traffic.

Data members
^^^^^^^^^^^^

The :ref:`transport_transportApi_locator` defines the following data members:

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

