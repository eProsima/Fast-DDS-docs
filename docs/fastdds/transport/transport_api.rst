.. _transport_transportApi:

Transport API
=============

The following diagram presents the classes defined on the transport API of eProsima Fast DDS.
It shows the abstract API interfaces, and the classes required to implement a transport.

.. figure:: /01-figures/transport_api_class_diagram.svg
    :align: center

    Transport API diagram


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
It is the object that actually performs the message distribition over a physical transport.

Each :class:`Transport` class defines its own ``transport_kind``, a unique identifier that is used to
check the compatibility of a :ref:`transport_transportApi_locator` with a :class:`Transport`, i.e.,
determine whether a :ref:`transport_transportApi_locator` refers to a :class:`Transport` or not.

Applications do not create the :class:`Transport` instance themselves.
Instead, applications use a :class:`TransportDescriptor` instance to configure the desired transport, and add
this configured instance to the list of user-defined transports of the :ref:`dds_layer_domainParticipant`.
The :ref:`dds_layer_domainParticipant` will use the factory function on the :class:`TransportDescriptor`
to create the :class:`Transport` when required.


Data members
^^^^^^^^^^^^

The :ref:`transport_transportApi_transport` defines the following data members:

+-------------------------+------------------+-----------------------------------------------------------------------+
| Member                  | Data type        | Description                                                           |
+=========================+==================+=======================================================================+
| ``transport_kind_``     | ``int32_t``      | Unique identifier of the transport type.                              |
+-------------------------+------------------+-----------------------------------------------------------------------+


.. _transport_transportApi_locator:

Locator
-------

A :class:`Locator` uniquely identifies a communication channel with a remote peer for a particular transport.
For example, on UDP transports, the :class:`Locator` will contain the information of the IP address and port
of the remote peer.

The :class:`Locator` class is not abstract, and no specializations are implemented for each trasnport type.
Instead, transports should map the data members of the :class:`Locator` class to their own channel identification
concepts. For example, on :ref:`transport_sharedMemory_sharedMemory` the ``address`` contains a uique ID
for the local host, and the ``port`` represents the shared ring buffer used to communicate buffer descriptors.

On :ref:`listening_locators` you can find more information about how to configure :ref:`dds_layer_domainParticipant`
to listen to incoming traffic.

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




