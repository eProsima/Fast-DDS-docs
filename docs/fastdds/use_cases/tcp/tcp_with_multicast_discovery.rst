.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-tcp-multicast:

TCP / SHM Communication with Multicast Discovery
================================================

The following snippets show how to configure *Fast DDS* |DomainParticipants| to run the
:ref:`PDP discovery<disc_phases>` phase over UDP multicast and communicate application data over a
:ref:`transport_tcp_tcp` transport.
With this approach, applications managing large samples can benefit from transmitting their data over TCP or SHM,
while at the same time have the flexibility of automatic discovery.

.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //LARGE_DATA_BUILTIN_TRANSPORTS
        :end-before: //!

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->LARGE_DATA_BUILTIN_TRANSPORTS<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-13, 15-16

.. note::
   ``LARGE_DATA`` configuration of the builtin transports will also create a SHM transport along the UDP and TCP
   transports. Shared Memory will be used whenever it is possible. Manual configuration will be required if a TCP
   communication is required when SHM is feasible.

.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //PDP-MULTICAST-DATA-TCP
        :end-before: //!

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->PDP-MULTICAST-DATA-TCP<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-80, 82-83
